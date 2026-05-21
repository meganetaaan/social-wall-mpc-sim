import { transitionBelief } from '../belief/beliefTransition'
import { observePointReturns } from '../belief/pointObjectBelief'
import { predictHumans, stepRobotInEnvironment } from '../simulation/dynamics'
import { mulberry32, normAngle } from '../simulation/math'
import type {
  BeliefState,
  CandidateRollout,
  ControlInput,
  CostBreakdown,
  Environment,
  HumanState,
  PlannerParameters,
  PointObservation,
  RobotState,
} from '../simulation/types'
import { addCost, evaluateStageCost, terminalBeliefGoalCost } from './cost'

const MAX_OBSERVATION_SAMPLES = 5
const OBSERVATION_KEEP_PROBABILITY = 0.82
const OBSERVATION_JITTER_METERS = 0.035

const zeroCost = (): CostBreakdown => ({
  terms: {
    wall: 0,
    wallHeading: 0,
    human: 0,
    collision: 0,
    control: 0,
    smoothness: 0,
    progress: 0,
    goalProgress: 0,
    goalTerminal: 0,
    uncertainty: 0,
    mapUncertainty: 0,
    observationGain: 0,
    wallBeliefConsistency: 0,
  },
  total: 0,
})

function normalizeSampleCount(value: number | undefined): number {
  if (!Number.isFinite(value)) return 1
  return Math.min(MAX_OBSERVATION_SAMPLES, Math.max(1, Math.floor(value ?? 1)))
}

function jitterPointObservations(observations: PointObservation[], rand: () => number): PointObservation[] {
  return observations.flatMap((observation) => {
    if (rand() > OBSERVATION_KEEP_PROBABILITY) return []
    const dx = (rand() - 0.5) * 2 * OBSERVATION_JITTER_METERS
    const dy = (rand() - 0.5) * 2 * OBSERVATION_JITTER_METERS
    const point = { x: observation.point.x + dx, y: observation.point.y + dy }
    const range = Math.hypot(point.x - observation.sensorPose.x, point.y - observation.sensorPose.y)
    const bearing = normAngle(
      Math.atan2(point.y - observation.sensorPose.y, point.x - observation.sensorPose.x) - observation.sensorPose.theta,
    )
    return [{ ...observation, point, range, bearing }]
  })
}

function rolloutCandidateWithObservationSample(args: {
  robot: RobotState
  humans: HumanState[]
  environment: Environment
  belief: BeliefState
  controls: ControlInput[]
  previousControl: ControlInput
  parameters: PlannerParameters
  seed: number
}): CandidateRollout {
  const rand = mulberry32(args.seed)
  const trajectory: RobotState[] = [args.robot]
  const predictedHumans: HumanState[][] = [args.humans]
  let robot = args.robot
  let humans = args.humans
  let belief = args.belief
  let previousControl = args.previousControl
  let rolloutTime = 0
  let cost = zeroCost()

  for (const control of args.controls) {
    const stage = evaluateStageCost({
      robot,
      humans,
      environment: args.environment,
      belief,
      control,
      previousControl,
      parameters: args.parameters,
    })
    cost = addCost(cost, stage)
    robot = stepRobotInEnvironment(robot, control, args.parameters.dt, args.environment, args.parameters)
    humans = predictHumans(humans, args.parameters.dt, undefined, robot, rolloutTime)
    rolloutTime += args.parameters.dt
    const pointObservations = jitterPointObservations(
      observePointReturns({
        robot,
        environment: args.environment,
        humans,
        parameters: args.parameters,
        time: rolloutTime,
      }),
      rand,
    )
    belief = transitionBelief({
      belief,
      control,
      observedRobot: robot,
      environment: args.environment,
      parameters: args.parameters,
      time: rolloutTime,
      pointObservations,
      mapUpdateMode: 'anonymous-line-features',
    })
    trajectory.push(robot)
    predictedHumans.push(humans)
    previousControl = control
  }

  const goalTerminal = terminalBeliefGoalCost(robot, args.environment, belief, args.parameters)
  cost = {
    terms: { ...cost.terms, goalTerminal: cost.terms.goalTerminal + goalTerminal },
    total: cost.total + goalTerminal,
  }

  return { controls: args.controls, trajectory, predictedHumans, cost }
}

function averageCosts(costs: CostBreakdown[]): CostBreakdown {
  const totalWeight = 1 / costs.length
  const averaged = zeroCost()

  for (const cost of costs) {
    averaged.total += cost.total * totalWeight
    for (const key of Object.keys(averaged.terms) as Array<keyof CostBreakdown['terms']>) {
      averaged.terms[key] += cost.terms[key] * totalWeight
    }
  }

  return averaged
}

export function rolloutCandidateWithFutureObservations(args: {
  robot: RobotState
  humans: HumanState[]
  environment: Environment
  belief: BeliefState
  controls: ControlInput[]
  previousControl: ControlInput
  parameters: PlannerParameters
  observationSamples?: number
  seed?: number
}): {
  rollout: CandidateRollout
  samples: Array<{ seed: number; rollout: CandidateRollout }>
} {
  const sampleCount = normalizeSampleCount(args.observationSamples)
  const baseSeed = args.seed ?? args.parameters.seed
  const samples = Array.from({ length: sampleCount }, (_, index) => {
    const seed = baseSeed + index
    return {
      seed,
      rollout: rolloutCandidateWithObservationSample({ ...args, seed }),
    }
  })
  const representative = samples[0].rollout

  return {
    rollout: {
      ...representative,
      cost: averageCosts(samples.map((sample) => sample.rollout.cost)),
    },
    samples,
  }
}
