import { transitionBelief } from '../belief/beliefTransition'
import { predictHumans, stepRobotInEnvironment } from '../simulation/dynamics'
import type {
  BeliefState,
  CandidateRollout,
  ControlInput,
  Environment,
  HumanState,
  PlannerParameters,
  RobotState,
} from '../simulation/types'
import { addCost, evaluateStageCost, terminalBeliefGoalCost } from './cost'

const zeroCost = () => ({
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

export function rolloutCandidate(args: {
  robot: RobotState
  humans: HumanState[]
  environment: Environment
  belief: BeliefState
  controls: ControlInput[]
  previousControl: ControlInput
  parameters: PlannerParameters
}): CandidateRollout {
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
    belief = transitionBelief({
      belief,
      control,
      observedRobot: robot,
      environment: args.environment,
      parameters: args.parameters,
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
