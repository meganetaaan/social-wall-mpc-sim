import { updateBelief } from '../belief/simpleBelief'
import { predictHumans, stepRobot } from '../simulation/dynamics'
import type {
  BeliefState,
  CandidateRollout,
  ControlInput,
  Environment,
  HumanState,
  PlannerParameters,
  RobotState,
} from '../simulation/types'
import { addCost, evaluateStageCost, terminalGoalCost } from './cost'

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
    robot = stepRobot(robot, control, args.parameters.dt)
    humans = predictHumans(humans, args.parameters.dt)
    belief = updateBelief(belief, robot, args.environment, args.parameters)
    trajectory.push(robot)
    predictedHumans.push(humans)
    previousControl = control
  }

  const goalTerminal = terminalGoalCost(robot, args.environment, args.parameters)
  cost = {
    terms: { ...cost.terms, goalTerminal: cost.terms.goalTerminal + goalTerminal },
    total: cost.total + goalTerminal,
  }

  return { controls: args.controls, trajectory, predictedHumans, cost }
}
