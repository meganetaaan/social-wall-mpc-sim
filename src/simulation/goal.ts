import { distance } from './math'
import type { Environment, RobotState } from './types'

export const defaultGoalRadius = 0.2

export function goalRadius(environment: Environment) {
  return environment.goalRadius ?? defaultGoalRadius
}

export function goalDistance(robot: RobotState, environment: Environment) {
  return distance(robot, environment.goal)
}

export function isGoalReached(robot: RobotState, environment: Environment) {
  return goalDistance(robot, environment) <= goalRadius(environment)
}
