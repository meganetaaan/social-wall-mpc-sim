import { tracePoseCovariance } from '../belief/poseBelief'
import { goalDistance, goalRadius, isGoalReached } from './goal'
import { distance, length, nearestWall, sub, wallTangentAngle } from './math'
import type {
  BeliefState,
  ControlInput,
  Environment,
  HumanState,
  PlannerParameters,
  RobotState,
  SimulationMetrics,
} from './types'

const stopVelocityThreshold = 0.05

export function estimatedMapCoverage(environment: Environment, belief: BeliefState) {
  const totalLength = environment.walls.reduce((sum, wall) => sum + length(sub(wall.b, wall.a)), 0)
  if (totalLength <= 0) return 0
  const coveredLength = environment.walls.reduce((sum, wall) => {
    const estimated = belief.estimatedWalls.find((candidate) => candidate.wallId === wall.id)
    if (!estimated) return sum
    const coverage = Math.max(0, Math.min(1, estimated.tMax - estimated.tMin))
    return sum + length(sub(wall.b, wall.a)) * coverage
  }, 0)
  return coveredLength / totalLength
}

export function createInitialMetrics(
  state: {
    robot: RobotState
    humans: HumanState[]
    environment: Environment
    belief: BeliefState
    costBreakdown?: { total: number }
  },
  parameters: Pick<PlannerParameters, 'dWallTarget'>,
): SimulationMetrics {
  const wallError = wallDistanceError(state.robot, state.environment, parameters)
  const currentGoalDistance = goalDistance(state.robot, state.environment)
  const reached = isGoalReached(state.robot, state.environment)
  return {
    elapsedTime: 0,
    meanWallDistanceError: Math.abs(wallError),
    maxWallDistanceError: Math.abs(wallError),
    minHumanDistance: minHumanDistance(state.robot, state.humans),
    socialViolationCount: 0,
    nearCollisionCount: 0,
    stopDuration: 0,
    progressAlongWall: 0,
    uncertaintyTrace: uncertaintyTrace(state.belief),
    estimatedMapCoverage: estimatedMapCoverage(state.environment, state.belief),
    selectedCost: state.costBreakdown?.total ?? 0,
    goalDistance: currentGoalDistance,
    goalReached: reached,
    timeToGoal: reached ? 0 : null,
    bestGoalDistance: currentGoalDistance,
  }
}

export function updateSimulationMetrics(args: {
  previous: SimulationMetrics
  previousRobot: RobotState
  robot: RobotState
  humans: HumanState[]
  environment: Environment
  belief: BeliefState
  control: ControlInput
  selectedCost: number
  parameters: PlannerParameters
}): SimulationMetrics {
  const { previous, previousRobot, robot, humans, environment, belief, control, selectedCost, parameters } = args
  const elapsedTime = previous.elapsedTime + parameters.dt
  const stepCount = Math.max(1, Math.round(elapsedTime / parameters.dt))
  const currentWallError = Math.abs(wallDistanceError(robot, environment, parameters))
  const humanDistance = minHumanDistance(robot, humans)
  const nearest = nearestWall({ x: robot.x, y: robot.y }, environment.walls)
  const nearObstacle = environment.obstacles.some(
    (obstacle) =>
      distance(robot, obstacle) - obstacle.radius - parameters.robotRadius < parameters.wallCollisionDistance,
  )
  const tangent = wallTangentAngle(nearest.wall)
  const dx = robot.x - previousRobot.x
  const dy = robot.y - previousRobot.y
  const progress = Math.max(0, dx * Math.cos(tangent) + dy * Math.sin(tangent))
  const currentGoalDistance = goalDistance(robot, environment)
  const reachedThisStep = currentGoalDistance <= goalRadius(environment)
  const goalReached = previous.goalReached || reachedThisStep

  return {
    elapsedTime,
    meanWallDistanceError:
      (previous.meanWallDistanceError * (stepCount - 1) + currentWallError) / Math.max(1, stepCount),
    maxWallDistanceError: Math.max(previous.maxWallDistanceError, currentWallError),
    minHumanDistance: Math.min(previous.minHumanDistance, humanDistance),
    socialViolationCount: previous.socialViolationCount + (humanDistance < parameters.dMin ? 1 : 0),
    nearCollisionCount:
      previous.nearCollisionCount + (nearest.distance < parameters.wallCollisionDistance || nearObstacle ? 1 : 0),
    stopDuration: previous.stopDuration + (Math.abs(control.v) < stopVelocityThreshold ? parameters.dt : 0),
    progressAlongWall: previous.progressAlongWall + progress,
    uncertaintyTrace: uncertaintyTrace(belief),
    estimatedMapCoverage: estimatedMapCoverage(environment, belief),
    selectedCost,
    goalDistance: currentGoalDistance,
    goalReached,
    timeToGoal: previous.timeToGoal ?? (reachedThisStep ? elapsedTime : null),
    bestGoalDistance: Math.min(previous.bestGoalDistance, currentGoalDistance),
  }
}

function wallDistanceError(
  robot: RobotState,
  environment: Environment,
  parameters: Pick<PlannerParameters, 'dWallTarget'>,
) {
  return nearestWall({ x: robot.x, y: robot.y }, environment.walls).distance - parameters.dWallTarget
}

function minHumanDistance(robot: RobotState, humans: HumanState[]) {
  if (humans.length === 0) return Number.POSITIVE_INFINITY
  return Math.min(...humans.map((human) => distance(robot, human)))
}

function uncertaintyTrace(belief: BeliefState) {
  return belief.pose ? tracePoseCovariance(belief.pose) : belief.sigmaX + belief.sigmaY + belief.sigmaTheta
}
