import { tracePoseCovariance } from '../belief/poseBelief'
import { wallAssociationAccuracy } from '../belief/wallAssociation'
import { goalDistance, goalRadius, isGoalReached } from './goal'
import { clamp, distance, length, nearestWall, normAngle, sub, wallTangentAngle } from './math'
import type {
  BeliefState,
  ControlInput,
  Environment,
  HumanState,
  PlannerParameters,
  RobotState,
  SimulationMetrics,
  WallObservation,
} from './types'

const stopVelocityThreshold = 0.05
const minPoseVariance = 1e-4

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

export function mapKnowledgeError(environment: Environment, belief: BeliefState) {
  const totalLength = environment.walls.reduce((sum, wall) => sum + length(sub(wall.b, wall.a)), 0)
  if (totalLength <= 0) return 0
  const knownLength = environment.walls.reduce((sum, wall) => {
    const estimated = belief.estimatedWalls.find((candidate) => candidate.wallId === wall.id)
    if (!estimated) return sum
    const coverage = clamp(estimated.tMax - estimated.tMin, 0, 1)
    const confidence = clamp(estimated.confidence, 0, 1)
    return sum + length(sub(wall.b, wall.a)) * coverage * confidence
  }, 0)
  return clamp(1 - knownLength / totalLength, 0, 1)
}

export function createInitialMetrics(
  state: {
    robot: RobotState
    humans: HumanState[]
    environment: Environment
    belief: BeliefState
    currentObservations?: WallObservation[]
    costBreakdown?: { total: number }
  },
  parameters: Pick<PlannerParameters, 'dWallTarget'>,
): SimulationMetrics {
  const wallError = wallDistanceError(state.robot, state.environment, parameters)
  const currentGoalDistance = goalDistance(state.robot, state.environment)
  const reached = isGoalReached(state.robot, state.environment)
  const poseErrors = poseInferenceErrors(state.robot, state.belief)
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
    ...poseErrors,
    mapKnowledgeError: mapKnowledgeError(state.environment, state.belief),
    wallAssociationAccuracy: wallAssociationAccuracy(state.currentObservations ?? [], state.environment.walls),
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
  currentObservations?: WallObservation[]
  parameters: PlannerParameters
}): SimulationMetrics {
  const {
    previous,
    previousRobot,
    robot,
    humans,
    environment,
    belief,
    control,
    selectedCost,
    currentObservations,
    parameters,
  } = args
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
  const poseErrors = poseInferenceErrors(robot, belief)

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
    ...poseErrors,
    mapKnowledgeError: mapKnowledgeError(environment, belief),
    wallAssociationAccuracy: wallAssociationAccuracy(currentObservations ?? [], environment.walls),
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

function poseInferenceErrors(robot: RobotState, belief: BeliefState) {
  const mean = belief.pose?.mean ?? robot
  const dx = robot.x - mean.x
  const dy = robot.y - mean.y
  const dtheta = normAngle(robot.theta - mean.theta)
  const covariance = belief.pose?.covariance
  const varX = Math.max(minPoseVariance, covariance?.[0]?.[0] ?? belief.sigmaX)
  const varY = Math.max(minPoseVariance, covariance?.[1]?.[1] ?? belief.sigmaY)
  const varTheta = Math.max(minPoseVariance, covariance?.[2]?.[2] ?? belief.sigmaTheta)

  return {
    posePositionError: Math.hypot(dx, dy),
    poseHeadingError: Math.abs(dtheta),
    poseNormalizedError: Math.sqrt((dx * dx) / varX + (dy * dy) / varY + (dtheta * dtheta) / varTheta),
  }
}
