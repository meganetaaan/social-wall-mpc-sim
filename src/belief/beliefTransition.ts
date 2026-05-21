import { nearestWall, normAngle } from '../simulation/math'
import type {
  BeliefState,
  ControlInput,
  Environment,
  MapUpdateMode,
  PlannerParameters,
  PointObservation,
  PoseGaussian,
  RobotState,
  WallBelief,
} from '../simulation/types'
import { updateEstimatedLineFeatures } from './lineFeatureMap'
import { updateObjectBeliefs } from './pointObjectBelief'
import {
  correctPoseGaussianWithObservation,
  poseGaussianFromSigmas,
  propagatePoseGaussian,
  sigmasFromPoseGaussian,
} from './poseBelief'
import {
  computeMapUncertainty,
  initializeEstimatedWallBelief,
  observeWalls,
  updateEstimatedWallBelief,
} from './wallMapBelief'

const clamp01 = (value: number) => Math.min(1, Math.max(0, value))

function completeWallBeliefs(belief: BeliefState, environment: Environment): WallBelief[] {
  return environment.walls.map((wall) => {
    const existing = belief.wallBeliefs.find((candidate) => candidate.wallId === wall.id)
    return existing ?? { wallId: wall.id, confidence: belief.mapConfidence * 0.6, lastObservedAt: -1 }
  })
}

function normalizePoseBelief(belief: BeliefState, robot: RobotState): PoseGaussian {
  return belief.pose ?? poseGaussianFromSigmas(robot, belief.sigmaX, belief.sigmaY, belief.sigmaTheta)
}

export function inferControl(previous: RobotState, robot: RobotState, dt: number): ControlInput {
  const safeDt = Math.max(0.001, Math.abs(dt))
  return {
    v: Math.hypot(robot.x - previous.x, robot.y - previous.y) / safeDt,
    omega: normAngle(robot.theta - previous.theta) / safeDt,
  }
}

export function predictBelief(args: {
  belief: BeliefState
  control: ControlInput
  parameters: PlannerParameters
}): BeliefState {
  const pose = normalizePoseBelief(args.belief, args.belief.pose.mean)
  const propagatedPose = propagatePoseGaussian(pose, args.control, args.parameters)
  const sigmas = sigmasFromPoseGaussian(propagatedPose)
  return {
    ...args.belief,
    pose: propagatedPose,
    ...sigmas,
  }
}

export function updateBeliefWithObservation(args: {
  belief: BeliefState
  robot: RobotState
  environment: Environment
  parameters: PlannerParameters
  time?: number
  pointObservations?: PointObservation[]
  mapUpdateMode?: MapUpdateMode
}): BeliefState {
  const time = args.time ?? 0
  const pointObservations = args.pointObservations ?? []
  const nearest = nearestWall({ x: args.robot.x, y: args.robot.y }, args.environment.walls)
  const observationStrength = Math.max(0, 1 - nearest.distance / 2.2)
  const correctedPose = correctPoseGaussianWithObservation(
    { ...args.belief.pose, mean: { ...args.robot } },
    observationStrength,
  )
  const sigmas = sigmasFromPoseGaussian(correctedPose)
  const observations = observeWalls(args.robot, args.environment, args.parameters)
  const mapUpdateMode = args.mapUpdateMode ?? 'true-id-coverage'
  const estimatedWalls =
    mapUpdateMode === 'true-id-coverage'
      ? updateEstimatedWallBelief(
          initializeEstimatedWallBelief(args.environment, args.belief),
          args.environment,
          observations,
          time,
        )
      : args.belief.estimatedWalls
  const estimatedFeatures =
    mapUpdateMode === 'anonymous-line-features'
      ? updateEstimatedLineFeatures({ existing: args.belief.estimatedFeatures ?? [], observations, time })
      : args.belief.estimatedFeatures
  const wallBeliefs = completeWallBeliefs(args.belief, args.environment).map((wallBelief) => {
    const wall = args.environment.walls.find((candidate) => candidate.id === wallBelief.wallId)
    if (!wall) return wallBelief
    const estimated = estimatedWalls.find((candidate) => candidate.wallId === wall.id)
    const confidence = clamp01(Math.max(wallBelief.confidence * 0.996, estimated?.confidence ?? 0))
    const observed = observations.some((candidate) => candidate.wallId === wallBelief.wallId)
    return {
      wallId: wallBelief.wallId,
      confidence,
      lastObservedAt: observed ? time : wallBelief.lastObservedAt,
    }
  })
  const mapConfidence = 1 - computeMapUncertainty(estimatedWalls)
  const objectBeliefs = updateObjectBeliefs(args.belief.objectBeliefs ?? [], pointObservations, time)
  return {
    pose: correctedPose,
    ...sigmas,
    mapConfidence,
    wallBeliefs,
    estimatedWalls,
    estimatedFeatures,
    objectBeliefs,
  }
}

export function transitionBelief(args: {
  belief: BeliefState
  control: ControlInput
  observedRobot: RobotState
  environment: Environment
  parameters: PlannerParameters
  time?: number
  pointObservations?: PointObservation[]
  mapUpdateMode?: MapUpdateMode
}): BeliefState {
  return updateBeliefWithObservation({
    belief: predictBelief({ belief: args.belief, control: args.control, parameters: args.parameters }),
    robot: args.observedRobot,
    environment: args.environment,
    parameters: args.parameters,
    time: args.time,
    pointObservations: args.pointObservations,
    mapUpdateMode: args.mapUpdateMode,
  })
}
