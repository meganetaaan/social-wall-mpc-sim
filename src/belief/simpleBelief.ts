import { nearestWall } from '../simulation/math'
import type { BeliefState, Environment, PlannerParameters, RobotState, WallBelief } from '../simulation/types'
import {
  computeMapUncertainty,
  estimatedWallConfidence,
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

export function updateBelief(
  belief: BeliefState,
  robot: RobotState,
  environment: Environment,
  parameters: PlannerParameters,
  time = 0,
): BeliefState {
  const nearest = nearestWall({ x: robot.x, y: robot.y }, environment.walls)
  const observationStrength = Math.max(0, 1 - nearest.distance / 2.2)
  const motionGrowth = 0.006 + Math.abs(parameters.dt) * 0.004
  const correction = 0.02 * observationStrength
  const observations = observeWalls(robot, environment, parameters)
  const estimatedWalls = updateEstimatedWallBelief(
    initializeEstimatedWallBelief(environment, belief),
    environment,
    observations,
    time,
  )
  const wallBeliefs = completeWallBeliefs(belief, environment).map((wallBelief) => {
    const wall = environment.walls.find((candidate) => candidate.id === wallBelief.wallId)
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
  return {
    sigmaX: Math.max(0.02, belief.sigmaX + motionGrowth - correction),
    sigmaY: Math.max(0.02, belief.sigmaY + motionGrowth - correction),
    sigmaTheta: Math.max(0.01, belief.sigmaTheta + motionGrowth * 0.5 - correction * 0.35),
    mapConfidence,
    wallBeliefs,
    estimatedWalls,
  }
}

export const traceSigma = (belief: BeliefState) => belief.sigmaX + belief.sigmaY + belief.sigmaTheta

export const mapUncertaintyTrace = (belief: BeliefState) => {
  if (belief.estimatedWalls.length > 0) return computeMapUncertainty(belief.estimatedWalls)
  if (belief.wallBeliefs.length === 0) return 1 - belief.mapConfidence
  return belief.wallBeliefs.reduce((sum, wall) => sum + (1 - wall.confidence), 0) / belief.wallBeliefs.length
}

export const wallConfidence = (belief: BeliefState, wallId: string) => estimatedWallConfidence(belief, wallId)
