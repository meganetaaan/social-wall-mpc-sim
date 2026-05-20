import { clamp, distance, dot, mulberry32, nearestPointOnSegment, normAngle, sub } from '../simulation/math'
import type {
  BeliefState,
  Environment,
  EstimatedWallSegment,
  PlannerParameters,
  RobotState,
  Vec2,
  WallObservation,
  WallSegment,
} from '../simulation/types'

const clamp01 = (value: number) => clamp(value, 0, 1)
const MIN_OBSERVATION_SIGMA = 1e-6
const segmentPoint = (wall: WallSegment, t: number): Vec2 => ({
  x: wall.a.x + (wall.b.x - wall.a.x) * t,
  y: wall.a.y + (wall.b.y - wall.a.y) * t,
})
const LOS_EPSILON = 1e-7

export type ObservationLikelihood = {
  rangeResidual: number
  bearingResidual: number
  rangeSigma: number
  bearingSigma: number
  likelihood: number
  logLikelihood: number
}

export type WallAssociation = {
  wallId: string | null
  likelihood: ObservationLikelihood
  affinity: number
}

function hashObservationSeed(input: string) {
  let hash = 2166136261
  for (let i = 0; i < input.length; i += 1) {
    hash ^= input.charCodeAt(i)
    hash = Math.imul(hash, 16777619)
  }
  return hash >>> 0
}

function quantized(value: number) {
  return Math.round(value * 1000)
}

export function observationNoiseScales(p: PlannerParameters) {
  return {
    rangeStdDev: Math.max(0.015, p.sensorRadius * 0.015),
    bearingStdDev: Math.max(0.005, p.sensorFov * 0.01),
  }
}

export function expectedRangeBearingToWall(
  robot: RobotState,
  wall: WallSegment,
): { range: number; bearing: number; point: Vec2 } {
  const point = nearestPointOnSegment(robot, wall)
  return {
    range: distance(robot, point),
    bearing: normAngle(Math.atan2(point.y - robot.y, point.x - robot.x) - robot.theta),
    point,
  }
}

export function wallObservationLikelihood(
  observation: Pick<WallObservation, 'range' | 'bearing' | 'rangeStdDev' | 'bearingStdDev'>,
  robot: RobotState,
  wall: WallSegment,
): ObservationLikelihood {
  const expected = expectedRangeBearingToWall(robot, wall)
  const rangeSigma = Math.max(observation.rangeStdDev, MIN_OBSERVATION_SIGMA)
  const bearingSigma = Math.max(observation.bearingStdDev, MIN_OBSERVATION_SIGMA)
  const rangeResidual = observation.range - expected.range
  const bearingResidual = normAngle(observation.bearing - expected.bearing)
  const mahalanobisSquared = (rangeResidual / rangeSigma) ** 2 + (bearingResidual / bearingSigma) ** 2
  const logLikelihood = -0.5 * mahalanobisSquared - Math.log(2 * Math.PI * rangeSigma * bearingSigma)
  return {
    rangeResidual,
    bearingResidual,
    rangeSigma,
    bearingSigma,
    likelihood: Math.exp(logLikelihood),
    logLikelihood,
  }
}

export function wallObservationAffinity(
  observation: Pick<WallObservation, 'range' | 'bearing' | 'rangeStdDev' | 'bearingStdDev'>,
  robot: RobotState,
  wall: WallSegment,
) {
  const likelihood = wallObservationLikelihood(observation, robot, wall)
  const mahalanobisSquared =
    (likelihood.rangeResidual / likelihood.rangeSigma) ** 2 +
    (likelihood.bearingResidual / likelihood.bearingSigma) ** 2
  return clamp01(Math.exp(-0.5 * mahalanobisSquared))
}

export function associateWallObservation(
  observation: Pick<WallObservation, 'range' | 'bearing' | 'rangeStdDev' | 'bearingStdDev' | 'sensorPose'>,
  candidateWalls: WallSegment[],
): WallAssociation {
  let best: WallAssociation | undefined
  for (const wall of candidateWalls) {
    const likelihood = wallObservationLikelihood(observation, observation.sensorPose, wall)
    const affinity = wallObservationAffinity(observation, observation.sensorPose, wall)
    if (!best || likelihood.logLikelihood > best.likelihood.logLikelihood) {
      best = { wallId: wall.id, likelihood, affinity }
    }
  }

  return (
    best ?? {
      wallId: null,
      likelihood: {
        rangeResidual: Number.POSITIVE_INFINITY,
        bearingResidual: Number.POSITIVE_INFINITY,
        rangeSigma: 1,
        bearingSigma: 1,
        likelihood: 0,
        logLikelihood: Number.NEGATIVE_INFINITY,
      },
      affinity: 0,
    }
  )
}

export function wallAssociationAccuracy(
  observations: Pick<
    WallObservation,
    'wallId' | 'range' | 'bearing' | 'rangeStdDev' | 'bearingStdDev' | 'sensorPose'
  >[],
  candidateWalls: WallSegment[],
) {
  if (observations.length === 0) return 1
  const correct = observations.filter(
    (observation) => associateWallObservation(observation, candidateWalls).wallId === observation.wallId,
  ).length
  return correct / observations.length
}

export function noisyRangeBearingMeasurement(robot: RobotState, point: Vec2, p: PlannerParameters, key: string) {
  const { rangeStdDev, bearingStdDev } = observationNoiseScales(p)
  const seed = hashObservationSeed(
    [key, quantized(robot.x), quantized(robot.y), quantized(robot.theta), quantized(point.x), quantized(point.y)].join(
      ':',
    ),
  )
  const random = mulberry32(seed)
  const rangeNoise = (random() * 2 - 1) * 2 * rangeStdDev
  const bearingNoise = (random() * 2 - 1) * 2 * bearingStdDev
  const trueRange = distance(robot, point)
  const trueBearing = normAngle(Math.atan2(point.y - robot.y, point.x - robot.x) - robot.theta)
  return {
    range: Math.max(0, trueRange + rangeNoise),
    bearing: normAngle(trueBearing + bearingNoise),
    rangeStdDev,
    bearingStdDev,
  }
}

const cross = (a: Vec2, b: Vec2) => a.x * b.y - a.y * b.x

function lineOfSightParameter(origin: Vec2, target: Vec2, wall: WallSegment) {
  const ray = sub(target, origin)
  const wallVector = sub(wall.b, wall.a)
  const denominator = cross(ray, wallVector)
  if (Math.abs(denominator) < LOS_EPSILON) return undefined

  const offset = sub(wall.a, origin)
  const rayT = cross(offset, wallVector) / denominator
  const wallT = cross(offset, ray) / denominator
  if (wallT <= LOS_EPSILON || wallT >= 1 - LOS_EPSILON) return undefined
  return rayT
}

function isLineOfSightBlocked(origin: Vec2, target: Vec2, wall: WallSegment) {
  const rayT = lineOfSightParameter(origin, target, wall)
  return rayT !== undefined && rayT > LOS_EPSILON && rayT < 1 - LOS_EPSILON
}

function hasLineOfSight(origin: Vec2, target: Vec2, targetWall: WallSegment, walls: WallSegment[]) {
  return !walls.some((blocker) => blocker.id !== targetWall.id && isLineOfSightBlocked(origin, target, blocker))
}

function parameterForPoint(wall: WallSegment, point: Vec2) {
  const ab = sub(wall.b, wall.a)
  const denom = dot(ab, ab) || 1
  return clamp01(dot(sub(point, wall.a), ab) / denom)
}

function observationStrength(robot: RobotState, point: Vec2, p: PlannerParameters) {
  const rangeRatio = clamp01(1 - distance(robot, point) / p.sensorRadius)
  const bearing = Math.atan2(point.y - robot.y, point.x - robot.x)
  const fovRatio = clamp01(1 - Math.abs(normAngle(bearing - robot.theta)) / (p.sensorFov / 2))
  return rangeRatio * (0.35 + 0.65 * fovRatio)
}

export function initializeEstimatedWallBelief(
  environment: Environment,
  belief?: Pick<BeliefState, 'estimatedWalls' | 'wallBeliefs' | 'mapConfidence'>,
): EstimatedWallSegment[] {
  return environment.walls.map((wall) => {
    const existing = belief?.estimatedWalls?.find((candidate) => candidate.wallId === wall.id)
    if (existing) return existing
    const legacy = belief?.wallBeliefs?.find((candidate) => candidate.wallId === wall.id)
    return {
      wallId: wall.id,
      tMin: 0,
      tMax: 0,
      confidence: legacy?.lastObservedAt !== undefined && legacy.lastObservedAt >= 0 ? legacy.confidence : 0,
      lastObservedAt: legacy?.lastObservedAt ?? -1,
    }
  })
}

export function observeWalls(robot: RobotState, environment: Environment, p: PlannerParameters): WallObservation[] {
  const observations: WallObservation[] = []
  const samplesPerWall = 18
  for (const wall of environment.walls) {
    const observedSamples: { t: number; point: Vec2; strength: number }[] = []
    for (let i = 0; i <= samplesPerWall; i += 1) {
      const t = i / samplesPerWall
      const point = segmentPoint(wall, t)
      const r = distance(robot, point)
      if (r > p.sensorRadius) continue
      const bearing = Math.atan2(point.y - robot.y, point.x - robot.x)
      if (Math.abs(normAngle(bearing - robot.theta)) > p.sensorFov / 2) continue
      if (!hasLineOfSight(robot, point, wall, environment.walls)) continue
      observedSamples.push({ t, point, strength: observationStrength(robot, point, p) })
    }
    if (observedSamples.length === 0) continue

    const nearest = nearestPointOnSegment(robot, wall)
    const tNearest = parameterForPoint(wall, nearest)
    const nearestVisible =
      distance(robot, nearest) <= p.sensorRadius &&
      Math.abs(normAngle(Math.atan2(nearest.y - robot.y, nearest.x - robot.x) - robot.theta)) <= p.sensorFov / 2 &&
      hasLineOfSight(robot, nearest, wall, environment.walls)
    const allT = nearestVisible
      ? [...observedSamples.map((sample) => sample.t), tNearest]
      : observedSamples.map((s) => s.t)
    const strength = observedSamples.reduce((sum, sample) => sum + sample.strength, 0) / observedSamples.length
    const rayTarget = nearestVisible ? nearest : observedSamples[Math.floor(observedSamples.length / 2)].point
    const measurement = noisyRangeBearingMeasurement(robot, rayTarget, p, `${wall.id}:${allT.join(':')}`)
    const rangeNoiseRatio = measurement.rangeStdDev / Math.max(p.sensorRadius, measurement.rangeStdDev)
    const bearingNoiseRatio = measurement.bearingStdDev / Math.max(p.sensorFov, measurement.bearingStdDev)
    const noiseQuality = clamp01(1 - 0.5 * (rangeNoiseRatio + bearingNoiseRatio))
    const adjustedStrength = strength * noiseQuality
    observations.push({
      wallId: wall.id,
      tMin: Math.min(...allT),
      tMax: Math.max(...allT),
      confidence: clamp01(0.35 + adjustedStrength * 0.65),
      strength: adjustedStrength,
      ...measurement,
      rayTarget,
      sensorPose: { ...robot },
    })
  }
  return observations
}

export function updateEstimatedWallBelief(
  estimatedWalls: EstimatedWallSegment[],
  environment: Environment,
  observations: WallObservation[],
  time: number,
): EstimatedWallSegment[] {
  const complete = initializeEstimatedWallBelief(environment, {
    estimatedWalls,
    wallBeliefs: [],
    mapConfidence: 0,
  })
  return complete.map((estimated) => {
    const observation = observations.find((candidate) => candidate.wallId === estimated.wallId)
    if (!observation) return { ...estimated, confidence: estimated.confidence * 0.998 }
    const hadCoverage = estimated.tMax > estimated.tMin
    const wall = environment.walls.find((candidate) => candidate.id === estimated.wallId)
    const affinity = wall ? wallObservationAffinity(observation, observation.sensorPose, wall) : 1
    const affinityGate = 0.35 + 0.65 * affinity
    const effectiveConfidence = observation.confidence * affinityGate
    const effectiveStrength = observation.strength * affinityGate
    return {
      wallId: estimated.wallId,
      tMin: hadCoverage ? Math.min(estimated.tMin, observation.tMin) : observation.tMin,
      tMax: hadCoverage ? Math.max(estimated.tMax, observation.tMax) : observation.tMax,
      confidence: clamp01(
        Math.max(estimated.confidence * 0.996, effectiveConfidence, estimated.confidence + effectiveStrength * 0.14),
      ),
      lastObservedAt: time,
    }
  })
}

export function observedCoverageRatio(estimated: Pick<EstimatedWallSegment, 'tMin' | 'tMax'>) {
  return clamp01(estimated.tMax - estimated.tMin)
}

export function estimatedWallKnowledge(estimated?: EstimatedWallSegment) {
  if (!estimated) return 0
  return clamp01(observedCoverageRatio(estimated) * estimated.confidence)
}

export function computeMapUncertainty(estimatedWalls: EstimatedWallSegment[]) {
  if (estimatedWalls.length === 0) return 1
  return (
    estimatedWalls.reduce((sum, estimated) => sum + (1 - estimatedWallKnowledge(estimated)), 0) / estimatedWalls.length
  )
}

export function estimatedWallConfidence(belief: BeliefState, wallId: string) {
  const estimated = belief.estimatedWalls.find((candidate) => candidate.wallId === wallId)
  if (estimated) return estimatedWallKnowledge(estimated)
  const legacy = belief.wallBeliefs.find((candidate) => candidate.wallId === wallId)
  return legacy?.confidence ?? belief.mapConfidence
}
