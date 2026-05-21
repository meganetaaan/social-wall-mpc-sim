import { distance, normAngle } from '../simulation/math'
import type {
  Environment,
  HumanState,
  ObjectBelief,
  PlannerParameters,
  PointObservation,
  RobotState,
  Vec2,
} from '../simulation/types'

const clamp01 = (value: number) => Math.min(1, Math.max(0, value))
const TRACK_ASSOCIATION_DISTANCE = 0.75

function pointObservation(id: string, robot: RobotState, point: Vec2, time: number): PointObservation {
  return {
    id,
    point: { ...point },
    range: distance(robot, point),
    bearing: normAngle(Math.atan2(point.y - robot.y, point.x - robot.x) - robot.theta),
    sensorPose: { ...robot },
    time,
    source: 'point-sensor',
  }
}

function isVisible(robot: RobotState, point: Vec2, parameters: PlannerParameters) {
  if (distance(robot, point) > parameters.sensorRadius) return false
  const bearing = normAngle(Math.atan2(point.y - robot.y, point.x - robot.x) - robot.theta)
  return Math.abs(bearing) <= parameters.sensorFov / 2
}

function wallSamplePoint(a: Vec2, b: Vec2, t: number): Vec2 {
  return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t }
}

export function observePointReturns(args: {
  robot: RobotState
  environment: Environment
  humans: HumanState[]
  parameters: PlannerParameters
  time: number
}): PointObservation[] {
  const observations: PointObservation[] = []
  const { robot, environment, humans, parameters, time } = args

  for (const wall of environment.walls) {
    for (const t of [0.25, 0.5, 0.75]) {
      const point = wallSamplePoint(wall.a, wall.b, t)
      if (isVisible(robot, point, parameters)) {
        observations.push(pointObservation(`point-${observations.length}`, robot, point, time))
      }
    }
  }

  for (const obstacle of environment.obstacles) {
    const point = { x: obstacle.x, y: obstacle.y }
    if (isVisible(robot, point, parameters)) {
      observations.push(pointObservation(`point-${observations.length}`, robot, point, time))
    }
  }

  for (const human of humans) {
    const point = { x: human.x, y: human.y }
    if (isVisible(robot, point, parameters)) {
      observations.push(pointObservation(`point-${observations.length}`, robot, point, time))
    }
  }

  return observations
}

function nextTrackId(existing: ObjectBelief[], created: ObjectBelief[]) {
  return `track-${existing.length + created.length + 1}`
}

function humanProbability(speed: number, radius: number, pDynamic: number) {
  if (pDynamic < 0.25) return 0
  const speedLikelihood = clamp01((speed - 0.08) / 0.25)
  const sizeLikelihood = clamp01(1 - Math.abs(radius - 0.23) / 0.28)
  return clamp01(0.02 + pDynamic * (0.62 + speedLikelihood * 0.2 + sizeLikelihood * 0.1))
}

function probabilities(speed: number, radius: number, observedCount: number) {
  const mature = observedCount > 1
  const pDynamic = mature ? clamp01((speed - 0.04) / 0.3) : 0.15
  const pStatic = clamp01(1 - pDynamic)
  return { pStatic, pDynamic, pHuman: humanProbability(speed, radius, pDynamic) }
}

export function updateObjectBeliefs(
  existing: ObjectBelief[],
  observations: PointObservation[],
  time: number,
): ObjectBelief[] {
  const updated = existing.map((track) => ({ ...track }))
  const created: ObjectBelief[] = []
  const associatedThisUpdate = new Set<string>()

  for (const observation of observations) {
    let bestIndex = -1
    let bestDistance = Number.POSITIVE_INFINITY
    for (let i = 0; i < updated.length; i += 1) {
      if (associatedThisUpdate.has(updated[i].id)) continue
      const candidateDistance = distance(updated[i].centroid, observation.point)
      if (candidateDistance < bestDistance) {
        bestDistance = candidateDistance
        bestIndex = i
      }
    }

    const target =
      bestIndex >= 0 && bestDistance <= TRACK_ASSOCIATION_DISTANCE
        ? updated[bestIndex]
        : {
            id: nextTrackId(existing, created),
            centroid: { ...observation.point },
            velocity: { x: 0, y: 0 },
            radius: 0.23,
            observedCount: 0,
            lastObservedAt: observation.time,
            pStatic: 0.85,
            pDynamic: 0.15,
            pHuman: 0.12,
          }

    const dt = Math.max(0.001, observation.time - target.lastObservedAt)
    const velocity = {
      x: (observation.point.x - target.centroid.x) / dt,
      y: (observation.point.y - target.centroid.y) / dt,
    }
    const observedCount = target.observedCount + 1
    const radius = Math.max(0.12, Math.min(0.45, target.radius * 0.85 + 0.23 * 0.15))
    const speed = Math.hypot(velocity.x, velocity.y)
    const inferred = probabilities(speed, radius, observedCount)
    const next = {
      ...target,
      centroid: { ...observation.point },
      velocity,
      radius,
      observedCount,
      lastObservedAt: time,
      ...inferred,
    }

    if (bestIndex >= 0 && bestDistance <= TRACK_ASSOCIATION_DISTANCE) {
      updated[bestIndex] = next
      associatedThisUpdate.add(next.id)
    } else {
      created.push(next)
    }
  }

  return [...updated, ...created]
}
