import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { distance, normAngle } from '../simulation/math'
import type { Environment, EstimatedWallSegment, RobotState } from '../simulation/types'
import {
  computeMapUncertainty,
  expectedRangeBearingToWall,
  observedCoverageRatio,
  observeWalls,
  updateEstimatedWallBelief,
  wallObservationAffinity,
  wallObservationLikelihood,
} from './wallMapBelief'

const environment: Environment = {
  goal: { x: 4, y: 2 },
  walls: [{ id: 'front-wall', a: { x: 1, y: -1 }, b: { x: 1, y: 1 } }],
  obstacles: [],
}

const params = { ...defaultParameters, sensorRadius: 2.5, sensorFov: Math.PI / 2 }

describe('wall map observation model', () => {
  it('computes expected range and relative bearing to the nearest point on a wall', () => {
    const robot: RobotState = { x: 0, y: 0.25, theta: Math.PI / 2 }
    const wall = environment.walls[0]

    const expected = expectedRangeBearingToWall(robot, wall)

    expect(expected.point).toEqual({ x: 1, y: 0.25 })
    expect(expected.range).toBeCloseTo(1)
    expect(expected.bearing).toBeCloseTo(normAngle(-Math.PI / 2))
  })

  it('scores a matching wall observation higher than a distant wrong wall', () => {
    const robot: RobotState = { x: 0, y: 0, theta: 0 }
    const wall = environment.walls[0]
    const expected = expectedRangeBearingToWall(robot, wall)
    const wrongWall = { id: 'wrong-wall', a: { x: 2, y: -1 }, b: { x: 2, y: 1 } }

    const matching = wallObservationLikelihood({ ...expected, rangeStdDev: 0.05, bearingStdDev: 0.02 }, robot, wall)
    const wrong = wallObservationLikelihood({ ...expected, rangeStdDev: 0.05, bearingStdDev: 0.02 }, robot, wrongWall)

    expect(matching.logLikelihood).toBeGreaterThan(wrong.logLikelihood)
    expect(matching.likelihood).toBeGreaterThan(wrong.likelihood)
    expect(Number.isFinite(matching.likelihood)).toBe(true)
  })

  it('converts observation likelihood residuals to a bounded wall affinity', () => {
    const robot: RobotState = { x: 0, y: 0, theta: 0 }
    const wall = environment.walls[0]
    const expected = expectedRangeBearingToWall(robot, wall)

    const matching = wallObservationAffinity({ ...expected, rangeStdDev: 0.05, bearingStdDev: 0.02 }, robot, wall)
    const inconsistent = wallObservationAffinity(
      { range: expected.range + 1, bearing: expected.bearing + 0.5, rangeStdDev: 0.05, bearingStdDev: 0.02 },
      robot,
      wall,
    )

    expect(matching).toBeGreaterThan(0.99)
    expect(matching).toBeLessThanOrEqual(1)
    expect(inconsistent).toBeGreaterThanOrEqual(0)
    expect(inconsistent).toBeLessThan(0.01)
  })

  it('observes a nearby wall segment inside the robot field of view', () => {
    const robot: RobotState = { x: 0, y: 0, theta: 0 }

    const observations = observeWalls(robot, environment, params)

    expect(observations).toHaveLength(1)
    expect(observations[0].wallId).toBe('front-wall')
    expect(observations[0].tMin).toBeLessThan(observations[0].tMax)
    expect(observations[0].confidence).toBeGreaterThan(0.35)
  })

  it('captures the sensor pose used to produce each wall observation', () => {
    const robot: RobotState = { x: 0, y: 0.1, theta: 0.05 }

    const [observation] = observeWalls(robot, environment, params)

    expect(observation.sensorPose).toEqual(robot)
  })

  it('includes deterministic noisy range and bearing measurements for the representative wall point', () => {
    const robot: RobotState = { x: 0, y: 0, theta: 0 }

    const [observation] = observeWalls(robot, environment, params)

    const trueRange = distance(robot, observation.rayTarget)
    const trueBearing = normAngle(
      Math.atan2(observation.rayTarget.y - robot.y, observation.rayTarget.x - robot.x) - robot.theta,
    )
    expect(observation.rangeStdDev).toBeGreaterThan(0)
    expect(observation.bearingStdDev).toBeGreaterThan(0)
    expect(Math.abs(observation.range - trueRange)).toBeLessThanOrEqual(2 * observation.rangeStdDev + 1e-6)
    expect(Math.abs(normAngle(observation.bearing - trueBearing))).toBeLessThanOrEqual(
      2 * observation.bearingStdDev + 1e-6,
    )
  })

  it('returns stable noisy measurements for the same robot, map, and parameters', () => {
    const robot: RobotState = { x: 0, y: 0, theta: 0 }

    const first = observeWalls(robot, environment, params)
    const second = observeWalls(robot, environment, params)

    expect(second[0].range).toBe(first[0].range)
    expect(second[0].bearing).toBe(first[0].bearing)
  })

  it('normalizes noisy bearing measurements at the angle wrap boundary', () => {
    const wrapEnvironment: Environment = {
      goal: { x: 0, y: 0 },
      walls: [{ id: 'wrap-wall', a: { x: -1.2, y: -0.1 }, b: { x: -1.2, y: 0.1 } }],
      obstacles: [],
    }
    const robot: RobotState = { x: 0, y: 0, theta: -Math.PI + 0.02 }

    const [observation] = observeWalls(robot, wrapEnvironment, { ...params, sensorFov: Math.PI / 2 })

    expect(observation.bearing).toBeGreaterThanOrEqual(-Math.PI)
    expect(observation.bearing).toBeLessThanOrEqual(Math.PI)
  })

  it('does not observe a wall behind the robot or outside sensor range', () => {
    const behindRobot: RobotState = { x: 0, y: 0, theta: Math.PI }
    const farRobot: RobotState = { x: -4, y: 0, theta: 0 }

    expect(observeWalls(behindRobot, environment, params)).toHaveLength(0)
    expect(observeWalls(farRobot, environment, params)).toHaveLength(0)
  })

  it('does not observe a far wall hidden behind a nearer wall', () => {
    const overlappingWalls: Environment = {
      goal: { x: 4, y: 0 },
      walls: [
        { id: 'near-wall', a: { x: 1, y: -1 }, b: { x: 1, y: 1 } },
        { id: 'far-wall', a: { x: 2, y: -1 }, b: { x: 2, y: 1 } },
      ],
      obstacles: [],
    }
    const robot: RobotState = { x: 0, y: 0, theta: 0 }

    const observations = observeWalls(robot, overlappingWalls, { ...params, sensorRadius: 3 })

    expect(observations.map((observation) => observation.wallId)).toContain('near-wall')
    expect(observations.map((observation) => observation.wallId)).not.toContain('far-wall')
  })

  it('observes far wall samples when line of sight does not cross the nearer blocker', () => {
    const offsetWalls: Environment = {
      goal: { x: 4, y: 0 },
      walls: [
        { id: 'near-wall', a: { x: 1, y: -1 }, b: { x: 1, y: 0 } },
        { id: 'far-wall', a: { x: 2, y: 0.55 }, b: { x: 2, y: 1.2 } },
      ],
      obstacles: [],
    }
    const robot: RobotState = { x: 0, y: 0, theta: 0 }

    const observations = observeWalls(robot, offsetWalls, { ...params, sensorRadius: 3 })

    expect(observations.map((observation) => observation.wallId)).toContain('near-wall')
    expect(observations.map((observation) => observation.wallId)).toContain('far-wall')
  })

  it('does not treat a connected wall endpoint touch as an occluding blocker', () => {
    const connectedWalls: Environment = {
      goal: { x: 4, y: 0 },
      walls: [
        { id: 'corner-wall', a: { x: 1, y: -1 }, b: { x: 1, y: 0 } },
        { id: 'continuing-wall', a: { x: 1, y: 0 }, b: { x: 2, y: 0 } },
      ],
      obstacles: [],
    }
    const robot: RobotState = { x: 0, y: 0, theta: 0 }

    const observations = observeWalls(robot, connectedWalls, { ...params, sensorRadius: 3 })

    expect(observations.map((observation) => observation.wallId)).toContain('continuing-wall')
  })

  it('repeated observations expand coverage and raise confidence', () => {
    const initial: EstimatedWallSegment[] = [
      { wallId: 'front-wall', tMin: 0.45, tMax: 0.55, confidence: 0.2, lastObservedAt: 0 },
    ]

    const first = updateEstimatedWallBelief(
      initial,
      environment,
      observeWalls({ x: 0, y: -0.45, theta: 0 }, environment, params),
      1,
    )
    const second = updateEstimatedWallBelief(
      first,
      environment,
      observeWalls({ x: 0, y: 0.45, theta: 0 }, environment, params),
      2,
    )

    expect(observedCoverageRatio(second[0])).toBeGreaterThan(observedCoverageRatio(initial[0]))
    expect(second[0].confidence).toBeGreaterThan(initial[0].confidence)
    expect(computeMapUncertainty(second)).toBeLessThan(computeMapUncertainty(initial))
  })

  it('uses observation affinity to reduce confidence growth for inconsistent measurements while merging coverage', () => {
    const initial: EstimatedWallSegment[] = [
      { wallId: 'front-wall', tMin: 0.45, tMax: 0.55, confidence: 0.2, lastObservedAt: 0 },
    ]
    const robot: RobotState = { x: 0, y: 0, theta: 0 }
    const [observed] = observeWalls(robot, environment, params)
    const expected = expectedRangeBearingToWall(robot, environment.walls[0])
    const consistentObservation = {
      ...observed,
      range: expected.range,
      bearing: expected.bearing,
      confidence: 0.95,
      strength: 1,
    }
    const inconsistentObservation = {
      ...consistentObservation,
      range: consistentObservation.range + 1.5,
      bearing: consistentObservation.bearing + 0.75,
      confidence: 0.95,
      strength: 1,
    }

    const consistent = updateEstimatedWallBelief(initial, environment, [consistentObservation], 1)
    const inconsistent = updateEstimatedWallBelief(initial, environment, [inconsistentObservation], 1)

    expect(observedCoverageRatio(inconsistent[0])).toBeGreaterThan(observedCoverageRatio(initial[0]))
    expect(inconsistent[0].confidence).toBeLessThan(consistent[0].confidence)
    expect(inconsistent[0].confidence - initial[0].confidence).toBeLessThan(0.3)
  })
})
