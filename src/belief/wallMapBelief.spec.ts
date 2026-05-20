import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { distance, normAngle } from '../simulation/math'
import type { Environment, EstimatedWallSegment, RobotState } from '../simulation/types'
import { computeMapUncertainty, observedCoverageRatio, observeWalls, updateEstimatedWallBelief } from './wallMapBelief'

const environment: Environment = {
  goal: { x: 4, y: 2 },
  walls: [{ id: 'front-wall', a: { x: 1, y: -1 }, b: { x: 1, y: 1 } }],
  obstacles: [],
}

const params = { ...defaultParameters, sensorRadius: 2.5, sensorFov: Math.PI / 2 }

describe('wall map observation model', () => {
  it('observes a nearby wall segment inside the robot field of view', () => {
    const robot: RobotState = { x: 0, y: 0, theta: 0 }

    const observations = observeWalls(robot, environment, params)

    expect(observations).toHaveLength(1)
    expect(observations[0].wallId).toBe('front-wall')
    expect(observations[0].tMin).toBeLessThan(observations[0].tMax)
    expect(observations[0].confidence).toBeGreaterThan(0.35)
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
})
