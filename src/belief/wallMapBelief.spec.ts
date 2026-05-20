import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
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
