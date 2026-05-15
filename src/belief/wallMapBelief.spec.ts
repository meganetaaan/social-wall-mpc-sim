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
