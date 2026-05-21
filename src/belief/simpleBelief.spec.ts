import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import type { BeliefState, Environment, RobotState, WallObservation } from '../simulation/types'
import { updateEstimatedLineFeatures } from './lineFeatureMap'
import { poseGaussianFromSigmas } from './poseBelief'
import { mapUncertaintyTrace, traceSigma, updateBelief } from './simpleBelief'

const environment: Environment = {
  goal: { x: 5, y: 2 },
  walls: [
    { id: 'observed-wall', a: { x: 0, y: 0 }, b: { x: 5, y: 0 } },
    { id: 'far-wall', a: { x: 7, y: 0 }, b: { x: 7, y: 4 } },
  ],
  obstacles: [],
}

const initialBelief: BeliefState = {
  pose: poseGaussianFromSigmas({ x: 0.8, y: 0.45, theta: -Math.PI / 2 }, 0.14, 0.14, 0.04),
  sigmaX: 0.14,
  sigmaY: 0.14,
  sigmaTheta: 0.04,
  mapConfidence: 0.45,
  wallBeliefs: [
    { wallId: 'observed-wall', confidence: 0.2, lastObservedAt: -1 },
    { wallId: 'far-wall', confidence: 0.7, lastObservedAt: -1 },
  ],
  estimatedWalls: [
    { wallId: 'observed-wall', tMin: 0, tMax: 0, confidence: 0, lastObservedAt: -1 },
    { wallId: 'far-wall', tMin: 0, tMax: 0.6, confidence: 0.7, lastObservedAt: -1 },
  ],
}

describe('map belief update', () => {
  it('raises confidence for nearby observed walls without claiming all walls are known', () => {
    const robot: RobotState = { x: 1, y: 0.45, theta: -Math.PI / 2 }

    const updated = updateBelief(initialBelief, robot, environment, defaultParameters, 12)

    const observed = updated.wallBeliefs.find((wall) => wall.wallId === 'observed-wall')
    const far = updated.wallBeliefs.find((wall) => wall.wallId === 'far-wall')
    expect(observed?.confidence).toBeGreaterThan(0.2)
    expect(observed?.lastObservedAt).toBe(12)
    expect(far?.confidence).toBeLessThanOrEqual(0.7)
    expect(updated.estimatedWalls.find((wall) => wall.wallId === 'observed-wall')?.tMax).toBeGreaterThan(0)
    expect(updated.mapConfidence).toBeGreaterThan(0)
  })

  it('keeps scalar pose sigmas synchronized with the covariance diagonal', () => {
    const robot: RobotState = { x: 1, y: 0.45, theta: -Math.PI / 2 }

    const updated = updateBelief(initialBelief, robot, environment, defaultParameters, 12)

    expect(updated.sigmaX).toBe(updated.pose.covariance[0][0])
    expect(updated.sigmaY).toBe(updated.pose.covariance[1][1])
    expect(updated.sigmaTheta).toBe(updated.pose.covariance[2][2])
  })

  it('keeps true-id wall coverage as the default map update mode', () => {
    const robot: RobotState = { x: 1, y: 0.45, theta: -Math.PI / 2 }

    const updated = updateBelief(initialBelief, robot, environment, defaultParameters, 12)

    expect(updated.estimatedWalls.find((wall) => wall.wallId === 'observed-wall')?.tMax).toBeGreaterThan(0)
    expect(updated.estimatedFeatures ?? []).toHaveLength(0)
  })

  it('updates anonymous estimated line features in opt-in map update mode', () => {
    const robot: RobotState = { x: 1, y: 0.45, theta: -Math.PI / 2 }

    const updated = updateBelief(initialBelief, robot, environment, defaultParameters, 12, [], {
      mapUpdateMode: 'anonymous-line-features',
    })

    expect(updated.estimatedWalls).toEqual(initialBelief.estimatedWalls)
    expect(updated.estimatedFeatures?.[0]?.id).toMatch(/^feature-/)
    expect(Object.keys(updated.estimatedFeatures?.[0] ?? {})).not.toContain('wallId')
  })

  it('reuses anonymous estimated line features across matching updates', () => {
    const first: WallObservation = {
      wallId: 'true-wall-a',
      tMin: 0.2,
      tMax: 0.4,
      confidence: 0.7,
      strength: 0.6,
      range: 1,
      bearing: 0,
      rangeStdDev: 0.03,
      bearingStdDev: 0.01,
      rayTarget: { x: 1, y: 0 },
      sensorPose: { x: 1, y: 1, theta: -Math.PI / 2 },
    }
    const second = { ...first, wallId: 'different-true-wall', rayTarget: { x: 1.02, y: 0.01 } }

    const created = updateEstimatedLineFeatures({ existing: [], observations: [first], time: 1 })
    const reused = updateEstimatedLineFeatures({ existing: created, observations: [second], time: 2 })

    expect(reused).toHaveLength(1)
    expect(reused[0].id).toBe(created[0].id)
    expect(reused[0].observationCount).toBe(2)
  })

  it('traces pose covariance before legacy scalar fields', () => {
    const belief: BeliefState = {
      ...initialBelief,
      pose: poseGaussianFromSigmas(initialBelief.pose.mean, 0.3, 0.2, 0.1),
      sigmaX: 9,
      sigmaY: 9,
      sigmaTheta: 9,
    }

    expect(traceSigma(belief)).toBeCloseTo(0.6)
  })

  it('makes map uncertainty trace larger when wall confidence is lower', () => {
    const confident: BeliefState = {
      ...initialBelief,
      wallBeliefs: initialBelief.wallBeliefs.map((wall) => ({ ...wall, confidence: 0.95 })),
      estimatedWalls: initialBelief.estimatedWalls.map((wall) => ({ ...wall, tMin: 0, tMax: 1, confidence: 0.95 })),
    }
    const uncertain: BeliefState = {
      ...initialBelief,
      wallBeliefs: initialBelief.wallBeliefs.map((wall) => ({ ...wall, confidence: 0.2 })),
      estimatedWalls: initialBelief.estimatedWalls.map((wall) => ({ ...wall, tMin: 0, tMax: 0.2, confidence: 0.2 })),
    }

    expect(mapUncertaintyTrace(uncertain)).toBeGreaterThan(mapUncertaintyTrace(confident))
  })
})
