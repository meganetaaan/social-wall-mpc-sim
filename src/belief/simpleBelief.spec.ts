import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import type { BeliefState, Environment, RobotState } from '../simulation/types'
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
