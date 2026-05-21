import { describe, expect, it } from 'vitest'
import { poseGaussianFromSigmas } from '../belief/poseBelief'
import type { BeliefState, Environment } from '../simulation/types'
import { expectedValueFieldCost } from './beliefValueExpectation'
import { createGridValueField } from './valueField'

const baseEnvironment: Environment = {
  walls: [
    { id: 'bottom', a: { x: 0, y: 0 }, b: { x: 6, y: 0 } },
    { id: 'top', a: { x: 0, y: 3 }, b: { x: 6, y: 3 } },
    { id: 'divider', a: { x: 3, y: 0 }, b: { x: 3, y: 2.25 } },
  ],
  obstacles: [],
  goal: { x: 2.5, y: 1.5 },
}

const environmentWithField: Environment = {
  ...baseEnvironment,
  valueField: createGridValueField(baseEnvironment, { resolution: 0.25, robotRadius: 0.08 }),
}

function beliefWithSigmas(sigmaX: number, sigmaY: number): BeliefState {
  const pose = poseGaussianFromSigmas({ x: 3.5, y: 1.5, theta: 0 }, sigmaX, sigmaY, 0.05)
  return {
    pose,
    sigmaX: pose.covariance[0][0],
    sigmaY: pose.covariance[1][1],
    sigmaTheta: pose.covariance[2][2],
    mapConfidence: 0.8,
    wallBeliefs: [],
    estimatedWalls: [],
    objectBeliefs: [],
  }
}

describe('expected value-field cost over pose belief', () => {
  it('penalizes broad pose covariance that overlaps wall-separated or unreachable cells', () => {
    const tight = expectedValueFieldCost(environmentWithField, beliefWithSigmas(0.02, 0.02))
    const broad = expectedValueFieldCost(environmentWithField, beliefWithSigmas(0.8, 0.2))

    expect(tight).not.toBeNull()
    expect(broad).not.toBeNull()
    expect(broad ?? 0).toBeGreaterThan((tight ?? 0) * 2)
  })

  it('returns null when no value field or pose belief is available', () => {
    const belief = beliefWithSigmas(0.02, 0.02)

    expect(expectedValueFieldCost(baseEnvironment, belief)).toBeNull()
    expect(expectedValueFieldCost(environmentWithField, { ...belief, pose: undefined as never })).toBeNull()
  })
})
