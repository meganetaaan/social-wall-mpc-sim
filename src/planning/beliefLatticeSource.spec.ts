import { describe, expect, it } from 'vitest'
import { poseGaussianFromSigmas } from '../belief/poseBelief'
import type { BeliefState, Environment } from '../simulation/types'
import { selectBeliefLatticeSource } from './beliefLatticeSource'

const environment: Environment = {
  goal: { x: 5, y: 2 },
  goalRadius: 0.25,
  walls: [
    { id: 'true-bottom', a: { x: 0, y: 0 }, b: { x: 6, y: 0 } },
    { id: 'true-top', a: { x: 0, y: 2 }, b: { x: 6, y: 2 } },
  ],
  obstacles: [],
}

describe('belief lattice source selection', () => {
  it('rejects short or low-confidence anonymous features for hard lattice walls', () => {
    const source = selectBeliefLatticeSource(environment, {
      ...baseBelief(),
      estimatedFeatures: [
        {
          id: 'short-feature',
          a: { x: 0, y: 0.2 },
          b: { x: 0.4, y: 0.2 },
          confidence: 0.9,
          observationCount: 4,
          lastObservedAt: 1,
        },
        {
          id: 'low-confidence-feature',
          a: { x: 1, y: 0.2 },
          b: { x: 5, y: 0.2 },
          confidence: 0.25,
          observationCount: 8,
          lastObservedAt: 1,
        },
      ],
    })

    expect(source.status).toBe('fallback')
    expect(source.source).toBe('anonymous-features')
    expect(source.reliability.totalLength).toBe(0)
  })

  it('uses sufficiently supported anonymous features without true wall ids', () => {
    const source = selectBeliefLatticeSource(environment, {
      ...baseBelief(),
      estimatedWalls: [
        { wallId: 'true-bottom', tMin: 0, tMax: 1, confidence: 0.95, lastObservedAt: 1 },
        { wallId: 'true-top', tMin: 0, tMax: 1, confidence: 0.95, lastObservedAt: 1 },
      ],
      estimatedFeatures: [
        {
          id: 'feature-bottom',
          a: { x: 0, y: 0.15 },
          b: { x: 5, y: 0.15 },
          confidence: 0.82,
          observationCount: 5,
          lastObservedAt: 1,
        },
      ],
    })

    expect(source.status).toBe('ready')
    if (source.status !== 'ready') throw new Error('expected ready lattice source')
    expect(source.source).toBe('anonymous-features')
    expect(source.environment.walls.map((wall) => wall.id)).toEqual(['feature-bottom'])
    expect(source.environment.walls.map((wall) => wall.id)).not.toContain('true-bottom')
  })

  it('keeps complete high-confidence known-map beliefs on the true environment source', () => {
    const source = selectBeliefLatticeSource(environment, {
      ...baseBelief(),
      estimatedWalls: [
        { wallId: 'true-bottom', tMin: 0, tMax: 1, confidence: 0.9, lastObservedAt: 1 },
        { wallId: 'true-top', tMin: 0, tMax: 1, confidence: 0.9, lastObservedAt: 1 },
      ],
    })

    expect(source.status).toBe('ready')
    if (source.status !== 'ready') throw new Error('expected ready lattice source')
    expect(source.source).toBe('known-map')
    expect(source.environment).toBe(environment)
  })
})

function baseBelief(): BeliefState {
  return {
    pose: poseGaussianFromSigmas({ x: 0, y: 0, theta: 0 }, 0.1, 0.1, 0.05),
    sigmaX: 0.1,
    sigmaY: 0.1,
    sigmaTheta: 0.05,
    mapConfidence: 0.2,
    wallBeliefs: [],
    estimatedWalls: [],
    objectBeliefs: [],
  }
}
