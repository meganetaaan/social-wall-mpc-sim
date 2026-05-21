import { describe, expect, it } from 'vitest'
import type { EstimatedLineFeature, WallObservation } from '../simulation/types'
import { createEstimatedLineFeature, updateEstimatedLineFeature, updateEstimatedLineFeatures } from './lineFeatureMap'

const observation = (overrides: Partial<WallObservation> = {}): WallObservation => ({
  wallId: overrides.wallId ?? 'true-wall-a',
  tMin: overrides.tMin ?? 0.2,
  tMax: overrides.tMax ?? 0.4,
  confidence: overrides.confidence ?? 0.7,
  strength: overrides.strength ?? 0.6,
  range: overrides.range ?? 1,
  bearing: overrides.bearing ?? 0,
  rangeStdDev: overrides.rangeStdDev ?? 0.03,
  bearingStdDev: overrides.bearingStdDev ?? 0.01,
  rayTarget: overrides.rayTarget ?? { x: 1, y: 0 },
  sensorPose: overrides.sensorPose ?? { x: 1, y: 1, theta: -Math.PI / 2 },
})

describe('estimated line feature map', () => {
  it('creates a local feature from an observed segment without storing the true wall id', () => {
    const feature = createEstimatedLineFeature(observation(), 12)

    expect(feature.id).toMatch(/^feature-/)
    expect(Object.keys(feature)).not.toContain('wallId')
    expect(feature.confidence).toBeCloseTo(0.7)
    expect(feature.lastObservedAt).toBe(12)
    expect(feature.observationCount).toBe(1)
  })

  it('updates a reused feature by refreshing endpoints, confidence, and observation count', () => {
    const feature = createEstimatedLineFeature(observation(), 12)
    const updated = updateEstimatedLineFeature(feature, observation({ rayTarget: { x: 1.4, y: 0 } }), 14)

    expect(updated.id).toBe(feature.id)
    expect(updated.observationCount).toBe(2)
    expect(updated.lastObservedAt).toBe(14)
    expect(updated.confidence).toBeGreaterThan(feature.confidence)
    expect(updated.a.x).toBeGreaterThan(feature.a.x)
    expect(updated.b.x).toBeGreaterThan(feature.b.x)
  })

  it('creates and reuses anonymous features without using true wall ids for identity', () => {
    const first = observation({ wallId: 'true-wall-a', rayTarget: { x: 1, y: 0 } })
    const second = observation({ wallId: 'true-wall-b', rayTarget: { x: 1.05, y: 0.02 } })

    const created = updateEstimatedLineFeatures({ existing: [], observations: [first], time: 5 })
    const reused = updateEstimatedLineFeatures({ existing: created, observations: [second], time: 6 })

    expect(reused).toHaveLength(1)
    expect(reused[0].id).toBe(created[0].id)
    expect(reused[0].observationCount).toBe(2)
    expect(Object.keys(reused[0] satisfies EstimatedLineFeature)).not.toContain('wallId')
  })

  it('creates a new anonymous feature when observations do not match existing local features', () => {
    const created = updateEstimatedLineFeatures({
      existing: [],
      observations: [observation({ rayTarget: { x: 1, y: 0 } })],
      time: 5,
    })
    const updated = updateEstimatedLineFeatures({
      existing: created,
      observations: [observation({ rayTarget: { x: 4, y: 0 } })],
      time: 6,
    })

    expect(updated.map((feature) => feature.id)).toEqual(['feature-1', 'feature-2'])
  })

  it('does not turn repeated lower-corridor wall observations into cross-corridor blockers', () => {
    const observations = [
      observation({ rayTarget: { x: 4.8, y: 0.15 }, sensorPose: { x: 4.7, y: 0.56, theta: 0 }, bearing: -1.33 }),
      observation({ rayTarget: { x: 5.3, y: 0.15 }, sensorPose: { x: 5.2, y: 0.56, theta: 0 }, bearing: -1.33 }),
      observation({ rayTarget: { x: 5.8, y: 0.15 }, sensorPose: { x: 5.7, y: 0.56, theta: 0 }, bearing: -1.33 }),
    ]

    const features = updateEstimatedLineFeatures({ existing: [], observations, time: 8 })

    expect(features.length).toBeGreaterThan(0)
    for (const feature of features) {
      const dx = Math.abs(feature.b.x - feature.a.x)
      const dy = Math.abs(feature.b.y - feature.a.y)
      expect(dx).toBeGreaterThan(dy)
    }
  })
})
