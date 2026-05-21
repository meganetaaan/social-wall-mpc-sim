import { describe, expect, it } from 'vitest'
import { associateWallObservation } from './wallAssociation'

describe('anonymous wall association diagnostics', () => {
  it('selects the wall with the highest observation likelihood', () => {
    const observation = {
      point: { x: 2, y: 0.1 },
      range: 2,
      bearing: 0,
      rangeStdDev: 0.1,
      bearingStdDev: 0.05,
      sensorPose: { x: 0, y: 0, theta: 0 },
      time: 0,
    }
    const walls = [
      { id: 'near', a: { x: 0, y: 0 }, b: { x: 4, y: 0 } },
      { id: 'far', a: { x: 0, y: 2 }, b: { x: 4, y: 2 } },
    ]

    const association = associateWallObservation(observation, walls)

    expect(association?.wallId).toBe('near')
    expect(association?.score).toBeGreaterThan(0)
    expect(association?.distanceResidual).toBeLessThan(0.2)
    expect(Math.abs(association?.bearingResidual ?? 1)).toBeLessThan(0.1)
    expect(association?.candidates).toHaveLength(2)
  })

  it('can prefer another wall when the anonymous measurement fits it better', () => {
    const observation = {
      point: { x: 2, y: 1.9 },
      range: Math.hypot(2, 1.9),
      bearing: Math.atan2(1.9, 2),
      rangeStdDev: 0.08,
      bearingStdDev: 0.03,
      sensorPose: { x: 0, y: 0, theta: 0 },
      time: 0,
    }
    const walls = [
      { id: 'true-lower', a: { x: 0, y: 0 }, b: { x: 4, y: 0 } },
      { id: 'preferred-upper', a: { x: 0, y: 2 }, b: { x: 4, y: 2 } },
    ]

    expect(associateWallObservation(observation, walls)?.wallId).toBe('preferred-upper')
  })

  it('returns null when no candidate walls are available', () => {
    const observation = {
      point: { x: 2, y: 0 },
      range: 2,
      bearing: 0,
      sensorPose: { x: 0, y: 0, theta: 0 },
      time: 0,
    }

    expect(associateWallObservation(observation, [])).toBeNull()
  })
})
