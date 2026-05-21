import { describe, expect, it } from 'vitest'
import type { PointObservation } from '../simulation/types'
import { plannerFacingObservationKeys, toPlannerPointObservation } from './observations'

describe('planner-facing observations', () => {
  it('does not expose true feature or semantic labels', () => {
    const keys = plannerFacingObservationKeys()

    expect(keys).not.toContain('wallId')
    expect(keys).not.toContain('humanId')
    expect(keys).not.toContain('class')
    expect(keys).not.toContain('kind')
  })

  it('strips source metadata from point returns while keeping generic sensor provenance', () => {
    const source = {
      id: 'point-1',
      point: { x: 1, y: 2 },
      range: 2.24,
      bearing: 0.46,
      sensorPose: { x: 0, y: 0, theta: 0 },
      time: 1.5,
      source: 'point-sensor',
      wallId: 'truth-wall',
      humanId: 'truth-human',
      class: 'human',
      kind: 'wall',
    } satisfies PointObservation & Record<string, unknown>

    const observation = toPlannerPointObservation(source)

    expect(observation).toEqual({
      id: 'point-1',
      observationType: 'point-return',
      point: { x: 1, y: 2 },
      range: 2.24,
      bearing: 0.46,
      sensorPose: { x: 0, y: 0, theta: 0 },
      time: 1.5,
      source: 'point-sensor',
    })
    expect(Object.keys(observation)).not.toContain('wallId')
    expect(Object.keys(observation)).not.toContain('humanId')
    expect(Object.keys(observation)).not.toContain('class')
    expect(Object.keys(observation)).not.toContain('kind')
  })
})
