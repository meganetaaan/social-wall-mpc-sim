import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import type { HumanState, PointObservation, RobotState } from '../simulation/types'
import { observePointReturns, updateObjectBeliefs } from './pointObjectBelief'

const robot: RobotState = { x: 0, y: 0, theta: 0 }

describe('point-derived object belief', () => {
  it('emits unlabeled point observations for visible humans', () => {
    const human: HumanState = { id: 'human-truth-id', x: 1.2, y: 0, vx: 0, vy: 0, radius: 0.23 }

    const observations = observePointReturns({
      robot,
      environment: { walls: [], obstacles: [], goal: { x: 4, y: 0 } },
      humans: [human],
      parameters: defaultParameters,
      time: 1.5,
    })

    expect(observations).toHaveLength(1)
    expect(observations[0].point.x).toBeCloseTo(human.x)
    expect(observations[0].range).toBeCloseTo(1.2)
    expect(Object.keys(observations[0])).not.toContain('humanId')
    expect(Object.keys(observations[0])).not.toContain('class')
    expect(Object.keys(observations[0])).not.toContain('kind')
  })

  it('infers high static probability for repeated stationary point clusters', () => {
    const stationary: PointObservation = {
      id: 'p0',
      point: { x: 1, y: 0 },
      range: 1,
      bearing: 0,
      sensorPose: robot,
      time: 0,
      source: 'point-sensor',
    }

    const once = updateObjectBeliefs([], [stationary], 0)
    const twice = updateObjectBeliefs(once, [{ ...stationary, id: 'p1', time: 1 }], 1)

    expect(twice).toHaveLength(1)
    expect(twice[0].pStatic).toBeGreaterThan(0.7)
    expect(twice[0].pDynamic).toBeLessThan(0.3)
  })

  it('infers dynamic and human probability for plausibly moving human-sized clusters', () => {
    const first: PointObservation = {
      id: 'p0',
      point: { x: 1, y: 0 },
      range: 1,
      bearing: 0,
      sensorPose: robot,
      time: 0,
      source: 'point-sensor',
    }
    const second: PointObservation = {
      ...first,
      id: 'p1',
      point: { x: 1.28, y: 0 },
      range: 1.28,
      time: 1,
    }

    const once = updateObjectBeliefs([], [first], 0)
    const twice = updateObjectBeliefs(once, [second], 1)

    expect(twice).toHaveLength(1)
    expect(twice[0].pDynamic).toBeGreaterThan(0.55)
    expect(twice[0].pHuman).toBeGreaterThan(0.45)
  })
})
