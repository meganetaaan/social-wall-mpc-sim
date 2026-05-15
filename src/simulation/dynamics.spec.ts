import { describe, expect, it } from 'vitest'
import { predictHumans } from './dynamics'
import type { HumanState } from './types'

describe('human motion patterns', () => {
  it('lets a following human stay behind the robot while moving along the corridor', () => {
    const [human] = predictHumans(
      [
        {
          id: 'follower',
          x: 1.2,
          y: 0.9,
          vx: 0,
          vy: 0,
          radius: 0.23,
          motion: { kind: 'follow-robot', distanceBehind: 0.9, lateralOffset: 0.05, speed: 0.45 },
        },
      ],
      1,
      undefined,
      { x: 2.4, y: 0.85, theta: 0 },
      0,
    )

    expect(human.x).toBeGreaterThan(1.2)
    expect(human.x).toBeLessThan(2.4)
    expect(Math.abs(human.y - 0.9)).toBeLessThan(0.1)
  })

  it('moves a timed blocker aside after the yielding time', () => {
    const blocker: HumanState = {
      id: 'yielding-blocker',
      x: 2.4,
      y: 0.86,
      vx: 0,
      vy: 0,
      radius: 0.28,
      motion: { kind: 'yield-after', yieldAt: 3, target: { x: 2.4, y: 1.75 }, speed: 0.55 },
    }

    const [before] = predictHumans([blocker], 1, undefined, undefined, 1)
    const [after] = predictHumans([blocker], 1, undefined, undefined, 3.2)

    expect(before.y).toBeCloseTo(blocker.y)
    expect(after.y).toBeGreaterThan(blocker.y)
  })
})
