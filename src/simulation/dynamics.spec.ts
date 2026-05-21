import { describe, expect, it } from 'vitest'
import { predictHumans, stepRobotInEnvironment } from './dynamics'
import type { Environment, HumanState } from './types'

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

describe('robot wall collision dynamics', () => {
  it('does not let the robot center cross a wall segment in one simulation step', () => {
    const environment: Environment = {
      walls: [{ id: 'vertical-wall', a: { x: 1, y: -1 }, b: { x: 1, y: 1 } }],
      obstacles: [],
      goal: { x: 2, y: 0 },
    }

    const next = stepRobotInEnvironment({ x: 0.8, y: 0, theta: 0 }, { v: 1.0, omega: 0 }, 0.4, environment, {
      robotRadius: 0.1,
    })

    expect(next.x).toBeLessThanOrEqual(0.9)
  })
})
