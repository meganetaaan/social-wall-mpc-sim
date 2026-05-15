import { describe, expect, it } from 'vitest'
import { createDefaultSimulationState } from './environment'

describe('default environment geometry', () => {
  it('exposes a prominent navigation goal and complex multi-segment wall geometry', () => {
    const state = createDefaultSimulationState()

    expect(state.environment.goal).toEqual({ x: 7.1, y: 2.7 })
    expect(state.environment.goalRadius).toBe(0.45)
    expect(state.environment.walls.length).toBeGreaterThanOrEqual(8)
    expect(state.environment.walls.map((wall) => wall.id)).toEqual(
      expect.arrayContaining(['lower-alcove-left', 'lower-alcove-back', 'inner-baffle', 'upper-bend']),
    )
  })
})
