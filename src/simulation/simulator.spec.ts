import { describe, expect, it } from 'vitest'
import { createDefaultSimulationState, defaultParameters } from './environment'
import { stepSimulation } from './simulator'

describe('simulator integration', () => {
  it('advances robot, humans, belief, trace, and planning outputs in one optimization loop', () => {
    const state = createDefaultSimulationState()
    const next = stepSimulation(state, { ...defaultParameters, sampleCount: 12, horizonSteps: 5 }, 99)

    expect(next.time).toBeGreaterThan(state.time)
    expect(next.robot.x).not.toBe(state.robot.x)
    expect(next.humans[0].y).not.toBe(state.humans[0].y)
    expect(next.trace.length).toBeGreaterThan(state.trace.length)
    expect(next.plan.candidates.length).toBe(12)
    expect(next.costBreakdown.total).toBe(next.plan.selected.cost.total)
    expect(next.belief.sigmaX + next.belief.sigmaY).toBeGreaterThan(0)
  })
})
