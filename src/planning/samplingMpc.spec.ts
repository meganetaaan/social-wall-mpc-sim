import { describe, expect, it } from 'vitest'
import { createDefaultSimulationState, defaultParameters } from '../simulation/environment'
import { planSamplingMpc } from './samplingMpc'

describe('sampling MPC planner', () => {
  it('selects a deterministic best trajectory with candidate rollouts and cost breakdown', () => {
    const state = createDefaultSimulationState()
    const resultA = planSamplingMpc({
      state,
      parameters: { ...defaultParameters, sampleCount: 24, horizonSteps: 8 },
      seed: 1234,
    })
    const resultB = planSamplingMpc({
      state,
      parameters: { ...defaultParameters, sampleCount: 24, horizonSteps: 8 },
      seed: 1234,
    })

    expect(resultA.candidates).toHaveLength(24)
    expect(resultA.selected.trajectory.length).toBe(9)
    expect(resultA.selected.controls.length).toBe(8)
    expect(resultA.bestControl.v).toBeGreaterThanOrEqual(0)
    expect(resultA.bestControl.v).toBeLessThanOrEqual(defaultParameters.vMax)
    expect(resultA.bestControl).toEqual(resultB.bestControl)
    expect(resultA.selected.cost.total).toBeLessThanOrEqual(resultA.candidates[0].cost.total)
  })

  it('normalizes direct programmatic horizon and sample counts to at least one rollout', () => {
    const state = createDefaultSimulationState()
    const result = planSamplingMpc({
      state,
      parameters: { ...defaultParameters, sampleCount: 0, horizonSteps: 0 },
      seed: 5,
    })

    expect(result.candidates).toHaveLength(1)
    expect(result.selected.trajectory.length).toBe(2)
    expect(result.selected.controls.length).toBe(1)
  })

  it('can prefer stopping when a human blocks every safe forward rollout', () => {
    const state = createDefaultSimulationState()
    const blocked = {
      ...state,
      humans: [{ id: 'blocking-human', x: state.robot.x + 0.25, y: state.robot.y, vx: 0, vy: 0, radius: 0.25 }],
    }

    const result = planSamplingMpc({
      state: blocked,
      parameters: { ...defaultParameters, sampleCount: 36, horizonSteps: 6, dMin: 0.9, wHuman: 16 },
      seed: 8,
    })

    expect(result.bestControl.v).toBeLessThan(0.08)
  })
})
