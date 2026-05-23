import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import { defaultPlannerMode, planWithPolicy } from './policies'
import { clearStateLatticePolicyCache, stateLatticePolicyCacheStats } from './stateLatticeValueIteration'

describe('policy mode planner selection', () => {
  it('preserves belief MPC as the default policy mode', () => {
    expect(defaultPlannerMode).toBe('belief-mpc')
  })

  it('reactive-stop returns zero velocity when a human is too close', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const closeHumanState = {
      ...state,
      humans: [{ id: 'close-human', x: state.robot.x + 0.3, y: state.robot.y, vx: 0, vy: 0, radius: 0.22 }],
    }

    const result = planWithPolicy({
      mode: 'reactive-stop',
      state: closeHumanState,
      parameters: defaultParameters,
      seed: 4,
    })

    expect(result.bestControl.v).toBe(0)
  })

  it('wall-only keeps moving in the same close-human setup', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const closeHumanState = {
      ...state,
      humans: [{ id: 'close-human', x: state.robot.x + 0.3, y: state.robot.y, vx: 0, vy: 0, radius: 0.22 }],
    }

    const result = planWithPolicy({
      mode: 'wall-only',
      state: closeHumanState,
      parameters: defaultParameters,
      seed: 4,
    })

    expect(result.bestControl.v).toBeGreaterThan(0)
  })

  it('state-lattice mode uses an orientation-aware policy lookup instead of sampling MPC', () => {
    clearStateLatticePolicyCache()
    const state = createSimulationStateForScenario('spiral-known')

    const result = planWithPolicy({
      mode: 'state-lattice',
      state,
      parameters: defaultParameters,
      seed: 4,
    })

    expect(result.bestControl.v).toBeGreaterThan(0)
    expect(result.candidates).toHaveLength(1)
    expect(result.selected.controls).toHaveLength(1)
    expect(stateLatticePolicyCacheStats()).toEqual({ size: 1, hits: 0, misses: 1 })
  })

  it('state-lattice mode reuses the precomputed value policy across unchanged planning calls', () => {
    clearStateLatticePolicyCache()
    const state = createSimulationStateForScenario('spiral-known')

    planWithPolicy({ mode: 'state-lattice', state, parameters: defaultParameters, seed: 4 })
    planWithPolicy({ mode: 'state-lattice', state, parameters: defaultParameters, seed: 5 })

    expect(stateLatticePolicyCacheStats()).toEqual({ size: 1, hits: 1, misses: 1 })
  })
})
