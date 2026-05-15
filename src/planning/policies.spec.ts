import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import { defaultPlannerMode, planWithPolicy } from './policies'

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
})
