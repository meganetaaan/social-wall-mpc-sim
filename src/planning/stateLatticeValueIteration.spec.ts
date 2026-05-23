import { describe, expect, it } from 'vitest'
import { stepRobotInEnvironment } from '../simulation/dynamics'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import {
  createStateLatticePolicy,
  lookupStateLatticeAction,
  rolloutStateLatticePolicy,
} from './stateLatticeValueIteration'

describe('state-lattice value iteration', () => {
  it('computes an orientation-aware policy table for a known spiral corridor', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const policy = createStateLatticePolicy(state.environment, {
      resolution: 0.4,
      headingBins: 16,
      robotRadius: defaultParameters.robotRadius,
      discount: 0.98,
      iterations: 160,
    })

    const lowerCorridor = { x: 5.8, y: 0.55, theta: 0 }
    const action = lookupStateLatticeAction(policy, lowerCorridor)

    expect(policy.headingBins).toBe(16)
    expect(action).not.toBeNull()
    expect(action?.v).toBeGreaterThan(0)
  })

  it('drives a known spiral corridor segment by policy lookup instead of sampling rollouts', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const policy = createStateLatticePolicy(state.environment, {
      resolution: 0.4,
      headingBins: 16,
      robotRadius: defaultParameters.robotRadius,
      discount: 0.98,
      iterations: 220,
    })
    const lowerCorridor = { x: 5.8, y: 0.55, theta: 0 }

    const trajectory = rolloutStateLatticePolicy({
      initial: lowerCorridor,
      environment: state.environment,
      policy,
      parameters: { ...defaultParameters, dt: 1 },
      steps: 20,
      stepRobot: stepRobotInEnvironment,
    })
    const last = trajectory.at(-1) ?? lowerCorridor

    expect(last.x).toBeGreaterThan(8)
    expect(last.y).toBeGreaterThan(3)
  })
})
