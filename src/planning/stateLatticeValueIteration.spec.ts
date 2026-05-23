import { describe, expect, it } from 'vitest'
import { stepRobotInEnvironment } from '../simulation/dynamics'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import {
  clearStateLatticePolicyCache,
  createStateLatticePolicy,
  getCachedStateLatticePolicy,
  lookupStateLatticeAction,
  lookupStateLatticeValue,
  rolloutStateLatticePolicy,
  stateLatticePolicyCacheStats,
} from './stateLatticeValueIteration'

describe('state-lattice value iteration', () => {
  it('reuses a deterministic policy table for unchanged environment and lattice parameters', () => {
    clearStateLatticePolicyCache()
    const state = createSimulationStateForScenario('spiral-known')
    const options = {
      resolution: 0.4,
      headingBins: 16,
      robotRadius: defaultParameters.robotRadius,
      discount: 0.98,
      iterations: 160,
    }

    const first = getCachedStateLatticePolicy(state.environment, options)
    const second = getCachedStateLatticePolicy(state.environment, options)

    expect(second).toBe(first)
    expect(stateLatticePolicyCacheStats()).toEqual({ size: 1, hits: 1, misses: 1 })
  })

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
    expect(policy.actions.length).toBeGreaterThanOrEqual(9)
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
      parameters: { ...defaultParameters, dt: 0.75 },
      steps: 30,
      stepRobot: stepRobotInEnvironment,
    })
    const last = trajectory.at(-1) ?? lowerCorridor

    expect(last.x).toBeGreaterThan(8)
    expect(last.y).toBeGreaterThan(3)
  })

  it('drives the known spiral from the scenario start through multiple bends toward the center goal', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const policy = createStateLatticePolicy(state.environment, {
      resolution: 0.35,
      headingBins: 24,
      robotRadius: defaultParameters.robotRadius,
      discount: 0.985,
      iterations: 280,
    })

    const trajectory = rolloutStateLatticePolicy({
      initial: state.robot,
      environment: state.environment,
      policy,
      parameters: { ...defaultParameters, dt: 0.75 },
      steps: 300,
      stepRobot: stepRobotInEnvironment,
    })
    const bestDistance = Math.min(
      ...trajectory.map((pose) => Math.hypot(pose.x - state.environment.goal.x, pose.y - state.environment.goal.y)),
    )
    const maxX = Math.max(...trajectory.map((pose) => pose.x))
    const maxY = Math.max(...trajectory.map((pose) => pose.y))

    expect(trajectory.length).toBeGreaterThan(70)
    expect(maxX).toBeGreaterThan(8)
    expect(maxY).toBeGreaterThan(2.4)
    expect(bestDistance).toBeLessThan(1.5)
  })

  it('adds object-belief risk to the value backup instead of only rollout scoring', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const options = {
      resolution: 0.4,
      headingBins: 16,
      robotRadius: defaultParameters.robotRadius,
      discount: 0.98,
      iterations: 180,
    }
    const probe = { x: 4.55, y: 2.95, theta: 0 }
    const baseline = createStateLatticePolicy(state.environment, options)
    const withRisk = createStateLatticePolicy(state.environment, {
      ...options,
      socialRisks: [{ id: 'believed-object', x: probe.x, y: probe.y, radius: 0.45, weight: 4 }],
    })

    expect(lookupStateLatticeValue(withRisk, probe)).toBeGreaterThan(lookupStateLatticeValue(baseline, probe))
  })
})
