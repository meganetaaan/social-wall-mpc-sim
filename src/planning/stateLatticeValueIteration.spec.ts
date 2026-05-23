import { describe, expect, it } from 'vitest'
import { stepRobotInEnvironment } from '../simulation/dynamics'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import type { PoseCovariance } from '../simulation/types'
import {
  advanceStateLatticePolicyBuild,
  clearStateLatticePolicyCache,
  createStateLatticePolicy,
  getCachedStateLatticePolicy,
  lookupStateLatticeAction,
  lookupStateLatticeActionForBelief,
  lookupStateLatticeValue,
  requestStateLatticePolicy,
  rolloutStateLatticePolicy,
  type StateLatticePolicy,
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
    expect(stateLatticePolicyCacheStats()).toEqual({ size: 1, pending: 0, hits: 1, misses: 1 })
  })

  it('stages policy construction without completing the cache on the first request', () => {
    clearStateLatticePolicyCache()
    const state = createSimulationStateForScenario('spiral-known')
    const options = {
      resolution: 0.4,
      headingBins: 16,
      robotRadius: defaultParameters.robotRadius,
      discount: 0.98,
      iterations: 12,
    }

    expect(requestStateLatticePolicy(state.environment, options)).toBeNull()
    expect(stateLatticePolicyCacheStats()).toEqual({ size: 0, pending: 1, hits: 0, misses: 1 })

    expect(advanceStateLatticePolicyBuild(state.environment, options, 8)).toBeNull()
    expect(stateLatticePolicyCacheStats()).toEqual({ size: 0, pending: 1, hits: 0, misses: 1 })

    let ready: ReturnType<typeof advanceStateLatticePolicyBuild> = null
    for (let i = 0; i < 2000 && !ready; i += 1) {
      ready = advanceStateLatticePolicyBuild(state.environment, options, 4096)
    }

    expect(ready).not.toBeNull()
    expect(stateLatticePolicyCacheStats()).toEqual({ size: 1, pending: 0, hits: 0, misses: 1 })
    expect(requestStateLatticePolicy(state.environment, options)).toBe(ready)
    expect(stateLatticePolicyCacheStats()).toEqual({ size: 1, pending: 0, hits: 1, misses: 1 })
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

  it('changes policy when a moving risk crosses a primitive path later in the transition', () => {
    const environment = { walls: [], obstacles: [], goal: { x: 2, y: 0 }, goalRadius: 0.35 }
    const options = {
      resolution: 0.5,
      headingBins: 8,
      robotRadius: 0.2,
      actionDuration: 1,
      discount: 0.96,
      iterations: 32,
      padding: 2,
      actions: [
        { id: 'forward', v: 1, omega: 0 },
        { id: 'turn-left', v: 0.5, omega: Math.PI / 2 },
        { id: 'turn-right', v: 0.5, omega: -Math.PI / 2 },
      ],
    }
    const probe = { x: 0, y: 0, theta: 0 }
    const nonCrossing = createStateLatticePolicy(environment, {
      ...options,
      socialRisks: [{ id: 'crossing-human', x: 1, y: -1, radius: 0.25, weight: 25, vx: 0, vy: 0 }],
    })
    const crossing = createStateLatticePolicy(environment, {
      ...options,
      socialRisks: [{ id: 'crossing-human', x: 1, y: -1, radius: 0.25, weight: 25, vx: 0, vy: 1 }],
    })

    expect(lookupStateLatticeAction(nonCrossing, probe)?.id).toBe('forward')
    expect(lookupStateLatticeAction(crossing, probe)?.id).not.toBe('forward')
  })

  it('falls back to mean-pose lookup for tiny pose covariance', () => {
    const policy = testConsensusPolicy()
    const robot = { x: 0, y: 0, theta: 0 }
    const belief = {
      pose: {
        mean: robot,
        covariance: [
          [1e-12, 0, 0],
          [0, 1e-12, 0],
          [0, 0, 1e-12],
        ] satisfies PoseCovariance,
      },
    }

    expect(lookupStateLatticeAction(policy, robot)?.id).toBe('mean-action')
    expect(lookupStateLatticeActionForBelief(policy, robot, belief)?.id).toBe('mean-action')
  })

  it('uses sigma-point consensus when broad pose belief crosses nearby policy cells', () => {
    const policy = testConsensusPolicy()
    const robot = { x: 0, y: 0, theta: 0 }
    const belief = {
      pose: {
        mean: robot,
        covariance: [
          [1, 0, 0],
          [0, 1, 0],
          [0, 0, 1e-12],
        ] satisfies PoseCovariance,
      },
    }

    expect(lookupStateLatticeAction(policy, robot)?.id).toBe('mean-action')
    expect(lookupStateLatticeActionForBelief(policy, robot, belief)?.id).toBe('consensus-action')
  })
})

function testConsensusPolicy(): StateLatticePolicy {
  const values = new Float64Array([2, 2, 2, 2, 10, 2, 2, 2, 2])
  const policy = new Int16Array([1, 1, 1, 1, 0, 1, 1, 1, 1])
  return {
    origin: { x: -1, y: -1 },
    width: 3,
    height: 3,
    resolution: 1,
    headingBins: 1,
    actions: [
      { id: 'mean-action', v: 0.1, omega: 0 },
      { id: 'consensus-action', v: 0.2, omega: 0.1 },
    ],
    values,
    policy,
    unreachableCost: 1000,
    actionDuration: 1,
  }
}
