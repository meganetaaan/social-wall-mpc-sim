import { beforeEach, describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import { defaultPlannerMode, planWithPolicy } from './policies'
import {
  getStateLatticePolicyService,
  resetStateLatticePolicyService,
  type StateLatticePolicyService,
  setStateLatticePolicyService,
} from './stateLatticePolicyService'
import {
  getCachedStateLatticePolicy,
  lookupStateLatticeAction,
  type StateLatticePolicyOptions,
} from './stateLatticeValueIteration'

describe('policy mode planner selection', () => {
  beforeEach(() => {
    resetStateLatticePolicyService()
    getStateLatticePolicyService().clear()
  })

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

  it('state-lattice mode requests background policy work and uses fallback on a cold cache', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const start = performance.now()

    const result = planWithPolicy({
      mode: 'state-lattice',
      state,
      parameters: defaultParameters,
      seed: 4,
    })
    const elapsedMs = performance.now() - start

    expect(result.bestControl.v).toBeGreaterThan(0)
    expect(result.candidates).toHaveLength(1)
    expect(result.selected.controls).toHaveLength(defaultParameters.horizonSteps)
    expect(elapsedMs).toBeLessThan(1000)
    expect(getStateLatticePolicyService().cacheStats()).toEqual({
      ready: 0,
      pending: 1,
      hits: 0,
      misses: 1,
      backend: 'in-process',
      lastError: null,
    })
  })

  it('state-lattice mode uses explicit service advancement and then hits cache', () => {
    const state = createSimulationStateForScenario('spiral-known')

    planWithPolicy({ mode: 'state-lattice', state, parameters: defaultParameters, seed: 4 })
    for (let i = 0; i < 1000 && getStateLatticePolicyService().cacheStats().ready === 0; i += 1) {
      getStateLatticePolicyService().advancePendingBuilds(4096)
    }

    expect(getStateLatticePolicyService().cacheStats().ready).toBe(1)
    const result = planWithPolicy({ mode: 'state-lattice', state, parameters: defaultParameters, seed: 5000 })

    expect(result.candidates).toHaveLength(1)
    expect(result.selected.controls).toHaveLength(1)
    expect(getStateLatticePolicyService().cacheStats()).toMatchObject({ ready: 1, pending: 0, hits: 1 })
  })

  it('state-lattice mode uses a ready policy lookup with a single-control rollout', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const options = stateLatticeOptionsForState(state, defaultParameters.robotRadius)
    const policy = getCachedStateLatticePolicy(state.environment, options)
    const action = lookupStateLatticeAction(policy, state.robot)

    const result = planWithPolicy({
      mode: 'state-lattice',
      state,
      parameters: defaultParameters,
      seed: 4,
    })

    expect(action).not.toBeNull()
    expect(result.candidates).toHaveLength(1)
    expect(result.selected.controls).toHaveLength(1)
    expect(result.bestControl.v).toBeCloseTo(action?.v ?? Number.NaN)
    expect(result.bestControl.omega).toBeCloseTo(action?.omega ?? Number.NaN)
    expect(getStateLatticePolicyService().cacheStats()).toEqual({
      ready: 1,
      pending: 0,
      hits: 1,
      misses: 1,
      backend: 'in-process',
      lastError: null,
    })
  })

  it('state-lattice mode falls back on an unknown sparse map without starting a misleading build', () => {
    const state = createSimulationStateForScenario('spiral-unknown')

    const result = planWithPolicy({
      mode: 'state-lattice',
      state,
      parameters: defaultParameters,
      seed: 4,
    })

    expect(result.candidates).toHaveLength(1)
    expect(result.selected.controls).toHaveLength(defaultParameters.horizonSteps)
    expect(getStateLatticePolicyService().cacheStats()).toEqual({
      ready: 0,
      pending: 0,
      hits: 0,
      misses: 0,
      backend: 'in-process',
      lastError: null,
    })
  })

  it('state-lattice mode builds and caches from reliable anonymous map features', () => {
    const state = createSimulationStateForScenario('spiral-unknown')
    const featureState = {
      ...state,
      belief: {
        ...state.belief,
        estimatedFeatures: state.environment.walls.map((wall, index) => ({
          id: `feature-${index}`,
          a: wall.a,
          b: wall.b,
          confidence: 0.86,
          observationCount: 4,
          lastObservedAt: 3,
        })),
      },
    }

    planWithPolicy({ mode: 'state-lattice', state: featureState, parameters: defaultParameters, seed: 4 })
    for (let i = 0; i < 1000 && getStateLatticePolicyService().cacheStats().ready === 0; i += 1) {
      getStateLatticePolicyService().advancePendingBuilds(4096)
    }

    expect(getStateLatticePolicyService().cacheStats().ready).toBe(1)
    const result = planWithPolicy({
      mode: 'state-lattice',
      state: featureState,
      parameters: defaultParameters,
      seed: 5000,
    })

    expect(result.candidates).toHaveLength(1)
    expect(result.selected.controls).toHaveLength(1)
    expect(getStateLatticePolicyService().cacheStats()).toMatchObject({ ready: 1, pending: 0, hits: 1 })
  })

  it('state-lattice mode passes moving humans into social-risk options', () => {
    const service = new CapturingPolicyService()
    setStateLatticePolicyService(service)
    const state = {
      ...createSimulationStateForScenario('spiral-known'),
      humans: [{ id: 'walker', x: 1.2, y: 0.8, vx: 0.3, vy: -0.4, radius: 0.22 }],
    }

    planWithPolicy({ mode: 'state-lattice', state, parameters: defaultParameters, seed: 4 })

    expect(service.lastOptions?.socialRisks).toContainEqual({
      id: 'walker',
      x: 1.2,
      y: 0.8,
      radius: 0.22,
      weight: 0.7,
      vx: 0.3,
      vy: -0.4,
    })
  })
})

function stateLatticeOptionsForState(state: ReturnType<typeof createSimulationStateForScenario>, robotRadius: number) {
  return {
    resolution: 0.4,
    headingBins: 16,
    robotRadius,
    discount: 0.98,
    iterations: 180,
    socialRisks: [
      ...(state.belief.objectBeliefs ?? []).map((object) => ({
        id: object.id,
        x: object.centroid.x,
        y: object.centroid.y,
        radius: object.radius,
        weight: 1 + object.pHuman + object.pStatic,
        vx: object.velocity.x,
        vy: object.velocity.y,
      })),
      ...state.humans.map((human) => ({
        id: human.id,
        x: human.x,
        y: human.y,
        radius: human.radius,
        weight: human.vx === 0 && human.vy === 0 ? 1.5 : 0.7,
        vx: human.vx,
        vy: human.vy,
      })),
    ],
  }
}

class CapturingPolicyService implements StateLatticePolicyService {
  lastOptions: StateLatticePolicyOptions | null = null

  requestPolicy(
    _environment: Parameters<StateLatticePolicyService['requestPolicy']>[0],
    options: StateLatticePolicyOptions,
  ) {
    this.lastOptions = options
    return null
  }

  advancePendingBuilds() {
    return []
  }

  cacheStats() {
    return { ready: 0, pending: 0, hits: 0, misses: 0, backend: 'in-process' as const, lastError: null }
  }

  clear() {
    this.lastOptions = null
  }
}
