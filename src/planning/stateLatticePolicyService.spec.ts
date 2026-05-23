import { beforeEach, describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import {
  createInProcessStateLatticePolicyService,
  handleStateLatticePolicyWorkerRequest,
  serializeStateLatticePolicyRequest,
} from './stateLatticePolicyService'

describe('state-lattice policy service', () => {
  beforeEach(() => {
    createInProcessStateLatticePolicyService().clear()
  })

  it('queues cold policy requests without completing the ready cache immediately', () => {
    const service = createInProcessStateLatticePolicyService()
    const state = createSimulationStateForScenario('spiral-known')
    const options = testOptions()

    expect(service.requestPolicy(state.environment, options)).toBeNull()
    expect(service.cacheStats()).toEqual({
      ready: 0,
      pending: 1,
      hits: 0,
      misses: 1,
      backend: 'in-process',
    })
  })

  it('advances pending work deterministically until a requested policy is ready', () => {
    const service = createInProcessStateLatticePolicyService()
    const state = createSimulationStateForScenario('spiral-known')
    const options = testOptions()

    expect(service.requestPolicy(state.environment, options)).toBeNull()
    let completed = 0
    for (let i = 0; i < 2000 && service.cacheStats().ready === 0; i += 1) {
      completed += service.advancePendingBuilds(4096).length
    }

    expect(completed).toBe(1)
    expect(service.cacheStats()).toMatchObject({ ready: 1, pending: 0, hits: 0, misses: 1 })
    expect(service.requestPolicy(state.environment, options)).not.toBeNull()
    expect(service.cacheStats()).toMatchObject({ ready: 1, pending: 0, hits: 1, misses: 1 })
  })

  it('serializes environment and options into a worker-safe request', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const options = testOptions()
    const request = serializeStateLatticePolicyRequest(state.environment, options)

    expect(request.id).toBe(JSON.stringify({ environment: request.environment, options: request.options }))
    expect(request.environment).toEqual({
      walls: state.environment.walls,
      obstacles: state.environment.obstacles,
      goal: state.environment.goal,
      goalRadius: state.environment.goalRadius,
    })
    expect(request.options).toEqual(options)
  })

  it('handles worker protocol request, advancement, stats, and clear messages', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const request = serializeStateLatticePolicyRequest(state.environment, testOptions())

    expect(handleStateLatticePolicyWorkerRequest({ type: 'clear-cache' })).toEqual({ type: 'cache-cleared' })
    expect(handleStateLatticePolicyWorkerRequest({ type: 'request-policy', request })).toEqual({
      type: 'policy-pending',
      id: request.id,
    })

    let response = handleStateLatticePolicyWorkerRequest({
      type: 'advance-policy-build',
      request,
      workBudget: 8,
    })
    expect(response).toEqual({ type: 'policy-pending', id: request.id })

    for (let i = 0; i < 2000 && response.type !== 'policy-ready'; i += 1) {
      response = handleStateLatticePolicyWorkerRequest({
        type: 'advance-policy-build',
        request,
        workBudget: 4096,
      })
    }

    expect(response.type).toBe('policy-ready')
    expect(handleStateLatticePolicyWorkerRequest({ type: 'cache-stats' })).toMatchObject({
      type: 'cache-stats',
      stats: { ready: 1, pending: 0, backend: 'worker' },
    })
  })
})

function testOptions() {
  return {
    resolution: 0.4,
    headingBins: 16,
    robotRadius: defaultParameters.robotRadius,
    discount: 0.98,
    iterations: 12,
  }
}
