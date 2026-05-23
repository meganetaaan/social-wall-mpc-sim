import { beforeEach, describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import {
  createInProcessStateLatticePolicyService,
  createWorkerStateLatticePolicyService,
  handleStateLatticePolicyWorkerRequest,
  type StateLatticePolicyMessagePort,
  type StateLatticePolicyWorkerResponse,
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
      lastError: null,
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

  it('serializes social-risk velocity through deterministic request ids and worker protocol', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const options = {
      ...testOptions(),
      socialRisks: [{ id: 'moving-human', x: 1.2, y: 2.3, radius: 0.24, weight: 3, vx: 0.4, vy: -0.2 }],
    }
    const request = serializeStateLatticePolicyRequest(state.environment, options)
    const reordered = serializeStateLatticePolicyRequest(state.environment, {
      ...options,
      socialRisks: [{ vy: -0.2, vx: 0.4, weight: 3, radius: 0.24, y: 2.3, x: 1.2, id: 'moving-human' }],
    })

    expect(request.id).toBe(reordered.id)
    expect(request.options.socialRisks).toEqual(options.socialRisks)
    expect(handleStateLatticePolicyWorkerRequest({ type: 'clear-cache' })).toEqual({ type: 'cache-cleared' })
    expect(handleStateLatticePolicyWorkerRequest({ type: 'request-policy', request })).toEqual({
      type: 'policy-pending',
      id: request.id,
    })
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

  it('worker service queues cold requests and advances pending builds in bounded chunks', () => {
    const worker = new FakeStateLatticePolicyWorker()
    const service = createWorkerStateLatticePolicyService(worker)
    const state = createSimulationStateForScenario('spiral-known')
    const options = testOptions()

    expect(service.requestPolicy(state.environment, options)).toBeNull()
    expect(worker.postedMessages.map((message) => message.type)).toEqual(['request-policy'])
    expect(service.cacheStats()).toEqual({
      ready: 0,
      pending: 1,
      hits: 0,
      misses: 1,
      backend: 'worker',
      lastError: null,
    })

    expect(service.advancePendingBuilds(17)).toEqual([])
    expect(worker.postedMessages.at(-1)).toMatchObject({ type: 'advance-policy-build', workBudget: 17 })

    worker.emit({ type: 'policy-ready', id: worker.lastRequestId(), policy: handleReadyPolicy(state, options) })

    expect(service.advancePendingBuilds(17)).toHaveLength(1)
    expect(service.cacheStats()).toMatchObject({ ready: 1, pending: 0, hits: 0, misses: 1, backend: 'worker' })
    expect(service.requestPolicy(state.environment, options)).not.toBeNull()
    expect(service.cacheStats()).toMatchObject({ ready: 1, pending: 0, hits: 1, misses: 1 })
  })

  it('worker service exposes worker errors in stats and clears failed pending requests', () => {
    const worker = new FakeStateLatticePolicyWorker()
    const service = createWorkerStateLatticePolicyService(worker)
    const state = createSimulationStateForScenario('spiral-known')

    service.requestPolicy(state.environment, testOptions())
    worker.emit({ type: 'policy-error', id: worker.lastRequestId(), message: 'worker failed' })

    expect(service.cacheStats()).toEqual({
      ready: 0,
      pending: 0,
      hits: 0,
      misses: 1,
      backend: 'worker',
      lastError: 'worker failed',
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

function handleReadyPolicy(
  state: ReturnType<typeof createSimulationStateForScenario>,
  options: ReturnType<typeof testOptions>,
) {
  let response = handleStateLatticePolicyWorkerRequest({
    type: 'advance-policy-build',
    request: serializeStateLatticePolicyRequest(state.environment, options),
    workBudget: 4096,
  })
  for (let i = 0; i < 2000 && response.type !== 'policy-ready'; i += 1) {
    response = handleStateLatticePolicyWorkerRequest({
      type: 'advance-policy-build',
      request: serializeStateLatticePolicyRequest(state.environment, options),
      workBudget: 4096,
    })
  }
  if (response.type !== 'policy-ready') throw new Error('Expected test policy to complete')
  return response.policy
}

class FakeStateLatticePolicyWorker implements StateLatticePolicyMessagePort {
  readonly postedMessages: Parameters<StateLatticePolicyMessagePort['postMessage']>[0][] = []
  private readonly listeners = new Set<(event: MessageEvent<StateLatticePolicyWorkerResponse>) => void>()

  postMessage(message: Parameters<StateLatticePolicyMessagePort['postMessage']>[0]) {
    this.postedMessages.push(message)
    if (message.type === 'request-policy') this.emit({ type: 'policy-pending', id: message.request.id })
  }

  addEventListener(_type: 'message', listener: (event: MessageEvent<StateLatticePolicyWorkerResponse>) => void) {
    this.listeners.add(listener)
  }

  removeEventListener(_type: 'message', listener: (event: MessageEvent<StateLatticePolicyWorkerResponse>) => void) {
    this.listeners.delete(listener)
  }

  emit(message: StateLatticePolicyWorkerResponse) {
    for (const listener of this.listeners) listener({ data: message } as MessageEvent<StateLatticePolicyWorkerResponse>)
  }

  lastRequestId() {
    const message = this.postedMessages.findLast((candidate) => 'request' in candidate)
    if (!message || !('request' in message)) throw new Error('Expected posted request')
    return message.request.id
  }
}
