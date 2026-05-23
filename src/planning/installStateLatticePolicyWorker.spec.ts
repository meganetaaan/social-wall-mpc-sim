import { beforeEach, describe, expect, it, vi } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import { installStateLatticePolicyWorker } from './installStateLatticePolicyWorker'
import {
  getStateLatticePolicyService,
  resetStateLatticePolicyService,
  type StateLatticePolicyMessagePort,
  type StateLatticePolicyWorkerResponse,
} from './stateLatticePolicyService'

describe('installStateLatticePolicyWorker', () => {
  beforeEach(() => {
    resetStateLatticePolicyService()
    getStateLatticePolicyService().clear()
  })

  it('installs a worker-backed policy service from a provided Worker factory', () => {
    const worker = new FakeStateLatticePolicyWorker()
    const workerFactory = vi.fn(() => worker)

    const service = installStateLatticePolicyWorker({ workerFactory })

    expect(workerFactory).toHaveBeenCalledOnce()
    expect(service).toBe(getStateLatticePolicyService())
    expect(getStateLatticePolicyService().cacheStats().backend).toBe('worker')

    expect(
      service.requestPolicy(createSimulationStateForScenario('spiral-known').environment, testOptions()),
    ).toBeNull()
    expect(worker.postedMessages.map((message) => message.type)).toEqual(['build-policy'])
  })

  it('falls back to the in-process policy service when no Worker factory is available', () => {
    const service = installStateLatticePolicyWorker({ workerFactory: null })

    expect(service).toBe(getStateLatticePolicyService())
    expect(service.cacheStats()).toMatchObject({ backend: 'in-process', ready: 0, pending: 0 })
  })

  it('falls back to the in-process policy service when Worker construction throws', () => {
    const service = installStateLatticePolicyWorker({
      workerFactory: () => {
        throw new Error('Worker construction failed')
      },
    })

    expect(service).toBe(getStateLatticePolicyService())
    expect(service.cacheStats()).toMatchObject({ backend: 'in-process', ready: 0, pending: 0 })
  })
})

class FakeStateLatticePolicyWorker implements StateLatticePolicyMessagePort {
  readonly postedMessages: Parameters<StateLatticePolicyMessagePort['postMessage']>[0][] = []
  private readonly listeners = new Set<(event: MessageEvent<StateLatticePolicyWorkerResponse>) => void>()

  postMessage(message: Parameters<StateLatticePolicyMessagePort['postMessage']>[0]) {
    this.postedMessages.push(message)
  }

  addEventListener(_type: 'message', listener: (event: MessageEvent<StateLatticePolicyWorkerResponse>) => void) {
    this.listeners.add(listener)
  }

  removeEventListener(_type: 'message', listener: (event: MessageEvent<StateLatticePolicyWorkerResponse>) => void) {
    this.listeners.delete(listener)
  }
}

function testOptions() {
  return {
    resolution: 0.4,
    headingBins: 16,
    robotRadius: defaultParameters.robotRadius,
    discount: 0.98,
    iterations: 12,
  }
}
