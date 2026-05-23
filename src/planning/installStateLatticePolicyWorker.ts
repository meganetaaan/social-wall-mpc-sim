import {
  createInProcessStateLatticePolicyService,
  createWorkerStateLatticePolicyService,
  type StateLatticePolicyMessagePort,
  type StateLatticePolicyService,
  setStateLatticePolicyService,
} from './stateLatticePolicyService'

export type StateLatticePolicyWorkerFactory = () => StateLatticePolicyMessagePort

export type InstallStateLatticePolicyWorkerOptions = {
  workerFactory?: StateLatticePolicyWorkerFactory | null
}

export function installStateLatticePolicyWorker(
  options: InstallStateLatticePolicyWorkerOptions = {},
): StateLatticePolicyService {
  const workerFactory =
    options.workerFactory === undefined ? defaultStateLatticePolicyWorkerFactory : options.workerFactory

  if (!workerFactory) return installInProcessStateLatticePolicyService()

  try {
    const service = createWorkerStateLatticePolicyService(workerFactory())
    setStateLatticePolicyService(service)
    return service
  } catch {
    return installInProcessStateLatticePolicyService()
  }
}

function installInProcessStateLatticePolicyService() {
  const service = createInProcessStateLatticePolicyService()
  setStateLatticePolicyService(service)
  return service
}

function defaultStateLatticePolicyWorkerFactory(): StateLatticePolicyMessagePort {
  if (typeof Worker !== 'function') throw new Error('State-lattice policy Worker is unavailable')
  return new Worker(new URL('./stateLatticePolicy.worker.ts', import.meta.url), { type: 'module' })
}
