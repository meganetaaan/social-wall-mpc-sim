import {
  handleStateLatticePolicyWorkerRequest,
  type StateLatticePolicyWorkerRequest,
  type StateLatticePolicyWorkerResponse,
} from './stateLatticePolicyService'

const workerScope = globalThis as unknown as {
  addEventListener(type: 'message', listener: (event: MessageEvent<StateLatticePolicyWorkerRequest>) => void): void
  postMessage(message: StateLatticePolicyWorkerResponse): void
}

workerScope.addEventListener('message', (event) => {
  workerScope.postMessage(handleStateLatticePolicyWorkerRequest(event.data))
})
