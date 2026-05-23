import type { Environment } from '../simulation/types'
import {
  advanceStateLatticePolicyBuild,
  clearStateLatticePolicyCache,
  requestStateLatticePolicy,
  type StateLatticePolicy,
  type StateLatticePolicyOptions,
  stateLatticePolicyCacheStats,
} from './stateLatticeValueIteration'

export type StateLatticePolicyRequest = {
  id: string
  environment: SerializableStateLatticeEnvironment
  options: SerializableStateLatticePolicyOptions
}

export type SerializableStateLatticeEnvironment = Pick<Environment, 'walls' | 'obstacles' | 'goal' | 'goalRadius'>
export type SerializableStateLatticePolicyOptions = StateLatticePolicyOptions

export type StateLatticePolicyWorkerRequest =
  | { type: 'build-policy'; request: StateLatticePolicyRequest; workBudget?: number }
  | { type: 'request-policy'; request: StateLatticePolicyRequest }
  | { type: 'advance-policy-build'; request: StateLatticePolicyRequest; workBudget?: number }
  | { type: 'cache-stats' }
  | { type: 'clear-cache' }

export type StateLatticePolicyWorkerResponse =
  | { type: 'policy-ready'; id: string; policy: StateLatticePolicy }
  | { type: 'policy-pending'; id: string }
  | { type: 'cache-stats'; stats: StateLatticePolicyServiceStats }
  | { type: 'cache-cleared' }
  | { type: 'policy-error'; id?: string; message: string }

export type StateLatticePolicyServiceStats = {
  ready: number
  pending: number
  hits: number
  misses: number
  backend: 'in-process' | 'worker'
  lastError: string | null
}

export type StateLatticePolicyService = {
  requestPolicy(environment: Environment, options: StateLatticePolicyOptions): StateLatticePolicy | null
  advancePendingBuilds(workBudget?: number): StateLatticePolicy[]
  cacheStats(): StateLatticePolicyServiceStats
  clear(): void
}

export type StateLatticePolicyMessagePort = {
  postMessage(message: StateLatticePolicyWorkerRequest): void
  addEventListener(type: 'message', listener: (event: MessageEvent<StateLatticePolicyWorkerResponse>) => void): void
  removeEventListener(type: 'message', listener: (event: MessageEvent<StateLatticePolicyWorkerResponse>) => void): void
}

let activeStateLatticePolicyService: StateLatticePolicyService = createInProcessStateLatticePolicyService()

export function getStateLatticePolicyService() {
  return activeStateLatticePolicyService
}

export function setStateLatticePolicyService(service: StateLatticePolicyService) {
  activeStateLatticePolicyService = service
}

export function resetStateLatticePolicyService() {
  activeStateLatticePolicyService = createInProcessStateLatticePolicyService()
}

export function createInProcessStateLatticePolicyService(): StateLatticePolicyService {
  const pendingRequests = new Map<string, StateLatticePolicyRequest>()
  return {
    requestPolicy(environment, options) {
      const request = serializeStateLatticePolicyRequest(environment, options)
      if (pendingRequests.has(request.id)) return null
      const policy = requestStateLatticePolicy(environment, options)
      if (!policy) pendingRequests.set(request.id, request)
      return policy
    },
    advancePendingBuilds(workBudget = 4096) {
      const ready: StateLatticePolicy[] = []
      for (const [id, request] of pendingRequests) {
        const policy = advanceStateLatticePolicyBuild(request.environment, request.options, workBudget)
        if (!policy) continue
        ready.push(policy)
        pendingRequests.delete(id)
      }
      return ready
    },
    cacheStats() {
      const stats = stateLatticePolicyCacheStats()
      return {
        ready: stats.size,
        pending: pendingRequests.size,
        hits: stats.hits,
        misses: stats.misses,
        backend: 'in-process',
        lastError: null,
      }
    },
    clear() {
      pendingRequests.clear()
      clearStateLatticePolicyCache()
    },
  }
}

export function createWorkerStateLatticePolicyService(
  worker: StateLatticePolicyMessagePort,
): StateLatticePolicyService {
  const readyPolicies = new Map<string, StateLatticePolicy>()
  const pendingRequests = new Map<string, StateLatticePolicyRequest>()
  const completedPolicies: StateLatticePolicy[] = []
  let hits = 0
  let misses = 0
  let lastError: string | null = null

  const onMessage = (event: MessageEvent<StateLatticePolicyWorkerResponse>) => {
    const message = event.data
    if (message.type === 'policy-ready') {
      readyPolicies.set(message.id, message.policy)
      pendingRequests.delete(message.id)
      completedPolicies.push(message.policy)
      return
    }
    if (message.type === 'policy-error') {
      lastError = message.message
      if (message.id) pendingRequests.delete(message.id)
    }
  }
  worker.addEventListener('message', onMessage)

  return {
    requestPolicy(environment, options) {
      const request = serializeStateLatticePolicyRequest(environment, options)
      const ready = readyPolicies.get(request.id)
      if (ready) {
        hits += 1
        return ready
      }
      if (!pendingRequests.has(request.id)) {
        misses += 1
        pendingRequests.set(request.id, request)
        worker.postMessage({ type: 'request-policy', request })
      }
      return null
    },
    advancePendingBuilds(workBudget = 4096) {
      for (const request of pendingRequests.values()) {
        worker.postMessage({ type: 'advance-policy-build', request, workBudget })
      }
      return completedPolicies.splice(0)
    },
    cacheStats() {
      return {
        ready: readyPolicies.size,
        pending: pendingRequests.size,
        hits,
        misses,
        backend: 'worker',
        lastError,
      }
    },
    clear() {
      readyPolicies.clear()
      pendingRequests.clear()
      completedPolicies.splice(0)
      hits = 0
      misses = 0
      lastError = null
      worker.postMessage({ type: 'clear-cache' })
    },
  }
}

export function serializeStateLatticePolicyRequest(
  environment: Environment,
  options: StateLatticePolicyOptions,
): StateLatticePolicyRequest {
  const serializableEnvironment: SerializableStateLatticeEnvironment = {
    walls: environment.walls.map((wall) => ({ ...wall, a: { ...wall.a }, b: { ...wall.b } })),
    obstacles: environment.obstacles.map((obstacle) => ({ ...obstacle })),
    goal: { ...environment.goal },
    goalRadius: environment.goalRadius,
  }
  const serializableOptions: SerializableStateLatticePolicyOptions = {
    ...options,
    actions: options.actions?.map((action) => ({ ...action })),
    socialRisks: options.socialRisks?.map(normalizeSocialRiskForRequest).sort(compareSerializedSocialRisks),
  }
  const id = JSON.stringify({ environment: serializableEnvironment, options: serializableOptions })
  return { id, environment: serializableEnvironment, options: serializableOptions }
}

function normalizeSocialRiskForRequest(risk: NonNullable<StateLatticePolicyOptions['socialRisks']>[number]) {
  return {
    ...(risk.id === undefined ? {} : { id: risk.id }),
    x: risk.x,
    y: risk.y,
    radius: risk.radius,
    ...(risk.weight === undefined ? {} : { weight: risk.weight }),
    ...(risk.vx === undefined ? {} : { vx: risk.vx }),
    ...(risk.vy === undefined ? {} : { vy: risk.vy }),
    ...(risk.uncertainty === undefined ? {} : { uncertainty: risk.uncertainty }),
    ...(risk.uncertaintyGrowthRate === undefined ? {} : { uncertaintyGrowthRate: risk.uncertaintyGrowthRate }),
  }
}

function compareSerializedSocialRisks(
  a: ReturnType<typeof normalizeSocialRiskForRequest>,
  b: ReturnType<typeof normalizeSocialRiskForRequest>,
) {
  return JSON.stringify(a).localeCompare(JSON.stringify(b))
}

export function handleStateLatticePolicyWorkerRequest(
  message: StateLatticePolicyWorkerRequest,
): StateLatticePolicyWorkerResponse {
  try {
    if (message.type === 'clear-cache') {
      clearStateLatticePolicyCache()
      return { type: 'cache-cleared' }
    }
    if (message.type === 'cache-stats') {
      const stats = stateLatticePolicyCacheStats()
      return {
        type: 'cache-stats',
        stats: {
          ready: stats.size,
          pending: stats.pending,
          hits: stats.hits,
          misses: stats.misses,
          backend: 'worker',
          lastError: null,
        },
      }
    }

    const { request } = message
    const environment = request.environment
    const options = request.options
    if (message.type === 'request-policy') {
      const policy = requestStateLatticePolicy(environment, options)
      return policy ? { type: 'policy-ready', id: request.id, policy } : { type: 'policy-pending', id: request.id }
    }

    const policy =
      message.type === 'build-policy'
        ? advanceStateLatticePolicyBuild(environment, options, message.workBudget ?? 4096)
        : advanceStateLatticePolicyBuild(environment, options, message.workBudget)
    return policy ? { type: 'policy-ready', id: request.id, policy } : { type: 'policy-pending', id: request.id }
  } catch (error) {
    return {
      type: 'policy-error',
      id: 'request' in message ? message.request.id : undefined,
      message: error instanceof Error ? error.message : String(error),
    }
  }
}
