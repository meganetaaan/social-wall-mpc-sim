import { stepRobotInEnvironment } from '../simulation/dynamics'
import { distance, nearestWall, normAngle } from '../simulation/math'
import type { ControlInput, Environment, PlannerParameters, RobotState, Vec2 } from '../simulation/types'

export type StateLatticeAction = ControlInput & { id: string }
export type StateLatticeSocialRisk = Vec2 & { id?: string; radius: number; weight?: number }

export type StateLatticePolicyOptions = {
  resolution: number
  headingBins: number
  robotRadius: number
  actions?: StateLatticeAction[]
  actionDuration?: number
  discount?: number
  iterations?: number
  padding?: number
  unreachableCost?: number
  socialRisks?: StateLatticeSocialRisk[]
}

export type StateLatticePolicy = {
  origin: Vec2
  width: number
  height: number
  resolution: number
  headingBins: number
  actions: StateLatticeAction[]
  values: Float64Array
  policy: Int16Array
  unreachableCost: number
  actionDuration: number
  cacheKey?: string
}

const defaultActions: StateLatticeAction[] = [
  { id: 'forward-fast', v: 0.5, omega: 0 },
  { id: 'forward-slow', v: 0.3, omega: 0 },
  { id: 'forward-left-small', v: 0.42, omega: 0.45 },
  { id: 'forward-right-small', v: 0.42, omega: -0.45 },
  { id: 'forward-left-large', v: 0.32, omega: 1.05 },
  { id: 'forward-right-large', v: 0.32, omega: -1.05 },
  { id: 'reverse', v: -0.18, omega: 0 },
  { id: 'reverse-left', v: -0.16, omega: 0.75 },
  { id: 'reverse-right', v: -0.16, omega: -0.75 },
]

let cacheHits = 0
let cacheMisses = 0
const policyCache = new Map<string, StateLatticePolicy>()
const pendingBuilds = new Map<string, StateLatticePolicyBuild>()

type StateLatticePolicyBuild = {
  cacheKey: string
  lattice: StateLatticePolicy
  transitions: ReturnType<typeof buildTransitionTable>
  discount: number
  iterations: number
  completedIterations: number
  stateIndex: number
  nextValues: Float64Array
}

export function getCachedStateLatticePolicy(
  environment: Environment,
  options: StateLatticePolicyOptions,
): StateLatticePolicy {
  const cacheKey = stateLatticePolicyCacheKey(environment, options)
  const cached = policyCache.get(cacheKey)
  if (cached) {
    cacheHits += 1
    return cached
  }
  cacheMisses += 1
  const policy = createStateLatticePolicy(environment, options)
  policy.cacheKey = cacheKey
  policyCache.set(cacheKey, policy)
  return policy
}

export function requestStateLatticePolicy(
  environment: Environment,
  options: StateLatticePolicyOptions,
): StateLatticePolicy | null {
  const cacheKey = stateLatticePolicyCacheKey(environment, options)
  const cached = policyCache.get(cacheKey)
  if (cached) {
    cacheHits += 1
    return cached
  }
  cacheMisses += 1
  if (!pendingBuilds.has(cacheKey))
    pendingBuilds.set(cacheKey, createStateLatticePolicyBuild(environment, options, cacheKey))
  return null
}

export function advanceStateLatticePolicyBuild(
  environment: Environment,
  options: StateLatticePolicyOptions,
  workBudget = 2048,
): StateLatticePolicy | null {
  const cacheKey = stateLatticePolicyCacheKey(environment, options)
  const cached = policyCache.get(cacheKey)
  if (cached) return cached
  let build = pendingBuilds.get(cacheKey)
  if (!build) {
    build = createStateLatticePolicyBuild(environment, options, cacheKey)
    pendingBuilds.set(cacheKey, build)
  }
  const completed = advanceBuild(build, Math.max(1, Math.floor(workBudget)))
  if (!completed) return null
  completed.cacheKey = cacheKey
  pendingBuilds.delete(cacheKey)
  policyCache.set(cacheKey, completed)
  return completed
}

export function clearStateLatticePolicyCache() {
  policyCache.clear()
  pendingBuilds.clear()
  cacheHits = 0
  cacheMisses = 0
}

export function stateLatticePolicyCacheStats() {
  return { size: policyCache.size, pending: pendingBuilds.size, hits: cacheHits, misses: cacheMisses }
}

export function createStateLatticePolicy(
  environment: Environment,
  options: StateLatticePolicyOptions,
): StateLatticePolicy {
  const build = createStateLatticePolicyBuild(environment, options)
  while (!advanceBuild(build, build.lattice.values.length)) {
    // Advance whole sweeps for direct/offline construction.
  }
  return build.lattice
}

function createStateLatticePolicyBuild(
  environment: Environment,
  options: StateLatticePolicyOptions,
  cacheKey?: string,
): StateLatticePolicyBuild {
  const resolution = options.resolution
  const headingBins = Math.max(4, Math.floor(options.headingBins))
  const padding = options.padding ?? resolution * 2
  const unreachableCost = options.unreachableCost ?? 1_000_000
  const actionDuration = options.actionDuration ?? 1
  const actions = options.actions ?? defaultActions
  const points = [environment.goal, ...environment.walls.flatMap((wall) => [wall.a, wall.b])]
  const minX = Math.min(...points.map((point) => point.x)) - padding
  const minY = Math.min(...points.map((point) => point.y)) - padding
  const maxX = Math.max(...points.map((point) => point.x)) + padding
  const maxY = Math.max(...points.map((point) => point.y)) + padding
  const width = Math.max(1, Math.ceil((maxX - minX) / resolution) + 1)
  const height = Math.max(1, Math.ceil((maxY - minY) / resolution) + 1)
  const stateCount = width * height * headingBins
  const values = new Float64Array(stateCount)
  values.fill(unreachableCost)
  const policy = new Int16Array(stateCount)
  policy.fill(-1)
  const lattice: StateLatticePolicy = {
    origin: { x: minX, y: minY },
    width,
    height,
    resolution,
    headingBins,
    actions,
    values,
    policy,
    unreachableCost,
    actionDuration,
  }
  const blocked = new Uint8Array(width * height)
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      blocked[toCellIndex(lattice, x, y)] = isBlocked(cellCenter(lattice, x, y), environment, options.robotRadius)
        ? 1
        : 0
    }
  }
  const goalRadius = environment.goalRadius ?? resolution
  for (let y = 0; y < height; y += 1) {
    for (let x = 0; x < width; x += 1) {
      if (blocked[toCellIndex(lattice, x, y)]) continue
      if (distance(cellCenter(lattice, x, y), environment.goal) > goalRadius) continue
      for (let h = 0; h < headingBins; h += 1) values[toStateIndex(lattice, x, y, h)] = 0
    }
  }

  const transitions = buildTransitionTable(
    lattice,
    environment,
    blocked,
    options.robotRadius,
    options.socialRisks ?? [],
  )
  const discount = options.discount ?? 0.98
  const iterations = Math.max(1, Math.floor(options.iterations ?? 64))
  return {
    cacheKey: cacheKey ?? '',
    lattice,
    transitions,
    discount,
    iterations,
    completedIterations: 0,
    stateIndex: 0,
    nextValues: new Float64Array(values),
  }
}

function advanceBuild(build: StateLatticePolicyBuild, workBudget: number): StateLatticePolicy | null {
  const { lattice, transitions, discount, iterations } = build
  const { actions, policy, unreachableCost, values } = lattice
  let remaining = workBudget
  while (remaining > 0 && build.completedIterations < iterations) {
    const stateIndex = build.stateIndex
    if (values[stateIndex] === 0) {
      policy[stateIndex] = -1
    } else {
      let bestCost = values[stateIndex]
      let bestAction = policy[stateIndex]
      for (let actionIndex = 0; actionIndex < actions.length; actionIndex += 1) {
        const transitionOffset = stateIndex * actions.length + actionIndex
        const nextState = transitions.nextState[transitionOffset]
        if (nextState < 0) continue
        const transitionValue = values[nextState]
        if (transitionValue >= unreachableCost) continue
        const total = transitions.stageCost[transitionOffset] + discount * transitionValue
        if (total < bestCost) {
          bestCost = total
          bestAction = actionIndex
        }
      }
      build.nextValues[stateIndex] = bestCost
      policy[stateIndex] = bestAction
    }

    remaining -= 1
    build.stateIndex += 1
    if (build.stateIndex < values.length) continue

    values.set(build.nextValues)
    build.completedIterations += 1
    build.stateIndex = 0
    if (build.completedIterations < iterations) build.nextValues = new Float64Array(values)
  }
  return build.completedIterations >= iterations ? lattice : null
}

export function lookupStateLatticeAction(lattice: StateLatticePolicy, state: RobotState): StateLatticeAction | null {
  const x = Math.round((state.x - lattice.origin.x) / lattice.resolution)
  const y = Math.round((state.y - lattice.origin.y) / lattice.resolution)
  if (x < 0 || y < 0 || x >= lattice.width || y >= lattice.height) return null
  const h = headingBin(state.theta, lattice.headingBins)
  const actionIndex = nearestPolicyActionIndex(lattice, x, y, h)
  return actionIndex >= 0 ? lattice.actions[actionIndex] : null
}

export function lookupStateLatticeValue(lattice: StateLatticePolicy, state: RobotState): number {
  const x = Math.round((state.x - lattice.origin.x) / lattice.resolution)
  const y = Math.round((state.y - lattice.origin.y) / lattice.resolution)
  if (x < 0 || y < 0 || x >= lattice.width || y >= lattice.height) return lattice.unreachableCost
  const h = headingBin(state.theta, lattice.headingBins)
  return lattice.values[toStateIndex(lattice, x, y, h)]
}

export function rolloutStateLatticePolicy(args: {
  initial: RobotState
  environment: Environment
  policy: StateLatticePolicy
  parameters: Pick<PlannerParameters, 'dt' | 'robotRadius'>
  steps: number
  stepRobot?: typeof stepRobotInEnvironment
}): RobotState[] {
  const trajectory: RobotState[] = [args.initial]
  let robot = args.initial
  const stepper = args.stepRobot ?? stepRobotInEnvironment
  for (let step = 0; step < args.steps; step += 1) {
    const action = lookupStateLatticeAction(args.policy, robot)
    if (!action) break
    robot = stepper(robot, action, args.parameters.dt, args.environment, args.parameters)
    trajectory.push(robot)
  }
  return trajectory
}

function buildTransitionTable(
  lattice: StateLatticePolicy,
  environment: Environment,
  blocked: Uint8Array,
  robotRadius: number,
  socialRisks: StateLatticeSocialRisk[],
) {
  const transitionCount = lattice.values.length * lattice.actions.length
  const nextState = new Int32Array(transitionCount)
  const stageCost = new Float64Array(transitionCount)
  nextState.fill(-1)
  for (let h = 0; h < lattice.headingBins; h += 1) {
    for (let y = 0; y < lattice.height; y += 1) {
      for (let x = 0; x < lattice.width; x += 1) {
        if (blocked[toCellIndex(lattice, x, y)]) continue
        const state = { ...cellCenter(lattice, x, y), theta: headingForBin(h, lattice.headingBins) }
        const stateIndex = toStateIndex(lattice, x, y, h)
        for (let actionIndex = 0; actionIndex < lattice.actions.length; actionIndex += 1) {
          const action = lattice.actions[actionIndex]
          const transitionOffset = stateIndex * lattice.actions.length + actionIndex
          const stepped = stepRobotInEnvironment(state, action, lattice.actionDuration, environment, { robotRadius })
          const nx = Math.round((stepped.x - lattice.origin.x) / lattice.resolution)
          const ny = Math.round((stepped.y - lattice.origin.y) / lattice.resolution)
          if (nx < 0 || ny < 0 || nx >= lattice.width || ny >= lattice.height) continue
          if (blocked[toCellIndex(lattice, nx, ny)]) continue
          if (distance(state, stepped) < lattice.resolution * 0.15 && Math.abs(action.omega) < 0.01) continue
          const nh = headingBin(stepped.theta, lattice.headingBins)
          nextState[transitionOffset] = toStateIndex(lattice, nx, ny, nh)
          stageCost[transitionOffset] =
            actionStageCost(action, lattice.actionDuration) +
            socialRiskStageCost(stepped, socialRisks, robotRadius, lattice.actionDuration)
        }
      }
    }
  }
  return { nextState, stageCost }
}

function actionStageCost(action: ControlInput, duration: number) {
  const distanceCost = Math.max(0.02, Math.abs(action.v) * duration)
  const turnCost = 0.08 * Math.abs(action.omega) * duration
  const reverseCost = action.v < 0 ? 0.25 : 0
  return distanceCost + turnCost + reverseCost
}

function socialRiskStageCost(
  point: Vec2,
  socialRisks: StateLatticeSocialRisk[],
  robotRadius: number,
  duration: number,
) {
  return socialRisks.reduce((total, risk) => {
    const preferredClearance = risk.radius + robotRadius + 0.55
    const clearanceDeficit = Math.max(0, preferredClearance - distance(point, risk))
    return total + (risk.weight ?? 1) * duration * clearanceDeficit * clearanceDeficit
  }, 0)
}

function isBlocked(point: Vec2, environment: Environment, robotRadius: number) {
  if (environment.walls.length > 0 && nearestWall(point, environment.walls).distance < robotRadius) return true
  return environment.obstacles.some((obstacle) => distance(point, obstacle) <= obstacle.radius + robotRadius)
}

function stateLatticePolicyCacheKey(environment: Environment, options: StateLatticePolicyOptions) {
  const actions = options.actions ?? defaultActions
  return [
    `r=${rounded(options.resolution)}`,
    `h=${Math.max(4, Math.floor(options.headingBins))}`,
    `rr=${rounded(options.robotRadius)}`,
    `dt=${rounded(options.actionDuration ?? 1)}`,
    `d=${rounded(options.discount ?? 0.98)}`,
    `i=${Math.max(1, Math.floor(options.iterations ?? 64))}`,
    `p=${rounded(options.padding ?? options.resolution * 2)}`,
    `u=${rounded(options.unreachableCost ?? 1_000_000)}`,
    `g=${pointKey(environment.goal)}:${rounded(environment.goalRadius ?? options.resolution)}`,
    `walls=${environment.walls
      .map((wall) => `${wall.id}:${pointKey(wall.a)}-${pointKey(wall.b)}`)
      .sort()
      .join('|')}`,
    `obstacles=${environment.obstacles
      .map((obstacle) => `${obstacle.id}:${pointKey(obstacle)}:${rounded(obstacle.radius)}`)
      .sort()
      .join('|')}`,
    `actions=${actions.map((action) => `${action.id}:${rounded(action.v)}:${rounded(action.omega)}`).join('|')}`,
    `social=${(options.socialRisks ?? [])
      .map((risk) => `${risk.id ?? ''}:${pointKey(risk)}:${rounded(risk.radius)}:${rounded(risk.weight ?? 1)}`)
      .sort()
      .join('|')}`,
  ].join(';')
}

function pointKey(point: Vec2) {
  return `${rounded(point.x)},${rounded(point.y)}`
}

function rounded(value: number) {
  return Math.round(value * 1000) / 1000
}

function cellCenter(lattice: Pick<StateLatticePolicy, 'origin' | 'resolution'>, x: number, y: number): Vec2 {
  return { x: lattice.origin.x + x * lattice.resolution, y: lattice.origin.y + y * lattice.resolution }
}

function toCellIndex(lattice: Pick<StateLatticePolicy, 'width'>, x: number, y: number) {
  return y * lattice.width + x
}

function toStateIndex(
  lattice: Pick<StateLatticePolicy, 'width' | 'height' | 'headingBins'>,
  x: number,
  y: number,
  h: number,
) {
  return (h * lattice.height + y) * lattice.width + x
}

function nearestPolicyActionIndex(lattice: StateLatticePolicy, x: number, y: number, h: number) {
  const exact = lattice.policy[toStateIndex(lattice, x, y, h)]
  if (exact >= 0) return exact
  let best: { actionIndex: number; value: number; offset: number } | null = null
  for (let dh = -1; dh <= 1; dh += 1) {
    const heading = (h + dh + lattice.headingBins) % lattice.headingBins
    for (let radius = 1; radius <= 2; radius += 1) {
      for (let dy = -radius; dy <= radius; dy += 1) {
        for (let dx = -radius; dx <= radius; dx += 1) {
          if (Math.max(Math.abs(dx), Math.abs(dy)) !== radius) continue
          const nx = x + dx
          const ny = y + dy
          if (nx < 0 || ny < 0 || nx >= lattice.width || ny >= lattice.height) continue
          const stateIndex = toStateIndex(lattice, nx, ny, heading)
          const actionIndex = lattice.policy[stateIndex]
          if (actionIndex < 0) continue
          const value = lattice.values[stateIndex]
          const offset = Math.abs(dx) + Math.abs(dy) + Math.abs(dh)
          if (!best || value < best.value || (value === best.value && offset < best.offset)) {
            best = { actionIndex, value, offset }
          }
        }
      }
    }
  }
  return best?.actionIndex ?? -1
}

function headingForBin(h: number, headingBins: number) {
  return normAngle((h / headingBins) * Math.PI * 2)
}

function headingBin(theta: number, headingBins: number) {
  const wrapped = normAngle(theta)
  const positive = wrapped < 0 ? wrapped + Math.PI * 2 : wrapped
  return Math.round((positive / (Math.PI * 2)) * headingBins) % headingBins
}
