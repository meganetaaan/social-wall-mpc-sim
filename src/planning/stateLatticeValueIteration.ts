import { stepRobotInEnvironment } from '../simulation/dynamics'
import { distance, nearestWall, normAngle } from '../simulation/math'
import type { ControlInput, Environment, PlannerParameters, RobotState, Vec2 } from '../simulation/types'

export type StateLatticeAction = ControlInput & { id: string }

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
}

const defaultActions: StateLatticeAction[] = [
  { id: 'forward', v: 0.45, omega: 0 },
  { id: 'forward-left', v: 0.34, omega: 0.85 },
  { id: 'forward-right', v: 0.34, omega: -0.85 },
  { id: 'reverse', v: -0.18, omega: 0 },
]

export function createStateLatticePolicy(
  environment: Environment,
  options: StateLatticePolicyOptions,
): StateLatticePolicy {
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

  const transitions = buildTransitionTable(lattice, environment, blocked, options.robotRadius)
  const discount = options.discount ?? 0.98
  const iterations = Math.max(1, Math.floor(options.iterations ?? 64))
  for (let iteration = 0; iteration < iterations; iteration += 1) {
    const nextValues = new Float64Array(values)
    for (let stateIndex = 0; stateIndex < stateCount; stateIndex += 1) {
      if (values[stateIndex] === 0) {
        policy[stateIndex] = -1
        continue
      }
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
      nextValues[stateIndex] = bestCost
      policy[stateIndex] = bestAction
    }
    values.set(nextValues)
  }
  return lattice
}

export function lookupStateLatticeAction(lattice: StateLatticePolicy, state: RobotState): StateLatticeAction | null {
  const x = Math.round((state.x - lattice.origin.x) / lattice.resolution)
  const y = Math.round((state.y - lattice.origin.y) / lattice.resolution)
  if (x < 0 || y < 0 || x >= lattice.width || y >= lattice.height) return null
  const h = headingBin(state.theta, lattice.headingBins)
  const actionIndex = lattice.policy[toStateIndex(lattice, x, y, h)]
  return actionIndex >= 0 ? lattice.actions[actionIndex] : null
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
          stageCost[transitionOffset] = actionStageCost(action, lattice.actionDuration)
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

function isBlocked(point: Vec2, environment: Environment, robotRadius: number) {
  if (environment.walls.length > 0 && nearestWall(point, environment.walls).distance < robotRadius) return true
  return environment.obstacles.some((obstacle) => distance(point, obstacle) <= obstacle.radius + robotRadius)
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

function headingForBin(h: number, headingBins: number) {
  return normAngle((h / headingBins) * Math.PI * 2)
}

function headingBin(theta: number, headingBins: number) {
  const wrapped = normAngle(theta)
  const positive = wrapped < 0 ? wrapped + Math.PI * 2 : wrapped
  return Math.round((positive / (Math.PI * 2)) * headingBins) % headingBins
}
