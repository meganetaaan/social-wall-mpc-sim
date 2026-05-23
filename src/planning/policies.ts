import { clampControl } from '../simulation/dynamics'
import { distance, nearestWall, normAngle, wallTangentAngle } from '../simulation/math'
import type { ControlInput, PlannerMode, PlannerParameters, PlanningResult, SimulationState } from '../simulation/types'
import { rolloutCandidate } from './rollout'
import { planSamplingMpc } from './samplingMpc'
import { getCachedStateLatticePolicy, lookupStateLatticeAction } from './stateLatticeValueIteration'

export const defaultPlannerMode: PlannerMode = 'belief-mpc'

const reactiveStopBuffer = 0.15

export function planWithPolicy(args: {
  mode?: PlannerMode
  state: SimulationState
  parameters: PlannerParameters
  seed?: number
}): PlanningResult {
  const mode = args.mode ?? defaultPlannerMode
  if (mode === 'belief-mpc') return planSamplingMpc(args)
  if (mode === 'state-lattice') return planStateLattice(args.state, args.parameters)
  if (mode === 'reactive-stop')
    return planBaseline(args.state, args.parameters, reactiveStopControl(args.state, args.parameters))
  return planBaseline(args.state, args.parameters, wallOnlyControl(args.state, args.parameters), true)
}

function planStateLattice(state: SimulationState, parameters: PlannerParameters): PlanningResult {
  const lattice = getCachedStateLatticePolicy(state.environment, {
    resolution: 0.4,
    headingBins: 16,
    robotRadius: parameters.robotRadius,
    discount: 0.98,
    iterations: 180,
    socialRisks: [
      ...(state.belief.objectBeliefs ?? []).map((object) => ({
        id: object.id,
        x: object.centroid.x,
        y: object.centroid.y,
        radius: object.radius,
        weight: 1 + object.pHuman + object.pStatic,
      })),
      ...state.humans.map((human) => ({
        id: human.id,
        x: human.x,
        y: human.y,
        radius: human.radius,
        weight: human.vx === 0 && human.vy === 0 ? 1.5 : 0.7,
      })),
    ],
  })
  const control = lookupStateLatticeAction(lattice, state.robot) ?? { v: 0, omega: 0 }
  const selected = rolloutCandidate({
    robot: state.robot,
    humans: state.humans,
    environment: state.environment,
    belief: state.belief,
    controls: [clampControl(control, parameters)],
    previousControl: state.previousControl,
    parameters,
  })
  return { candidates: [selected], selected, bestControl: selected.controls[0] ?? control }
}

function planBaseline(
  state: SimulationState,
  parameters: PlannerParameters,
  control: ControlInput,
  ignoreHumans = false,
): PlanningResult {
  const controls = Array.from({ length: Math.max(1, Math.floor(parameters.horizonSteps)) }, () => control)
  const selected = rolloutCandidate({
    robot: state.robot,
    humans: ignoreHumans ? [] : state.humans,
    environment: state.environment,
    belief: state.belief,
    controls,
    previousControl: state.previousControl,
    parameters,
  })
  return { candidates: [selected], selected, bestControl: selected.controls[0] ?? control }
}

function wallOnlyControl(state: SimulationState, parameters: PlannerParameters): ControlInput {
  const nearest = nearestWall({ x: state.robot.x, y: state.robot.y }, state.environment.walls)
  const headingError = normAngle(wallTangentAngle(nearest.wall) - state.robot.theta)
  const distanceError = nearest.distance - parameters.dWallTarget
  return clampControl(
    { v: Math.min(0.42, parameters.vMax), omega: headingError * 1.1 - distanceError * 0.55 },
    parameters,
  )
}

function reactiveStopControl(state: SimulationState, parameters: PlannerParameters): ControlInput {
  const shouldStop = state.humans.some((human) => distance(state.robot, human) < parameters.dMin + reactiveStopBuffer)
  return shouldStop ? { v: 0, omega: 0 } : wallOnlyControl(state, parameters)
}
