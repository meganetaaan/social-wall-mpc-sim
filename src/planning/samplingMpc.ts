import { clampControl, stepRobotInEnvironment } from '../simulation/dynamics'
import { mulberry32, nearestWall, normAngle, wallTangentAngle } from '../simulation/math'
import type {
  ControlInput,
  ObjectBelief,
  PlannerParameters,
  PlanningResult,
  RobotState,
  SimulationState,
} from '../simulation/types'
import { valueFieldReliability } from './cost'
import { rolloutCandidate } from './rollout'
import { valueFieldDescentHeading } from './valueField'

function beliefObjectVelocityCap(robot: RobotState, objects: ObjectBelief[] | undefined, p: PlannerParameters) {
  let cap = p.vMax
  for (const object of objects ?? []) {
    if (object.pHuman < 0.5) continue
    const dx = object.centroid.x - robot.x
    const dy = object.centroid.y - robot.y
    const distance = Math.hypot(dx, dy)
    const bearing = normAngle(Math.atan2(dy, dx) - robot.theta)
    if (Math.abs(bearing) > p.sensorFov / 2 || distance > p.dPref) continue
    const risk = object.pHuman * Math.max(0, (p.dPref - distance) / Math.max(0.001, p.dPref - p.dMin))
    cap = Math.min(cap, p.vMax * Math.max(0, 1 - risk))
  }
  return Math.max(0, cap)
}

function applyBeliefObjectVelocityCap(controls: ControlInput[], state: SimulationState, p: PlannerParameters) {
  const cap = beliefObjectVelocityCap(state.robot, state.belief.objectBeliefs, p)
  if (cap >= p.vMax) return controls
  return controls.map((control) => ({ ...control, v: Math.min(control.v, cap) }))
}

function sampleControlSequence(
  state: SimulationState,
  p: PlannerParameters,
  rand: () => number,
  index: number,
): ControlInput[] {
  const nearest = nearestWall({ x: state.robot.x, y: state.robot.y }, state.environment.walls)
  const headingError = normAngle(wallTangentAngle(nearest.wall) - state.robot.theta)
  if (index === 3) return guideSequence(state, p, 'goal')
  if (index === 4) return guideSequence(state, p, 'blend')
  if (index === 5 && state.environment.valueField && valueFieldReliability(state.environment) > 0.35)
    return guideValueFieldSequence(state, p)
  const templates: ControlInput[] = [
    { v: 0, omega: 0 },
    { v: -0.18, omega: 0 },
    { v: -0.12, omega: -0.45 },
    { v: -0.12, omega: 0.45 },
    { v: 0.18, omega: headingError * 0.8 },
    { v: 0.45, omega: headingError },
    { v: 0.25, omega: -0.8 },
    { v: 0.25, omega: 0.8 },
  ]
  const base = templates[index % templates.length]
  const controls: ControlInput[] = []
  let current = base
  const horizonSteps = Math.max(1, Math.floor(p.horizonSteps))
  for (let k = 0; k < horizonSteps; k += 1) {
    const noiseScale = index < templates.length ? 0.18 : 0.55
    current = clampControl(
      {
        v: current.v * 0.78 + (base.v + (rand() - 0.5) * noiseScale) * 0.22,
        omega: current.omega * 0.68 + (base.omega + (rand() - 0.5) * noiseScale * 3) * 0.32,
      },
      p,
    )
    controls.push(current)
  }
  return controls
}

function guideSequence(state: SimulationState, p: PlannerParameters, kind: 'goal' | 'blend'): ControlInput[] {
  const controls: ControlInput[] = []
  let robot = state.robot
  let previous = state.previousControl
  const horizonSteps = Math.max(1, Math.floor(p.horizonSteps))
  for (let k = 0; k < horizonSteps; k += 1) {
    const nearest = nearestWall({ x: robot.x, y: robot.y }, state.environment.walls)
    const wallHeading = wallTangentAngle(nearest.wall)
    const goalHeading = Math.atan2(state.environment.goal.y - robot.y, state.environment.goal.x - robot.x)
    const targetHeading =
      kind === 'goal'
        ? goalHeading
        : robot.theta + 0.45 * normAngle(wallHeading - robot.theta) + 0.55 * normAngle(goalHeading - robot.theta)
    const headingError = normAngle(targetHeading - robot.theta)
    const control = clampControl(
      {
        v: previous.v * 0.35 + (kind === 'goal' ? 0.68 : 0.56) * 0.65,
        omega: previous.omega * 0.2 + headingError * (kind === 'goal' ? 1.3 : 1.05),
      },
      p,
    )
    controls.push(control)
    robot = stepRobotInEnvironment(robot, control, p.dt, state.environment, p)
    previous = control
  }
  return controls
}

function guideValueFieldSequence(state: SimulationState, p: PlannerParameters): ControlInput[] {
  const controls: ControlInput[] = []
  const field = state.environment.valueField
  let robot = state.robot
  let previous = state.previousControl
  const horizonSteps = Math.max(1, Math.floor(p.horizonSteps))
  for (let k = 0; k < horizonSteps; k += 1) {
    const descent = field ? valueFieldDescentHeading(field, robot) : null
    const nearest = nearestWall({ x: robot.x, y: robot.y }, state.environment.walls)
    const wallHeading = wallTangentAngle(nearest.wall)
    const targetHeading = descent?.heading ?? wallHeading
    const headingError = normAngle(targetHeading - robot.theta)
    const control = clampControl(
      {
        v: previous.v * 0.2 + 0.48,
        omega: previous.omega * 0.15 + headingError * 1.25,
      },
      p,
    )
    controls.push(control)
    robot = stepRobotInEnvironment(robot, control, p.dt, state.environment, p)
    previous = control
  }
  return controls
}

export function planSamplingMpc(args: {
  state: SimulationState
  parameters: PlannerParameters
  seed?: number
}): PlanningResult {
  const seed = args.seed ?? args.parameters.seed
  const rand = mulberry32(seed)
  const sampleCount = Math.max(1, Math.floor(args.parameters.sampleCount))
  const candidates = Array.from({ length: sampleCount }, (_, index) =>
    rolloutCandidate({
      robot: args.state.robot,
      humans: args.state.humans,
      environment: args.state.environment,
      belief: args.state.belief,
      controls: applyBeliefObjectVelocityCap(
        sampleControlSequence(args.state, args.parameters, rand, index),
        args.state,
        args.parameters,
      ),
      previousControl: args.state.previousControl,
      parameters: args.parameters,
    }),
  )
  const selected = candidates.reduce((best, item) => (item.cost.total < best.cost.total ? item : best), candidates[0])
  return { candidates, selected, bestControl: selected.controls[0] ?? { v: 0, omega: 0 } }
}
