import { clampControl, stepRobot } from '../simulation/dynamics'
import { mulberry32, nearestWall, normAngle, wallTangentAngle } from '../simulation/math'
import type { ControlInput, PlannerParameters, PlanningResult, SimulationState } from '../simulation/types'
import { rolloutCandidate } from './rollout'

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
  const templates: ControlInput[] = [
    { v: 0, omega: 0 },
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
    robot = stepRobot(robot, control, p.dt)
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
      controls: sampleControlSequence(args.state, args.parameters, rand, index),
      previousControl: args.state.previousControl,
      parameters: args.parameters,
    }),
  )
  const selected = candidates.reduce((best, item) => (item.cost.total < best.cost.total ? item : best), candidates[0])
  return { candidates, selected, bestControl: selected.controls[0] ?? { v: 0, omega: 0 } }
}
