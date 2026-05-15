import { clampControl } from '../simulation/dynamics'
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
