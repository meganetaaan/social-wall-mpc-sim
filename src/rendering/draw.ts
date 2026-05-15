import { goalRadius } from '../simulation/goal'
import type {
  BeliefState,
  CandidateRollout,
  Environment,
  HumanState,
  RobotState,
  Vec2,
  WallObservation,
  WallSegment,
} from '../simulation/types'

type Transform = { scale: number; offsetX: number; offsetY: number; height: number }
const worldToCanvas = (p: Vec2, t: Transform): Vec2 => ({
  x: t.offsetX + p.x * t.scale,
  y: t.height - (t.offsetY + p.y * t.scale),
})
const pointOnWall = (wall: WallSegment, t: number): Vec2 => ({
  x: wall.a.x + (wall.b.x - wall.a.x) * t,
  y: wall.a.y + (wall.b.y - wall.a.y) * t,
})

export function drawScene(args: {
  ctx: CanvasRenderingContext2D
  width: number
  height: number
  environment: Environment
  belief?: BeliefState
  robot: RobotState
  humans: HumanState[]
  trace: Vec2[]
  candidates: CandidateRollout[]
  selected?: CandidateRollout
  currentObservations?: WallObservation[]
  dMin: number
  dPref: number
  sensorRadius: number
  sensorFov: number
  goalReached?: boolean
}) {
  const { ctx, width, height } = args
  const transform = { scale: Math.min(width / 10.8, height / 5.6), offsetX: 28, offsetY: 28, height }
  ctx.clearRect(0, 0, width, height)
  ctx.fillStyle = '#0f172a'
  ctx.fillRect(0, 0, width, height)
  ctx.strokeStyle = '#1e293b'
  ctx.lineWidth = 1
  for (let x = 0; x <= 10; x += 1) {
    const a = worldToCanvas({ x, y: 0 }, transform)
    const b = worldToCanvas({ x, y: 5 }, transform)
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(b.x, b.y)
    ctx.stroke()
  }
  for (let y = 0; y <= 5; y += 1) {
    const a = worldToCanvas({ x: 0, y }, transform)
    const b = worldToCanvas({ x: 10, y }, transform)
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(b.x, b.y)
    ctx.stroke()
  }

  drawSensorFov(ctx, args.robot, transform, args.sensorRadius, args.sensorFov)

  ctx.lineCap = 'round'
  ctx.lineWidth = 8
  ctx.strokeStyle = 'rgba(148, 163, 184, 0.32)'
  for (const wall of args.environment.walls) {
    const a = worldToCanvas(wall.a, transform)
    const b = worldToCanvas(wall.b, transform)
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(b.x, b.y)
    ctx.stroke()
  }

  if (args.belief) {
    ctx.lineWidth = 6
    for (const estimated of args.belief.estimatedWalls) {
      if (estimated.tMax <= estimated.tMin) continue
      const wall = args.environment.walls.find((candidate) => candidate.id === estimated.wallId)
      if (!wall) continue
      const a = worldToCanvas(pointOnWall(wall, estimated.tMin), transform)
      const b = worldToCanvas(pointOnWall(wall, estimated.tMax), transform)
      ctx.strokeStyle = `rgba(125, 211, 252, ${Math.max(0.22, estimated.confidence).toFixed(2)})`
      ctx.beginPath()
      ctx.moveTo(a.x, a.y)
      ctx.lineTo(b.x, b.y)
      ctx.stroke()
    }
    ctx.fillStyle = '#bae6fd'
    ctx.font = '12px Inter, sans-serif'
    ctx.fillText('estimated wall coverage', 32, 24)
  }

  if (args.currentObservations && args.currentObservations.length > 0) {
    ctx.lineWidth = 2
    ctx.strokeStyle = 'rgba(186, 230, 253, 0.46)'
    const robot = worldToCanvas(args.robot, transform)
    for (const observation of args.currentObservations) {
      const target = worldToCanvas(observation.rayTarget, transform)
      ctx.beginPath()
      ctx.moveTo(robot.x, robot.y)
      ctx.lineTo(target.x, target.y)
      ctx.stroke()
      const wall = args.environment.walls.find((candidate) => candidate.id === observation.wallId)
      if (!wall) continue
      const a = worldToCanvas(pointOnWall(wall, observation.tMin), transform)
      const b = worldToCanvas(pointOnWall(wall, observation.tMax), transform)
      ctx.lineWidth = 9
      ctx.strokeStyle = 'rgba(34, 211, 238, 0.78)'
      ctx.beginPath()
      ctx.moveTo(a.x, a.y)
      ctx.lineTo(b.x, b.y)
      ctx.stroke()
      ctx.lineWidth = 2
      ctx.strokeStyle = 'rgba(186, 230, 253, 0.46)'
    }
  }

  for (const obstacle of args.environment.obstacles) {
    const p = worldToCanvas(obstacle, transform)
    ctx.fillStyle = '#64748b'
    ctx.beginPath()
    ctx.arc(p.x, p.y, obstacle.radius * transform.scale, 0, Math.PI * 2)
    ctx.fill()
  }

  for (const candidate of args.candidates.slice(0, 80))
    drawPath(ctx, candidate.trajectory, transform, 'rgba(56, 189, 248, 0.10)', 1)
  if (args.selected) drawPath(ctx, args.selected.trajectory, transform, '#facc15', 3)
  drawPath(
    ctx,
    args.trace.map((p) => ({ ...p, theta: 0 })),
    transform,
    'rgba(34, 197, 94, 0.75)',
    2,
  )

  for (const human of args.humans) {
    const p = worldToCanvas(human, transform)
    ctx.fillStyle = 'rgba(244, 114, 182, 0.12)'
    ctx.beginPath()
    ctx.arc(p.x, p.y, args.dPref * transform.scale, 0, Math.PI * 2)
    ctx.fill()
    ctx.fillStyle = 'rgba(239, 68, 68, 0.22)'
    ctx.beginPath()
    ctx.arc(p.x, p.y, args.dMin * transform.scale, 0, Math.PI * 2)
    ctx.fill()
    ctx.fillStyle = '#fb7185'
    ctx.beginPath()
    ctx.arc(p.x, p.y, human.radius * transform.scale, 0, Math.PI * 2)
    ctx.fill()
  }

  const goal = worldToCanvas(args.environment.goal, transform)
  const reached = args.goalReached ?? false
  ctx.strokeStyle = reached ? 'rgba(187, 247, 208, 0.95)' : 'rgba(34, 197, 94, 0.62)'
  ctx.lineWidth = reached ? 6 : 4
  ctx.beginPath()
  ctx.arc(goal.x, goal.y, goalRadius(args.environment) * transform.scale, 0, Math.PI * 2)
  ctx.stroke()
  ctx.fillStyle = '#22c55e'
  ctx.beginPath()
  ctx.arc(goal.x, goal.y, 0.16 * transform.scale, 0, Math.PI * 2)
  ctx.fill()
  ctx.fillStyle = '#bbf7d0'
  ctx.font = 'bold 14px Inter, sans-serif'
  ctx.fillText(reached ? 'GOAL REACHED' : 'GOAL', goal.x + 12, goal.y - 12)

  const r = worldToCanvas(args.robot, transform)
  ctx.fillStyle = '#38bdf8'
  ctx.beginPath()
  ctx.arc(r.x, r.y, 0.22 * transform.scale, 0, Math.PI * 2)
  ctx.fill()
  ctx.strokeStyle = '#e0f2fe'
  ctx.lineWidth = 3
  ctx.beginPath()
  ctx.moveTo(r.x, r.y)
  ctx.lineTo(
    r.x + Math.cos(args.robot.theta) * 0.42 * transform.scale,
    r.y - Math.sin(args.robot.theta) * 0.42 * transform.scale,
  )
  ctx.stroke()
}

function drawSensorFov(
  ctx: CanvasRenderingContext2D,
  robot: RobotState,
  transform: Transform,
  sensorRadius: number,
  sensorFov: number,
) {
  const center = worldToCanvas(robot, transform)
  const start = -robot.theta - sensorFov / 2
  const end = -robot.theta + sensorFov / 2
  ctx.fillStyle = 'rgba(14, 165, 233, 0.08)'
  ctx.strokeStyle = 'rgba(125, 211, 252, 0.18)'
  ctx.lineWidth = 1
  ctx.beginPath()
  ctx.moveTo(center.x, center.y)
  ctx.arc(center.x, center.y, sensorRadius * transform.scale, start, end)
  ctx.lineTo(center.x, center.y)
  ctx.fill()
  ctx.stroke()
}

function drawPath(
  ctx: CanvasRenderingContext2D,
  trajectory: RobotState[],
  transform: Transform,
  color: string,
  width: number,
) {
  if (trajectory.length < 2) return
  ctx.strokeStyle = color
  ctx.lineWidth = width
  ctx.beginPath()
  const start = worldToCanvas(trajectory[0], transform)
  ctx.moveTo(start.x, start.y)
  for (const pose of trajectory.slice(1)) {
    const p = worldToCanvas(pose, transform)
    ctx.lineTo(p.x, p.y)
  }
  ctx.stroke()
}
