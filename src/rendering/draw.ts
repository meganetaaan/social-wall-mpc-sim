import type { CandidateRollout, Environment, HumanState, RobotState, Vec2 } from '../simulation/types'

type Transform = { scale: number; offsetX: number; offsetY: number; height: number }
const worldToCanvas = (p: Vec2, t: Transform): Vec2 => ({
  x: t.offsetX + p.x * t.scale,
  y: t.height - (t.offsetY + p.y * t.scale),
})

export function drawScene(args: {
  ctx: CanvasRenderingContext2D
  width: number
  height: number
  environment: Environment
  robot: RobotState
  humans: HumanState[]
  trace: Vec2[]
  candidates: CandidateRollout[]
  selected?: CandidateRollout
  dMin: number
  dPref: number
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

  ctx.lineCap = 'round'
  ctx.lineWidth = 8
  ctx.strokeStyle = '#94a3b8'
  for (const wall of args.environment.walls) {
    const a = worldToCanvas(wall.a, transform)
    const b = worldToCanvas(wall.b, transform)
    ctx.beginPath()
    ctx.moveTo(a.x, a.y)
    ctx.lineTo(b.x, b.y)
    ctx.stroke()
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
  ctx.strokeStyle = 'rgba(34, 197, 94, 0.62)'
  ctx.lineWidth = 4
  ctx.beginPath()
  ctx.arc(goal.x, goal.y, 0.34 * transform.scale, 0, Math.PI * 2)
  ctx.stroke()
  ctx.fillStyle = '#22c55e'
  ctx.beginPath()
  ctx.arc(goal.x, goal.y, 0.16 * transform.scale, 0, Math.PI * 2)
  ctx.fill()
  ctx.fillStyle = '#bbf7d0'
  ctx.font = 'bold 14px Inter, sans-serif'
  ctx.fillText('GOAL', goal.x + 12, goal.y - 12)

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
