import { clamp, normAngle } from './math'
import type { ControlInput, HumanState, PlannerParameters, RobotState, Vec2 } from './types'

export function clampControl(u: ControlInput, p: PlannerParameters): ControlInput {
  return { v: clamp(u.v, p.vMin, p.vMax), omega: clamp(u.omega, p.omegaMin, p.omegaMax) }
}

export function stepRobot(robot: RobotState, control: ControlInput, dt: number): RobotState {
  return {
    x: robot.x + control.v * Math.cos(robot.theta) * dt,
    y: robot.y + control.v * Math.sin(robot.theta) * dt,
    theta: normAngle(robot.theta + control.omega * dt),
  }
}

export function predictHumans(
  humans: HumanState[],
  dt: number,
  bounds = { minX: 0.2, maxX: 9.8, minY: 0.2, maxY: 4.8 },
  robot?: RobotState,
  time = 0,
): HumanState[] {
  return humans.map((human) => predictHuman(human, dt, bounds, robot, time))
}

function predictHuman(
  human: HumanState,
  dt: number,
  bounds: { minX: number; maxX: number; minY: number; maxY: number },
  robot: RobotState | undefined,
  time: number,
): HumanState {
  switch (human.motion?.kind) {
    case 'follow-robot':
      return moveToward(
        human,
        followTarget(robot, human.motion.distanceBehind, human.motion.lateralOffset),
        human.motion.speed,
        dt,
        bounds,
      )
    case 'scripted':
      return scriptedHuman(human, time + dt, bounds)
    case 'yield-after':
      if (time < human.motion.yieldAt) return { ...human, vx: 0, vy: 0 }
      return moveToward(human, human.motion.target, human.motion.speed, dt, bounds)
    default:
      return linearHuman(human, dt, bounds)
  }
}

function followTarget(robot: RobotState | undefined, distanceBehind: number, lateralOffset: number): Vec2 {
  if (!robot) return { x: 0, y: 0 }
  return {
    x: robot.x - distanceBehind * Math.cos(robot.theta) - lateralOffset * Math.sin(robot.theta),
    y: robot.y - distanceBehind * Math.sin(robot.theta) + lateralOffset * Math.cos(robot.theta),
  }
}

function scriptedHuman(
  human: HumanState,
  time: number,
  bounds: { minX: number; maxX: number; minY: number; maxY: number },
): HumanState {
  if (human.motion?.kind !== 'scripted' || human.motion.waypoints.length === 0) return human
  const waypoints = human.motion.waypoints
  const duration = waypoints.at(-1)?.at ?? time
  const localTime = human.motion.loop && duration > 0 ? time % duration : time
  let previous = waypoints[0]
  let next = waypoints.at(-1) ?? previous
  for (const waypoint of waypoints) {
    if (waypoint.at <= localTime) previous = waypoint
    if (waypoint.at >= localTime) {
      next = waypoint
      break
    }
  }
  const span = Math.max(0.001, next.at - previous.at)
  const alpha = clamp((localTime - previous.at) / span, 0, 1)
  const x = previous.x + (next.x - previous.x) * alpha
  const y = previous.y + (next.y - previous.y) * alpha
  return {
    ...human,
    x: clamp(x, bounds.minX, bounds.maxX),
    y: clamp(y, bounds.minY, bounds.maxY),
    vx: (next.x - previous.x) / span,
    vy: (next.y - previous.y) / span,
  }
}

function moveToward(
  human: HumanState,
  target: Vec2,
  speed: number,
  dt: number,
  bounds: { minX: number; maxX: number; minY: number; maxY: number },
): HumanState {
  const dx = target.x - human.x
  const dy = target.y - human.y
  const distance = Math.hypot(dx, dy)
  if (distance < 0.001) return { ...human, vx: 0, vy: 0 }
  const step = Math.min(distance, speed * dt)
  const vx = (dx / distance) * speed
  const vy = (dy / distance) * speed
  return {
    ...human,
    x: clamp(human.x + (dx / distance) * step, bounds.minX, bounds.maxX),
    y: clamp(human.y + (dy / distance) * step, bounds.minY, bounds.maxY),
    vx,
    vy,
  }
}

function linearHuman(
  human: HumanState,
  dt: number,
  bounds: { minX: number; maxX: number; minY: number; maxY: number },
): HumanState {
  let x = human.x + human.vx * dt
  let y = human.y + human.vy * dt
  let vx = human.vx
  let vy = human.vy
  if (x < bounds.minX || x > bounds.maxX) {
    vx = -vx
    x = clamp(x, bounds.minX, bounds.maxX)
  }
  if (y < bounds.minY || y > bounds.maxY) {
    vy = -vy
    y = clamp(y, bounds.minY, bounds.maxY)
  }
  return { ...human, x, y, vx, vy }
}
