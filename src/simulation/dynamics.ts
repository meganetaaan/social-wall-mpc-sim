import { clamp, normAngle } from './math'
import type { ControlInput, HumanState, PlannerParameters, RobotState } from './types'

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
): HumanState[] {
  return humans.map((h) => {
    let x = h.x + h.vx * dt
    let y = h.y + h.vy * dt
    let vx = h.vx
    let vy = h.vy
    if (x < bounds.minX || x > bounds.maxX) {
      vx = -vx
      x = clamp(x, bounds.minX, bounds.maxX)
    }
    if (y < bounds.minY || y > bounds.maxY) {
      vy = -vy
      y = clamp(y, bounds.minY, bounds.maxY)
    }
    return { ...h, x, y, vx, vy }
  })
}
