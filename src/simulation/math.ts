import type { Vec2, WallSegment } from './types'

export const clamp = (value: number, min: number, max: number) => Math.min(max, Math.max(min, value))
export const normAngle = (angle: number) => Math.atan2(Math.sin(angle), Math.cos(angle))
export const distance = (a: Vec2, b: Vec2) => Math.hypot(a.x - b.x, a.y - b.y)
export const dot = (a: Vec2, b: Vec2) => a.x * b.x + a.y * b.y
export const sub = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x - b.x, y: a.y - b.y })
export const add = (a: Vec2, b: Vec2): Vec2 => ({ x: a.x + b.x, y: a.y + b.y })
export const scale = (a: Vec2, s: number): Vec2 => ({ x: a.x * s, y: a.y * s })
export const length = (a: Vec2) => Math.hypot(a.x, a.y)
export const wallTangentAngle = (wall: WallSegment) => Math.atan2(wall.b.y - wall.a.y, wall.b.x - wall.a.x)

export function nearestPointOnSegment(point: Vec2, wall: WallSegment): Vec2 {
  const ab = sub(wall.b, wall.a)
  const denom = dot(ab, ab) || 1
  const t = clamp(dot(sub(point, wall.a), ab) / denom, 0, 1)
  return add(wall.a, scale(ab, t))
}

export function distanceToWall(point: Vec2, wall: WallSegment) {
  const nearest = nearestPointOnSegment(point, wall)
  return { distance: distance(point, nearest), nearest }
}

export function nearestWall(point: Vec2, walls: WallSegment[]) {
  if (walls.length === 0) throw new Error('environment requires at least one wall')
  return walls.map((wall) => ({ wall, ...distanceToWall(point, wall) })).sort((a, b) => a.distance - b.distance)[0]
}

export type Random = () => number
export function mulberry32(seed: number): Random {
  let t = seed >>> 0
  return () => {
    t += 0x6d2b79f5
    let r = Math.imul(t ^ (t >>> 15), 1 | t)
    r ^= r + Math.imul(r ^ (r >>> 7), 61 | r)
    return ((r ^ (r >>> 14)) >>> 0) / 4294967296
  }
}
