import { distance, nearestWall } from '../simulation/math'
import type { Environment, GridValueField, Vec2, WallSegment } from '../simulation/types'

export type GridValueFieldOptions = {
  resolution: number
  robotRadius: number
  padding?: number
  unreachableCost?: number
}

const diagonalCost = Math.SQRT2
const directions = [
  { dx: 1, dy: 0, cost: 1 },
  { dx: -1, dy: 0, cost: 1 },
  { dx: 0, dy: 1, cost: 1 },
  { dx: 0, dy: -1, cost: 1 },
  { dx: 1, dy: 1, cost: diagonalCost },
  { dx: 1, dy: -1, cost: diagonalCost },
  { dx: -1, dy: 1, cost: diagonalCost },
  { dx: -1, dy: -1, cost: diagonalCost },
]

export function createGridValueField(environment: Environment, options: GridValueFieldOptions): GridValueField {
  const resolution = options.resolution
  const padding = options.padding ?? resolution * 2
  const unreachableCost = options.unreachableCost ?? 1_000_000
  const points = [environment.goal, ...environment.walls.flatMap((wall) => [wall.a, wall.b])]
  const minX = Math.min(...points.map((point) => point.x)) - padding
  const minY = Math.min(...points.map((point) => point.y)) - padding
  const maxX = Math.max(...points.map((point) => point.x)) + padding
  const maxY = Math.max(...points.map((point) => point.y)) + padding
  const width = Math.max(1, Math.ceil((maxX - minX) / resolution) + 1)
  const height = Math.max(1, Math.ceil((maxY - minY) / resolution) + 1)
  const field: GridValueField = {
    origin: { x: minX, y: minY },
    width,
    height,
    resolution,
    values: Array.from({ length: width * height }, () => unreachableCost),
    unreachableCost,
  }
  const blocked = Array.from({ length: width * height }, (_, index) =>
    isBlocked(cellCenter(field, index % width, Math.floor(index / width)), environment, options.robotRadius),
  )
  const goalCell = nearestFreeCell(field, environment.goal, blocked)
  if (!goalCell) return field

  const queue = new MinQueue()
  const goalIndex = toIndex(field, goalCell.x, goalCell.y)
  field.values[goalIndex] = 0
  queue.push(goalIndex, 0)

  while (queue.length > 0) {
    const current = queue.pop()
    if (!current) break
    if (current.priority !== field.values[current.value]) continue
    const x = current.value % width
    const y = Math.floor(current.value / width)
    const from = cellCenter(field, x, y)
    for (const direction of directions) {
      const nx = x + direction.dx
      const ny = y + direction.dy
      if (nx < 0 || ny < 0 || nx >= width || ny >= height) continue
      const nextIndex = toIndex(field, nx, ny)
      if (blocked[nextIndex]) continue
      const to = cellCenter(field, nx, ny)
      if (crossesWall(from, to, environment.walls)) continue
      const nextCost = current.priority + direction.cost * resolution
      if (nextCost < field.values[nextIndex]) {
        field.values[nextIndex] = nextCost
        queue.push(nextIndex, nextCost)
      }
    }
  }

  return field
}

export function lookupValueField(field: GridValueField, point: Vec2): number {
  const x = Math.round((point.x - field.origin.x) / field.resolution)
  const y = Math.round((point.y - field.origin.y) / field.resolution)
  if (x < 0 || y < 0 || x >= field.width || y >= field.height) return field.unreachableCost
  return field.values[toIndex(field, x, y)]
}

export function valueFieldDescentHeading(
  field: GridValueField,
  point: Vec2,
  options: { searchRadiusCells?: number } = {},
): { heading: number; slope: number; value: number } | null {
  const cx = Math.round((point.x - field.origin.x) / field.resolution)
  const cy = Math.round((point.y - field.origin.y) / field.resolution)
  if (cx < 0 || cy < 0 || cx >= field.width || cy >= field.height) return null
  const here = field.values[toIndex(field, cx, cy)]
  if (!Number.isFinite(here) || here >= field.unreachableCost) return null
  const searchRadius = options.searchRadiusCells ?? 3
  let best: { x: number; y: number; value: number; distanceCells: number } | null = null
  for (let dy = -searchRadius; dy <= searchRadius; dy += 1) {
    for (let dx = -searchRadius; dx <= searchRadius; dx += 1) {
      if (dx === 0 && dy === 0) continue
      const x = cx + dx
      const y = cy + dy
      if (x < 0 || y < 0 || x >= field.width || y >= field.height) continue
      const value = field.values[toIndex(field, x, y)]
      if (!Number.isFinite(value) || value >= here) continue
      const distanceCells = Math.hypot(dx, dy)
      if (!best || value < best.value || (value === best.value && distanceCells < best.distanceCells)) {
        best = { x, y, value, distanceCells }
      }
    }
  }
  if (!best) return null
  const target = cellCenter(field, best.x, best.y)
  const travelDistance = Math.max(field.resolution, distance(point, target))
  return {
    heading: Math.atan2(target.y - point.y, target.x - point.x),
    slope: (here - best.value) / travelDistance,
    value: here,
  }
}

function nearestFreeCell(
  field: GridValueField,
  point: Vec2,
  blocked: boolean[],
): { x: number; y: number; d: number } | null {
  const centerX = Math.round((point.x - field.origin.x) / field.resolution)
  const centerY = Math.round((point.y - field.origin.y) / field.resolution)
  let best: { x: number; y: number; d: number } | null = null
  const radius = Math.max(field.width, field.height)
  for (let r = 0; r <= radius; r += 1) {
    for (let y = centerY - r; y <= centerY + r; y += 1) {
      for (let x = centerX - r; x <= centerX + r; x += 1) {
        if (x < 0 || y < 0 || x >= field.width || y >= field.height) continue
        if (blocked[toIndex(field, x, y)]) continue
        const d = distance(point, cellCenter(field, x, y))
        if (!best || d < best.d) best = { x, y, d }
      }
    }
    if (best) return best
  }
  return null
}

function isBlocked(point: Vec2, environment: Environment, robotRadius: number) {
  if (nearestWall(point, environment.walls).distance < robotRadius) return true
  return environment.obstacles.some((obstacle) => distance(point, obstacle) <= obstacle.radius + robotRadius)
}

function cellCenter(field: GridValueField, x: number, y: number): Vec2 {
  return { x: field.origin.x + x * field.resolution, y: field.origin.y + y * field.resolution }
}

function toIndex(field: Pick<GridValueField, 'width'>, x: number, y: number) {
  return y * field.width + x
}

function crossesWall(a: Vec2, b: Vec2, walls: WallSegment[]) {
  return walls.some((wall) => segmentsIntersect(a, b, wall.a, wall.b))
}

function segmentsIntersect(a: Vec2, b: Vec2, c: Vec2, d: Vec2) {
  const o1 = orientation(a, b, c)
  const o2 = orientation(a, b, d)
  const o3 = orientation(c, d, a)
  const o4 = orientation(c, d, b)
  return o1 * o2 < 0 && o3 * o4 < 0
}

function orientation(a: Vec2, b: Vec2, c: Vec2) {
  return Math.sign((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x))
}

class MinQueue {
  private items: Array<{ value: number; priority: number }> = []

  get length() {
    return this.items.length
  }

  push(value: number, priority: number) {
    this.items.push({ value, priority })
    this.items.sort((a, b) => b.priority - a.priority)
  }

  pop() {
    return this.items.pop()
  }
}
