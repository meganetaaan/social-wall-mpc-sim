import type {
  BeliefState,
  Environment,
  EstimatedLineFeature,
  EstimatedWallSegment,
  WallSegment,
} from '../simulation/types'
import { createGridValueField, type GridValueFieldOptions } from './valueField'

export const spiralValueFieldOptions: GridValueFieldOptions = { resolution: 0.2, robotRadius: 0.22 }

const MIN_VALUE_FIELD_WALL_LENGTH_DELTA = 1.2

export function environmentWithBeliefValueField(
  environment: Environment,
  belief: Pick<BeliefState, 'estimatedWalls' | 'estimatedFeatures'>,
  options: GridValueFieldOptions = spiralValueFieldOptions,
): Environment {
  if (!environment.valueField) return environment
  const observedWalls =
    belief.estimatedFeatures && belief.estimatedFeatures.length > 0
      ? wallSegmentsFromEstimatedFeatures(belief.estimatedFeatures)
      : observedWallSegments(environment, belief.estimatedWalls)
  const sourceKey = valueFieldSourceKey(observedWalls, options)
  const sourceWallLength = totalWallLength(observedWalls)
  if (environment.valueField.sourceKey === sourceKey) return environment
  if (
    environment.valueField.sourceWallLength !== undefined &&
    sourceWallLength < environment.valueField.sourceWallLength * 0.98
  ) {
    return environment
  }
  if (
    environment.valueField.sourceWallLength !== undefined &&
    Math.abs(sourceWallLength - environment.valueField.sourceWallLength) < MIN_VALUE_FIELD_WALL_LENGTH_DELTA
  ) {
    return environment
  }
  const fieldEnvironment = { ...environment, walls: observedWalls }
  return {
    ...environment,
    valueField: { ...createGridValueField(fieldEnvironment, options), sourceKey, sourceWallLength },
  }
}

export function wallSegmentsFromEstimatedFeatures(features: EstimatedLineFeature[]): WallSegment[] {
  return features
    .filter((feature) => feature.confidence > 0)
    .map((feature) => ({
      id: feature.id,
      a: feature.a,
      b: feature.b,
    }))
}

export function observedWallSegments(environment: Environment, estimatedWalls: EstimatedWallSegment[]): WallSegment[] {
  const trueWalls = new Map(environment.walls.map((wall) => [wall.id, wall]))
  return estimatedWalls.flatMap((estimated): WallSegment[] => {
    if (estimated.tMax <= estimated.tMin || estimated.confidence <= 0) return []
    const wall = trueWalls.get(estimated.wallId)
    if (!wall) return []
    const tMin = clamp01(estimated.tMin)
    const tMax = clamp01(estimated.tMax)
    if (tMax <= tMin) return []
    return [
      {
        id: `${wall.id}:observed:${tMin.toFixed(3)}-${tMax.toFixed(3)}`,
        a: segmentPoint(wall, tMin),
        b: segmentPoint(wall, tMax),
      },
    ]
  })
}

function valueFieldSourceKey(walls: WallSegment[], options: GridValueFieldOptions) {
  const wallKey = walls
    .map((wall) => `${wall.id}:${rounded(wall.a.x)},${rounded(wall.a.y)}-${rounded(wall.b.x)},${rounded(wall.b.y)}`)
    .sort()
    .join('|')
  return `r=${options.resolution};rr=${options.robotRadius};p=${options.padding ?? ''};walls=${wallKey}`
}

function totalWallLength(walls: WallSegment[]) {
  return walls.reduce((total, wall) => total + Math.hypot(wall.b.x - wall.a.x, wall.b.y - wall.a.y), 0)
}

function rounded(value: number) {
  return Math.round(value * 1000) / 1000
}

function segmentPoint(wall: WallSegment, t: number) {
  return {
    x: wall.a.x + (wall.b.x - wall.a.x) * t,
    y: wall.a.y + (wall.b.y - wall.a.y) * t,
  }
}

function clamp01(value: number) {
  return Math.max(0, Math.min(1, value))
}
