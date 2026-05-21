import type {
  BeliefState,
  Environment,
  EstimatedLineFeature,
  EstimatedWallSegment,
  WallSegment,
} from '../simulation/types'
import { createGridValueField, type GridValueFieldOptions } from './valueField'

export const spiralValueFieldOptions: GridValueFieldOptions = { resolution: 0.2, robotRadius: 0.22 }

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
  const fieldEnvironment = { ...environment, walls: observedWalls }
  return { ...environment, valueField: createGridValueField(fieldEnvironment, options) }
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

function segmentPoint(wall: WallSegment, t: number) {
  return {
    x: wall.a.x + (wall.b.x - wall.a.x) * t,
    y: wall.a.y + (wall.b.y - wall.a.y) * t,
  }
}

function clamp01(value: number) {
  return Math.max(0, Math.min(1, value))
}
