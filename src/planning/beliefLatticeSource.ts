import type { BeliefState, Environment, WallSegment } from '../simulation/types'
import { observedWallSegments, wallSegmentsFromEstimatedFeatures } from './beliefValueField'

export type BeliefLatticeSource =
  | {
      status: 'ready'
      source: 'known-map' | 'anonymous-features' | 'estimated-walls'
      environment: Environment
      reliability: LatticeSourceReliability
    }
  | {
      status: 'fallback'
      source: 'anonymous-features' | 'estimated-walls'
      reliability: LatticeSourceReliability
      reason: string
    }

export type LatticeSourceReliability = {
  wallCount: number
  totalLength: number
  meanConfidence: number
  meanObservationCount: number
  knownCoverageRatio: number
}

export type BeliefLatticeSourceOptions = {
  minFeatureConfidence?: number
  minFeatureObservationCount?: number
  minFeatureTotalLength?: number
  minEstimatedWallConfidence?: number
  minEstimatedWallTotalLength?: number
  knownMapCoverageRatio?: number
  knownMapMeanConfidence?: number
}

const defaultOptions = {
  minFeatureConfidence: 0.55,
  minFeatureObservationCount: 2,
  minFeatureTotalLength: 3.0,
  minEstimatedWallConfidence: 0.6,
  minEstimatedWallTotalLength: 3.0,
  knownMapCoverageRatio: 0.72,
  knownMapMeanConfidence: 0.65,
}

export function selectBeliefLatticeSource(
  environment: Environment,
  belief: Pick<BeliefState, 'estimatedWalls' | 'estimatedFeatures'>,
  options: BeliefLatticeSourceOptions = {},
): BeliefLatticeSource {
  const thresholds = { ...defaultOptions, ...options }
  if (belief.estimatedFeatures && belief.estimatedFeatures.length > 0) {
    const supportedFeatures = belief.estimatedFeatures.filter(
      (feature) =>
        feature.confidence >= thresholds.minFeatureConfidence &&
        feature.observationCount >= thresholds.minFeatureObservationCount,
    )
    const walls = wallSegmentsFromEstimatedFeatures(supportedFeatures)
    const reliability = reliabilityForWalls(
      walls,
      supportedFeatures.map((feature) => feature.confidence),
      supportedFeatures.map((feature) => feature.observationCount),
      environment,
    )
    if (reliability.totalLength < thresholds.minFeatureTotalLength) {
      return {
        status: 'fallback',
        source: 'anonymous-features',
        reliability,
        reason: 'anonymous feature support is too sparse for hard lattice walls',
      }
    }
    return {
      status: 'ready',
      source: 'anonymous-features',
      environment: { ...environment, walls },
      reliability,
    }
  }

  const estimatedWallSegments = observedWallSegments(environment, belief.estimatedWalls)
  const estimatedReliability = reliabilityForWalls(
    estimatedWallSegments,
    belief.estimatedWalls
      .filter((estimated) => estimated.confidence > 0 && estimated.tMax > estimated.tMin)
      .map((estimated) => estimated.confidence),
    belief.estimatedWalls
      .filter((estimated) => estimated.confidence > 0 && estimated.tMax > estimated.tMin)
      .map(() => 1),
    environment,
  )
  if (
    estimatedReliability.knownCoverageRatio >= thresholds.knownMapCoverageRatio &&
    estimatedReliability.meanConfidence >= thresholds.knownMapMeanConfidence
  ) {
    return {
      status: 'ready',
      source: 'known-map',
      environment,
      reliability: estimatedReliability,
    }
  }
  if (
    estimatedReliability.totalLength >= thresholds.minEstimatedWallTotalLength &&
    estimatedReliability.meanConfidence >= thresholds.minEstimatedWallConfidence
  ) {
    return {
      status: 'ready',
      source: 'estimated-walls',
      environment: { ...environment, walls: estimatedWallSegments },
      reliability: estimatedReliability,
    }
  }
  return {
    status: 'fallback',
    source: 'estimated-walls',
    reliability: estimatedReliability,
    reason: 'estimated wall support is too sparse for hard lattice walls',
  }
}

function reliabilityForWalls(
  walls: WallSegment[],
  confidences: number[],
  observationCounts: number[],
  environment: Environment,
): LatticeSourceReliability {
  return {
    wallCount: walls.length,
    totalLength: totalWallLength(walls),
    meanConfidence: mean(confidences),
    meanObservationCount: mean(observationCounts),
    knownCoverageRatio: totalWallLength(walls) / Math.max(1e-6, totalWallLength(environment.walls)),
  }
}

function totalWallLength(walls: WallSegment[]) {
  return walls.reduce((total, wall) => total + Math.hypot(wall.b.x - wall.a.x, wall.b.y - wall.a.y), 0)
}

function mean(values: number[]) {
  if (values.length === 0) return 0
  return values.reduce((sum, value) => sum + value, 0) / values.length
}
