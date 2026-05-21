import { clamp, distance, nearestPointOnSegment } from '../simulation/math'
import type { EstimatedLineFeature, WallObservation, WallSegment } from '../simulation/types'
import { associateWallObservation } from './wallAssociation'

const DEFAULT_FEATURE_LENGTH = 0.6
const ASSOCIATION_AFFINITY_THRESHOLD = 0.18
const ASSOCIATION_DISTANCE_THRESHOLD = 0.45

const clamp01 = (value: number) => clamp(value, 0, 1)

export function createEstimatedLineFeature(observation: WallObservation, time: number, id = 'feature-1') {
  const segment = segmentFromObservation(observation)
  return {
    id,
    ...segment,
    confidence: clamp01(observation.confidence),
    lastObservedAt: time,
    observationCount: 1,
  }
}

export function updateEstimatedLineFeature(
  feature: EstimatedLineFeature,
  observation: WallObservation,
  time: number,
): EstimatedLineFeature {
  const observed = segmentFromObservation(observation)
  const observationWeight = 1 / (feature.observationCount + 1)
  const priorWeight = 1 - observationWeight
  return {
    id: feature.id,
    a: blend(feature.a, observed.a, priorWeight, observationWeight),
    b: blend(feature.b, observed.b, priorWeight, observationWeight),
    confidence: clamp01(Math.max(feature.confidence * 0.995, feature.confidence + observation.strength * 0.12)),
    lastObservedAt: time,
    observationCount: feature.observationCount + 1,
  }
}

export function updateEstimatedLineFeatures(args: {
  existing: EstimatedLineFeature[]
  observations: WallObservation[]
  time: number
}): EstimatedLineFeature[] {
  const features = args.existing.map((feature) => ({ ...feature, confidence: feature.confidence * 0.998 }))
  for (const observation of args.observations) {
    const matchIndex = matchingFeatureIndex(features, observation)
    if (matchIndex >= 0) {
      features[matchIndex] = updateEstimatedLineFeature(features[matchIndex], observation, args.time)
      continue
    }
    features.push(createEstimatedLineFeature(observation, args.time, nextFeatureId(features)))
  }
  return features
}

function matchingFeatureIndex(features: EstimatedLineFeature[], observation: WallObservation) {
  if (features.length === 0) return -1
  const candidates = features.map(featureToWallSegment)
  const association = associateWallObservation(observation, candidates)
  if (!association || association.affinity < ASSOCIATION_AFFINITY_THRESHOLD) return -1
  const index = features.findIndex((feature) => feature.id === association.wallId)
  if (index < 0) return -1
  const pointDistance = distance(nearestPointOnSegment(observation.rayTarget, candidates[index]), observation.rayTarget)
  return pointDistance <= ASSOCIATION_DISTANCE_THRESHOLD ? index : -1
}

function nextFeatureId(features: EstimatedLineFeature[]) {
  const used = new Set(features.map((feature) => feature.id))
  for (let index = features.length + 1; ; index += 1) {
    const id = `feature-${index}`
    if (!used.has(id)) return id
  }
}

function featureToWallSegment(feature: EstimatedLineFeature): WallSegment {
  return { id: feature.id, a: feature.a, b: feature.b }
}

function segmentFromObservation(observation: WallObservation): Pick<EstimatedLineFeature, 'a' | 'b'> {
  const rayAngle = observation.sensorPose.theta + observation.bearing
  const tangent = { x: -Math.sin(rayAngle), y: Math.cos(rayAngle) }
  const observedSpan = Math.max(0, observation.tMax - observation.tMin)
  const length = Math.max(DEFAULT_FEATURE_LENGTH, observation.range * observedSpan)
  const half = length / 2
  return {
    a: {
      x: observation.rayTarget.x - tangent.x * half,
      y: observation.rayTarget.y - tangent.y * half,
    },
    b: {
      x: observation.rayTarget.x + tangent.x * half,
      y: observation.rayTarget.y + tangent.y * half,
    },
  }
}

function blend(
  previous: { x: number; y: number },
  observed: { x: number; y: number },
  priorWeight: number,
  observationWeight: number,
) {
  return {
    x: previous.x * priorWeight + observed.x * observationWeight,
    y: previous.y * priorWeight + observed.y * observationWeight,
  }
}
