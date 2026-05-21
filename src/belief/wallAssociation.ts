import { distance, nearestPointOnSegment, normAngle } from '../simulation/math'
import type { RobotState, Vec2, WallObservation, WallSegment } from '../simulation/types'
import {
  expectedRangeBearingToWall,
  type ObservationLikelihood,
  wallObservationAffinity,
  wallObservationLikelihood,
} from './wallMapBelief'

export type WallAssociationObservation = {
  point?: Vec2
  range: number
  bearing: number
  rangeStdDev?: number
  bearingStdDev?: number
  sensorPose: RobotState
}

export type WallAssociationCandidate = {
  wallId: string
  score: number
  distanceResidual: number
  rangeResidual: number
  bearingResidual: number
  likelihood: ObservationLikelihood
  affinity: number
}

export type WallAssociation = WallAssociationCandidate & {
  candidates: WallAssociationCandidate[]
}

export function associateWallObservation(
  observation: WallAssociationObservation,
  candidateWalls: WallSegment[],
): WallAssociation | null {
  if (candidateWalls.length === 0) return null

  const rangeStdDev = observation.rangeStdDev ?? 0.1
  const bearingStdDev = observation.bearingStdDev ?? 0.05
  const pointSigma = Math.max(0.15, rangeStdDev * 2)
  const scoringObservation = {
    range: observation.range,
    bearing: observation.bearing,
    rangeStdDev,
    bearingStdDev,
  }

  const candidates = candidateWalls.map((wall) => {
    const expected = expectedRangeBearingToWall(observation.sensorPose, wall)
    const likelihood = wallObservationLikelihood(scoringObservation, observation.sensorPose, wall)
    const distanceResidual = observation.point
      ? distance(observation.point, nearestPointOnSegment(observation.point, wall))
      : 0
    const pointPenalty = observation.point ? (distanceResidual / pointSigma) ** 2 : 0
    const likelihoodAffinity = wallObservationAffinity(scoringObservation, observation.sensorPose, wall)
    const score = likelihoodAffinity * Math.exp(-0.5 * pointPenalty)

    return {
      wallId: wall.id,
      score,
      distanceResidual,
      rangeResidual: observation.range - expected.range,
      bearingResidual: normAngle(observation.bearing - expected.bearing),
      likelihood,
      affinity: score,
    }
  })

  const [best] = [...candidates].sort((a, b) => b.score - a.score)
  return { ...best, candidates }
}

export function wallAssociationAccuracy(
  observations: Pick<
    WallObservation,
    'wallId' | 'range' | 'bearing' | 'rangeStdDev' | 'bearingStdDev' | 'sensorPose' | 'rayTarget'
  >[],
  candidateWalls: WallSegment[],
) {
  if (observations.length === 0) return 1
  const correct = observations.filter((observation) => {
    const association = associateWallObservation(
      {
        point: observation.rayTarget,
        range: observation.range,
        bearing: observation.bearing,
        rangeStdDev: observation.rangeStdDev,
        bearingStdDev: observation.bearingStdDev,
        sensorPose: observation.sensorPose,
      },
      candidateWalls,
    )
    return association?.wallId === observation.wallId
  }).length
  return correct / observations.length
}
