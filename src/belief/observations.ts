import type { PointObservation, RobotState, Vec2 } from '../simulation/types'

export type PlannerObservationSource = 'point-sensor'
export type PlannerObservationType = 'point-return'

export type PlannerPointObservation = {
  id: string
  observationType: PlannerObservationType
  point: Vec2
  range: number
  bearing: number
  sensorPose: RobotState
  time: number
  source: PlannerObservationSource
}

export type PlannerObservation = PlannerPointObservation

export function plannerFacingObservationKeys(): Array<keyof PlannerObservation> {
  return ['id', 'observationType', 'point', 'range', 'bearing', 'sensorPose', 'time', 'source']
}

export function toPlannerPointObservation(observation: PointObservation): PlannerPointObservation {
  return {
    id: observation.id,
    observationType: 'point-return',
    point: { ...observation.point },
    range: observation.range,
    bearing: observation.bearing,
    sensorPose: { ...observation.sensorPose },
    time: observation.time,
    source: observation.source,
  }
}

export function toPlannerPointObservations(observations: PointObservation[]): PlannerPointObservation[] {
  return observations.map(toPlannerPointObservation)
}
