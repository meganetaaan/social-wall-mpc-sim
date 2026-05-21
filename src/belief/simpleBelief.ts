import type {
  BeliefState,
  Environment,
  MapUpdateMode,
  PlannerParameters,
  PointObservation,
  RobotState,
} from '../simulation/types'
import { inferControl, transitionBelief } from './beliefTransition'
import { tracePoseCovariance } from './poseBelief'
import { computeMapUncertainty, estimatedWallConfidence } from './wallMapBelief'

export function updateBelief(
  belief: BeliefState,
  robot: RobotState,
  environment: Environment,
  parameters: PlannerParameters,
  time = 0,
  pointObservations: PointObservation[] = [],
  options: { mapUpdateMode?: MapUpdateMode } = {},
): BeliefState {
  return transitionBelief({
    belief,
    control: inferControl(belief.pose.mean, robot, parameters.dt),
    observedRobot: robot,
    environment,
    parameters,
    time,
    pointObservations,
    mapUpdateMode: options.mapUpdateMode,
  })
}

export const traceSigma = (belief: BeliefState) =>
  belief.pose ? tracePoseCovariance(belief.pose) : belief.sigmaX + belief.sigmaY + belief.sigmaTheta

export const mapUncertaintyTrace = (belief: BeliefState) => {
  if (belief.estimatedWalls.length > 0) return computeMapUncertainty(belief.estimatedWalls)
  if (belief.wallBeliefs.length === 0) return 1 - belief.mapConfidence
  return belief.wallBeliefs.reduce((sum, wall) => sum + (1 - wall.confidence), 0) / belief.wallBeliefs.length
}

export const wallConfidence = (belief: BeliefState, wallId: string) => estimatedWallConfidence(belief, wallId)
