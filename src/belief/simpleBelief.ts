import { nearestWall } from '../simulation/math'
import type { BeliefState, Environment, PlannerParameters, RobotState } from '../simulation/types'

export function updateBelief(
  belief: BeliefState,
  robot: RobotState,
  environment: Environment,
  parameters: PlannerParameters,
): BeliefState {
  const nearest = nearestWall({ x: robot.x, y: robot.y }, environment.walls)
  const observationStrength = Math.max(0, 1 - nearest.distance / 2.2)
  const motionGrowth = 0.006 + Math.abs(parameters.dt) * 0.004
  const correction = 0.02 * observationStrength
  return {
    sigmaX: Math.max(0.02, belief.sigmaX + motionGrowth - correction),
    sigmaY: Math.max(0.02, belief.sigmaY + motionGrowth - correction),
    sigmaTheta: Math.max(0.01, belief.sigmaTheta + motionGrowth * 0.5 - correction * 0.35),
    mapConfidence: Math.min(1, Math.max(0.05, belief.mapConfidence + 0.01 * observationStrength - 0.002)),
  }
}

export const traceSigma = (belief: BeliefState) => belief.sigmaX + belief.sigmaY + belief.sigmaTheta
