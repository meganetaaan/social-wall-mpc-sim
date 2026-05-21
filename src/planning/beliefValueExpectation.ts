import type { BeliefState, Environment, PoseGaussian, RobotState, Vec2 } from '../simulation/types'
import { lookupValueField } from './valueField'

export function expectedValueFieldCost(environment: Environment, belief: BeliefState): number | null {
  const field = environment.valueField
  if (!field || !belief.pose || !isUsablePoseMean(belief.pose.mean)) return null

  const sigmaX = sigmaFromVariance(belief.pose.covariance?.[0]?.[0])
  const sigmaY = sigmaFromVariance(belief.pose.covariance?.[1]?.[1])
  const meanCost = lookupValueField(field, belief.pose.mean)

  if (sigmaX === null || sigmaY === null) return meanCost

  const samples = sigmaPoints(belief.pose, sigmaX, sigmaY)
  return samples.reduce((sum, sample) => sum + lookupValueField(field, sample), 0) / samples.length
}

function sigmaPoints(pose: PoseGaussian, sigmaX: number, sigmaY: number): Vec2[] {
  const { x, y } = pose.mean
  return [
    { x, y },
    { x: x + sigmaX * 0.5, y },
    { x: x - sigmaX * 0.5, y },
    { x, y: y + sigmaY * 0.5 },
    { x, y: y - sigmaY * 0.5 },
    { x: x + sigmaX, y },
    { x: x - sigmaX, y },
    { x, y: y + sigmaY },
    { x, y: y - sigmaY },
    { x: x + sigmaX, y: y + sigmaY },
    { x: x + sigmaX, y: y - sigmaY },
    { x: x - sigmaX, y: y + sigmaY },
    { x: x - sigmaX, y: y - sigmaY },
  ]
}

function sigmaFromVariance(value: number | undefined): number | null {
  if (value === undefined || Number.isNaN(value)) return null
  return Math.sqrt(Math.max(0, value))
}

function isUsablePoseMean(mean: RobotState | undefined) {
  return !!mean && Number.isFinite(mean.x) && Number.isFinite(mean.y)
}
