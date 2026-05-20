import { stepRobot } from '../simulation/dynamics'
import type { ControlInput, PlannerParameters, PoseCovariance, PoseGaussian, RobotState } from '../simulation/types'

const minSigmaX = 0.02
const minSigmaY = 0.02
const minSigmaTheta = 0.01

const clamp01 = (value: number) => Math.min(1, Math.max(0, value))

export function diagonalPoseCovariance(sigmaX: number, sigmaY: number, sigmaTheta: number): PoseCovariance {
  return [
    [Math.max(minSigmaX, sigmaX), 0, 0],
    [0, Math.max(minSigmaY, sigmaY), 0],
    [0, 0, Math.max(minSigmaTheta, sigmaTheta)],
  ]
}

export function poseGaussianFromSigmas(
  mean: RobotState,
  sigmaX: number,
  sigmaY: number,
  sigmaTheta: number,
): PoseGaussian {
  return {
    mean: { ...mean },
    covariance: diagonalPoseCovariance(sigmaX, sigmaY, sigmaTheta),
  }
}

export function sigmasFromPoseGaussian(pose: PoseGaussian) {
  return {
    sigmaX: Math.max(minSigmaX, pose.covariance[0][0]),
    sigmaY: Math.max(minSigmaY, pose.covariance[1][1]),
    sigmaTheta: Math.max(minSigmaTheta, pose.covariance[2][2]),
  }
}

export function tracePoseCovariance(pose: PoseGaussian) {
  return pose.covariance[0][0] + pose.covariance[1][1] + pose.covariance[2][2]
}

export function propagatePoseGaussian(
  pose: PoseGaussian,
  control: ControlInput,
  parameters: Pick<PlannerParameters, 'dt'>,
): PoseGaussian {
  const dt = Math.abs(parameters.dt)
  const linearGrowth = 0.006 + dt * (0.004 + Math.abs(control.v) * 0.01)
  const angularGrowth = 0.003 + dt * (0.002 + Math.abs(control.omega) * 0.006)
  return {
    mean: stepRobot(pose.mean, control, parameters.dt),
    covariance: diagonalPoseCovariance(
      pose.covariance[0][0] + linearGrowth,
      pose.covariance[1][1] + linearGrowth,
      pose.covariance[2][2] + angularGrowth,
    ),
  }
}

export function correctPoseGaussianWithObservation(pose: PoseGaussian, observationStrength: number): PoseGaussian {
  const strength = clamp01(observationStrength)
  const linearCorrection = 0.02 * strength
  const angularCorrection = 0.007 * strength
  return {
    mean: { ...pose.mean },
    covariance: diagonalPoseCovariance(
      pose.covariance[0][0] - linearCorrection,
      pose.covariance[1][1] - linearCorrection,
      pose.covariance[2][2] - angularCorrection,
    ),
  }
}
