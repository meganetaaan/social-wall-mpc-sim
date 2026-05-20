import { describe, expect, it } from 'vitest'
import type { PoseGaussian, RobotState } from '../simulation/types'
import {
  correctPoseGaussianWithObservation,
  poseGaussianFromSigmas,
  propagatePoseGaussian,
  tracePoseCovariance,
} from './poseBelief'

const mean: RobotState = { x: 1, y: 2, theta: 0 }

describe('pose belief helpers', () => {
  it('traces the pose covariance diagonal', () => {
    const pose: PoseGaussian = {
      mean,
      covariance: [
        [0.1, 0, 0],
        [0, 0.2, 0],
        [0, 0, 0.03],
      ],
    }

    expect(tracePoseCovariance(pose)).toBeCloseTo(0.33)
  })

  it('propagates the mean and increases covariance with motion', () => {
    const pose = poseGaussianFromSigmas(mean, 0.1, 0.1, 0.03)

    const propagated = propagatePoseGaussian(pose, { v: 0.8, omega: 0.5 }, { dt: 0.25 })

    expect(propagated.mean.x).toBeGreaterThan(pose.mean.x)
    expect(propagated.mean.y).toBeCloseTo(pose.mean.y)
    expect(propagated.mean.theta).toBeGreaterThan(pose.mean.theta)
    expect(propagated.covariance[0][0]).toBeGreaterThan(pose.covariance[0][0])
    expect(propagated.covariance[1][1]).toBeGreaterThan(pose.covariance[1][1])
    expect(propagated.covariance[2][2]).toBeGreaterThan(pose.covariance[2][2])
  })

  it('reduces covariance more for stronger observations and respects lower bounds', () => {
    const pose = poseGaussianFromSigmas(mean, 0.1, 0.1, 0.03)

    const weak = correctPoseGaussianWithObservation(pose, 0.2)
    const strong = correctPoseGaussianWithObservation(pose, 1)
    const bounded = correctPoseGaussianWithObservation(poseGaussianFromSigmas(mean, 0.021, 0.021, 0.011), 1)

    expect(strong.covariance[0][0]).toBeLessThan(weak.covariance[0][0])
    expect(strong.covariance[1][1]).toBeLessThan(weak.covariance[1][1])
    expect(strong.covariance[2][2]).toBeLessThan(weak.covariance[2][2])
    expect(bounded.covariance[0][0]).toBe(0.02)
    expect(bounded.covariance[1][1]).toBe(0.02)
    expect(bounded.covariance[2][2]).toBe(0.01)
  })
})
