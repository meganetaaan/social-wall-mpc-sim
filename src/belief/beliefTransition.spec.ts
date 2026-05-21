import { describe, expect, it } from 'vitest'
import { defaultParameters } from '../simulation/environment'
import type { BeliefState, ControlInput, Environment, PointObservation, RobotState } from '../simulation/types'
import { predictBelief, transitionBelief, updateBeliefWithObservation } from './beliefTransition'
import { poseGaussianFromSigmas } from './poseBelief'

const environment: Environment = {
  goal: { x: 5, y: 2 },
  walls: [
    { id: 'observed-wall', a: { x: 0, y: 0 }, b: { x: 5, y: 0 } },
    { id: 'far-wall', a: { x: 7, y: 0 }, b: { x: 7, y: 4 } },
  ],
  obstacles: [],
}

const initialRobot: RobotState = { x: 0.8, y: 0.45, theta: -Math.PI / 2 }
const observedRobot: RobotState = { x: 1, y: 0.45, theta: -Math.PI / 2 }
const control: ControlInput = { v: 0.5, omega: 0.1 }

const initialBelief: BeliefState = {
  pose: poseGaussianFromSigmas(initialRobot, 0.14, 0.14, 0.04),
  sigmaX: 0.14,
  sigmaY: 0.14,
  sigmaTheta: 0.04,
  mapConfidence: 0.45,
  wallBeliefs: [
    { wallId: 'observed-wall', confidence: 0.2, lastObservedAt: -1 },
    { wallId: 'far-wall', confidence: 0.7, lastObservedAt: -1 },
  ],
  estimatedWalls: [
    { wallId: 'observed-wall', tMin: 0, tMax: 0, confidence: 0, lastObservedAt: -1 },
    { wallId: 'far-wall', tMin: 0, tMax: 0.6, confidence: 0.7, lastObservedAt: -1 },
  ],
}

const pointObservation: PointObservation = {
  id: 'point-1',
  point: { x: 1.25, y: 0.55 },
  range: 0.25,
  bearing: 0.1,
  sensorPose: observedRobot,
  time: 12,
  source: 'point-sensor',
}

describe('explicit belief transition API', () => {
  it('predicts pose uncertainty from a control without applying map observations', () => {
    const predicted = predictBelief({
      belief: initialBelief,
      control,
      parameters: defaultParameters,
    })

    expect(predicted.pose.mean.y).toBeLessThan(initialBelief.pose.mean.y)
    expect(predicted.estimatedWalls).toEqual(initialBelief.estimatedWalls)
    expect(predicted.wallBeliefs).toEqual(initialBelief.wallBeliefs)
  })

  it('updates predicted belief from anonymous observations and point returns', () => {
    const predicted = predictBelief({ belief: initialBelief, control, parameters: defaultParameters })

    const updated = updateBeliefWithObservation({
      belief: predicted,
      robot: observedRobot,
      environment,
      parameters: defaultParameters,
      time: 12,
      pointObservations: [pointObservation],
      mapUpdateMode: 'anonymous-line-features',
    })

    expect(updated.estimatedWalls).toEqual(initialBelief.estimatedWalls)
    expect(updated.estimatedFeatures?.[0]?.id).toMatch(/^feature-/)
    expect(updated.objectBeliefs?.length).toBeGreaterThan(0)
  })

  it('combines prediction and observation update for one POMDP-style transition', () => {
    const transitioned = transitionBelief({
      belief: initialBelief,
      control,
      observedRobot,
      environment,
      parameters: defaultParameters,
      time: 12,
      pointObservations: [pointObservation],
      mapUpdateMode: 'anonymous-line-features',
    })

    const feature = transitioned.estimatedFeatures?.[0]
    expect(transitioned.pose.mean).toEqual(observedRobot)
    expect(feature?.id).toMatch(/^feature-/)
    expect(Object.keys(feature ?? {})).not.toContain('wallId')
  })
})
