import { describe, expect, it } from 'vitest'
import { createDefaultSimulationState, defaultParameters } from '../simulation/environment'
import { rolloutCandidateWithFutureObservations } from './beliefRollout'

describe('sampled future-observation belief rollouts', () => {
  it('evaluates a bounded deterministic set of future observation samples and averages their costs', () => {
    const state = createDefaultSimulationState()
    const controls = Array.from({ length: 3 }, () => ({ v: 0.3, omega: 0.1 }))

    const first = rolloutCandidateWithFutureObservations({
      robot: state.robot,
      humans: state.humans,
      environment: state.environment,
      belief: state.belief,
      controls,
      previousControl: state.previousControl,
      parameters: { ...defaultParameters, horizonSteps: 3 },
      observationSamples: 3,
      seed: 123,
    })
    const second = rolloutCandidateWithFutureObservations({
      robot: state.robot,
      humans: state.humans,
      environment: state.environment,
      belief: state.belief,
      controls,
      previousControl: state.previousControl,
      parameters: { ...defaultParameters, horizonSteps: 3 },
      observationSamples: 3,
      seed: 123,
    })

    expect(first.samples).toHaveLength(3)
    expect(first.samples.map((sample) => sample.seed)).toEqual([123, 124, 125])
    const average = first.samples.reduce((sum, sample) => sum + sample.rollout.cost.total, 0) / first.samples.length
    expect(first.rollout.cost.total).toBeCloseTo(average)
    expect(first.rollout.cost.total).toBeCloseTo(second.rollout.cost.total)
    expect(first.samples.map((sample) => sample.rollout.cost.total)).toEqual(
      second.samples.map((sample) => sample.rollout.cost.total),
    )
  })
})
