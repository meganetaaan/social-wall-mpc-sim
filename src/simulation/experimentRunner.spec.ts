import { describe, expect, it } from 'vitest'
import { runScenarioExperiment } from './experimentRunner'

const fastParams = { sampleCount: 12, horizonSteps: 6 }

describe('headless experiment runner', () => {
  it('is deterministic for the same scenario, policy, parameters, and step bound', () => {
    const args = {
      scenarioId: 'crossing-human' as const,
      plannerMode: 'belief-mpc' as const,
      parameters: fastParams,
      maxSteps: 120,
    }

    expect(runScenarioExperiment(args)).toMatchObject(runScenarioExperiment(args))
  })

  it('reaches the default crossing-human goal with belief-mpc within the bounded run', () => {
    const result = runScenarioExperiment({
      scenarioId: 'crossing-human',
      plannerMode: 'belief-mpc',
      maxSteps: 700,
    })

    expect(result.reachedGoal).toBe(true)
    expect(result.timeToGoal).not.toBeNull()
    expect(result.steps).toBeLessThanOrEqual(700)
    expect(result.finalGoalDistance).toBeLessThanOrEqual(0.45)
  }, 10_000)

  it('keeps a meaningful baseline comparison against wall-only', () => {
    const beliefMpc = runScenarioExperiment({
      scenarioId: 'crossing-human',
      plannerMode: 'belief-mpc',
      parameters: fastParams,
      maxSteps: 120,
    })
    const wallOnly = runScenarioExperiment({
      scenarioId: 'crossing-human',
      plannerMode: 'wall-only',
      parameters: fastParams,
      maxSteps: 120,
    })

    expect(beliefMpc.reachedGoal || beliefMpc.bestGoalDistance < wallOnly.bestGoalDistance).toBe(true)
    expect(
      wallOnly.reachedGoal ||
        wallOnly.finalState.metrics.socialViolationCount >= beliefMpc.finalState.metrics.socialViolationCount,
    ).toBe(true)
  })
})
