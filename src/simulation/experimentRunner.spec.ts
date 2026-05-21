import { describe, expect, it } from 'vitest'
import { runScenarioBatch, runScenarioExperiment } from './experimentRunner'

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

  it('reaches the crossing-human goal with belief-mpc within a bounded fast replay', () => {
    const result = runScenarioExperiment({
      scenarioId: 'crossing-human',
      plannerMode: 'belief-mpc',
      parameters: fastParams,
      maxSteps: 120,
    })
    expect(result.reachedGoal).toBe(true)
    expect(result.timeToGoal).not.toBeNull()
    expect(result.steps).toBeLessThanOrEqual(120)
    expect(result.finalGoalDistance).toBeLessThanOrEqual(0.21)
    expect(result.bestGoalDistance).toBeLessThanOrEqual(0.21)
  })

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

  it('runs a deterministic ordered batch for scenario and policy combinations', () => {
    const args = {
      scenarioIds: ['crossing-human', 'standing-human'] as const,
      plannerModes: ['belief-mpc', 'wall-only', 'reactive-stop'] as const,
      parameters: { sampleCount: 8, horizonSteps: 5 },
      maxSteps: 80,
    }

    const first = runScenarioBatch(args)
    const second = runScenarioBatch(args)

    expect(first.maxSteps).toBe(args.maxSteps)
    expect(first.summaries).toHaveLength(args.scenarioIds.length * args.plannerModes.length)
    expect(first).toEqual(second)
    expect(first.summaries.map((summary) => [summary.scenarioId, summary.plannerMode])).toEqual([
      ['crossing-human', 'belief-mpc'],
      ['crossing-human', 'wall-only'],
      ['crossing-human', 'reactive-stop'],
      ['standing-human', 'belief-mpc'],
      ['standing-human', 'wall-only'],
      ['standing-human', 'reactive-stop'],
    ])
  })

  it('covers new POMDP/SLAM gap scenarios in deterministic headless replay', () => {
    const result = runScenarioBatch({
      scenarioIds: ['ambiguous-parallel-corridor', 'occluded-corner-human', 'kidnapped-pose-bend'] as const,
      plannerModes: ['belief-mpc'] as const,
      parameters: { sampleCount: 8, horizonSteps: 5 },
      maxSteps: 30,
    })

    expect(result.summaries.map((summary) => summary.scenarioId)).toEqual([
      'ambiguous-parallel-corridor',
      'occluded-corner-human',
      'kidnapped-pose-bend',
    ])
    expect(
      result.summaries.every((summary) =>
        [
          summary.finalGoalDistance,
          summary.bestGoalDistance,
          summary.wallAssociationAccuracy,
          summary.poseNormalizedError,
          summary.mapKnowledgeError,
        ].every(Number.isFinite),
      ),
    ).toBe(true)
  })

  it('summarizes batch rows with readable labels and finite numeric metrics', () => {
    const result = runScenarioBatch({
      scenarioIds: ['crossing-human'],
      plannerModes: ['belief-mpc'],
      parameters: { sampleCount: 8, horizonSteps: 5 },
      maxSteps: 40,
    })
    const [summary] = result.summaries

    expect(summary.scenarioName).toBe('Crossing human')
    expect(summary.plannerMode).toBe('belief-mpc')

    const numericValues = [
      summary.finalGoalDistance,
      summary.bestGoalDistance,
      summary.steps,
      summary.elapsedTime,
      summary.minHumanDistance,
      summary.socialViolationCount,
      summary.nearCollisionCount,
      summary.stopDuration,
      summary.meanWallDistanceError,
      summary.maxWallDistanceError,
      summary.uncertaintyTrace,
      summary.estimatedMapCoverage,
      summary.posePositionError,
      summary.poseHeadingError,
      summary.poseNormalizedError,
      summary.mapKnowledgeError,
      summary.wallAssociationAccuracy,
    ]

    expect(numericValues.every((value) => Number.isFinite(value))).toBe(true)
    expect(summary.mapKnowledgeError).toBeGreaterThanOrEqual(0)
    expect(summary.mapKnowledgeError).toBeLessThanOrEqual(1)
    expect(summary.wallAssociationAccuracy).toBeGreaterThanOrEqual(0)
    expect(summary.wallAssociationAccuracy).toBeLessThanOrEqual(1)
    if (summary.timeToGoal !== null) {
      expect(Number.isFinite(summary.timeToGoal)).toBe(true)
    }
  })

  it('keeps belief-mpc meaningfully comparable to a baseline in a focused batch', () => {
    const result = runScenarioBatch({
      scenarioIds: ['crossing-human'],
      plannerModes: ['belief-mpc', 'wall-only'],
      parameters: { sampleCount: 8, horizonSteps: 5 },
      maxSteps: 120,
    })
    const beliefMpc = result.summaries.find((summary) => summary.plannerMode === 'belief-mpc')
    const wallOnly = result.summaries.find((summary) => summary.plannerMode === 'wall-only')

    expect(beliefMpc).toBeDefined()
    expect(wallOnly).toBeDefined()
    if (!beliefMpc || !wallOnly) throw new Error('Missing focused comparison rows')
    expect(
      beliefMpc.reachedGoal ||
        beliefMpc.socialViolationCount <= wallOnly.socialViolationCount ||
        beliefMpc.bestGoalDistance < wallOnly.bestGoalDistance,
    ).toBe(true)
  })
})
