import { describe, expect, it } from 'vitest'
import { poseGaussianFromSigmas } from '../belief/poseBelief'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import type { BeliefState, Environment, RobotState } from '../simulation/types'
import { environmentWithBeliefValueField, wallSegmentsFromEstimatedFeatures } from './beliefValueField'
import { createGridValueField, lookupValueField } from './valueField'

const simpleWallEnvironment: Environment = {
  goal: { x: 3, y: 0.5 },
  goalRadius: 0.2,
  walls: [
    { id: 'bottom', a: { x: 0, y: 0 }, b: { x: 4, y: 0 } },
    { id: 'top', a: { x: 0, y: 1 }, b: { x: 4, y: 1 } },
  ],
  obstacles: [],
}

describe('grid value field', () => {
  it('computes lower cost-to-go near the goal than near the start', () => {
    const field = createGridValueField(simpleWallEnvironment, { resolution: 0.25, robotRadius: 0.05 })

    expect(lookupValueField(field, { x: 2.75, y: 0.5 })).toBeLessThan(lookupValueField(field, { x: 0.75, y: 0.5 }))
  })

  it('keeps spiral cost-to-go lower on the real corridor detour than below the wall-separated goal', () => {
    const { environment } = createSimulationStateForScenario('spiral-known')
    const field = createGridValueField(environment, { resolution: 0.2, robotRadius: 0.12 })
    const belowGoalLocalMinimum: RobotState = { x: 4.2, y: 1.4, theta: 0 }
    const innerApproach: RobotState = { x: 5, y: 2.05, theta: Math.PI / 2 }

    expect(lookupValueField(field, innerApproach)).toBeLessThan(lookupValueField(field, belowGoalLocalMinimum))
  })

  it('computes unknown spiral cost-to-go from observed wall belief instead of true hidden walls', () => {
    const known = createSimulationStateForScenario('spiral-known')
    const unknown = createSimulationStateForScenario('spiral-unknown')
    const wallSeparatedPoint: RobotState = { x: 4.2, y: 1.4, theta: 0 }

    const unknownField = unknown.environment.valueField
    const knownField = known.environment.valueField
    expect(unknownField).toBeDefined()
    expect(knownField).toBeDefined()
    if (!unknownField || !knownField) throw new Error('spiral value fields should be defined')
    expect(lookupValueField(unknownField, wallSeparatedPoint)).toBeLessThan(
      lookupValueField(knownField, wallSeparatedPoint),
    )
  })

  it('converts estimated local line features into planner wall segments without true ids', () => {
    const segments = wallSegmentsFromEstimatedFeatures([
      {
        id: 'feature-1',
        a: { x: 0, y: 1 },
        b: { x: 3, y: 1 },
        confidence: 0.8,
        lastObservedAt: 4,
        observationCount: 1,
      },
    ])

    expect(segments).toEqual([{ id: 'feature-1', a: { x: 0, y: 1 }, b: { x: 3, y: 1 } }])
  })

  it('builds unknown-map value fields from local estimated features before true-id estimated walls', () => {
    const environment: Environment = {
      goal: { x: 2, y: 2 },
      goalRadius: 0.2,
      walls: [{ id: 'true-hidden-wall', a: { x: 0, y: 1 }, b: { x: 4, y: 1 } }],
      obstacles: [],
      valueField: createGridValueField(
        { goal: { x: 2, y: 2 }, walls: [], obstacles: [] },
        { resolution: 0.25, robotRadius: 0.05 },
      ),
    }
    const belief: BeliefState = {
      pose: poseGaussianFromSigmas({ x: 2, y: 0.5, theta: 0 }, 0.1, 0.1, 0.05),
      sigmaX: 0.1,
      sigmaY: 0.1,
      sigmaTheta: 0.05,
      mapConfidence: 0.1,
      wallBeliefs: [],
      estimatedWalls: [],
      estimatedFeatures: [
        {
          id: 'feature-1',
          a: { x: 0, y: 1 },
          b: { x: 4, y: 1 },
          confidence: 0.9,
          lastObservedAt: 1,
          observationCount: 1,
        },
      ],
    }

    const options = { resolution: 0.25, robotRadius: 0.05, padding: 2 }
    const withoutFeature = environmentWithBeliefValueField(environment, { ...belief, estimatedFeatures: [] }, options)
    const withFeature = environmentWithBeliefValueField(environment, belief, options)

    expect(wallSegmentsFromEstimatedFeatures(belief.estimatedFeatures ?? []).map((wall) => wall.id)).toEqual([
      'feature-1',
    ])
    if (!withFeature.valueField || !withoutFeature.valueField) throw new Error('value fields should be defined')
    expect(lookupValueField(withFeature.valueField, { x: 2, y: 0.5 })).toBeGreaterThan(
      lookupValueField(withoutFeature.valueField, { x: 2, y: 0.5 }),
    )
  })

  it('keeps known-map spiral value fields constrained by the full map instead of degrading to observed fragments', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const refreshed = environmentWithBeliefValueField(state.environment, state.belief)

    expect(refreshed.valueField).toBe(state.environment.valueField)
  })

  it('reuses a belief-derived value field when the estimated feature map has not changed', () => {
    const firstState = createSimulationStateForScenario('crossing-human')
    const first = environmentWithBeliefValueField(firstState.environment, firstState.belief)
    const second = environmentWithBeliefValueField(first, firstState.belief)

    expect(second.valueField).toBe(first.valueField)
  })
})
