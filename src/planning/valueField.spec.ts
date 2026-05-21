import { describe, expect, it } from 'vitest'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import type { Environment, RobotState } from '../simulation/types'
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
})
