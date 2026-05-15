import { describe, expect, it } from 'vitest'
import { createSimulationStateForScenario, scenarioDefinitions } from './scenarios'

describe('scenario definitions', () => {
  it('creates deterministic state for the same scenario id', () => {
    const first = createSimulationStateForScenario('crossing-human')
    const second = createSimulationStateForScenario('crossing-human')

    expect(first).toEqual(second)
    expect(first).not.toBe(second)
    expect(first.environment).not.toBe(second.environment)
  })

  it('includes all required comparison scenarios with walls and humans', () => {
    expect(scenarioDefinitions.map((scenario) => scenario.id)).toEqual([
      'crossing-human',
      'standing-human',
      'head-on-human',
      'blocked-corridor',
      'partial-map-bend',
      'multi-human',
    ])

    for (const scenario of scenarioDefinitions) {
      const state = createSimulationStateForScenario(scenario.id)
      expect(state.environment.walls.length).toBeGreaterThan(0)
      expect(state.humans.length).toBeGreaterThan(0)
      expect(state.belief.estimatedWalls.map((wall) => wall.wallId)).toEqual(
        state.environment.walls.map((wall) => wall.id),
      )
    }
  })

  it('keeps the multi-human scenario useful for competing social costs', () => {
    const state = createSimulationStateForScenario('multi-human')

    expect(state.humans.length).toBeGreaterThanOrEqual(2)
  })
})
