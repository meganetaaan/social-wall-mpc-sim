import { describe, expect, it } from 'vitest'
import { defaultParameters } from './environment'
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
      'follow-behind-human',
      'overtaking-human',
      'yielding-blocker',
      'ambiguous-parallel-corridor',
      'spiral-known',
      'spiral-unknown',
    ])

    for (const scenario of scenarioDefinitions) {
      const state = createSimulationStateForScenario(scenario.id)
      expect(state.environment.walls.length).toBeGreaterThan(0)
      if (scenario.id !== 'spiral-known' && scenario.id !== 'spiral-unknown')
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

  it('adds requested social-motion scenarios with explicit behavior metadata', () => {
    expect(
      createSimulationStateForScenario('follow-behind-human').humans.some(
        (human) => human.motion?.kind === 'follow-robot',
      ),
    ).toBe(true)
    expect(
      createSimulationStateForScenario('overtaking-human').humans.some((human) => human.motion?.kind === 'scripted'),
    ).toBe(true)
    expect(
      createSimulationStateForScenario('yielding-blocker').humans.some((human) => human.motion?.kind === 'yield-after'),
    ).toBe(true)
  })

  it('adds known and unknown spiral corridors with central goals and no humans', () => {
    const known = createSimulationStateForScenario('spiral-known')
    const unknown = createSimulationStateForScenario('spiral-unknown')

    expect(known.environment.walls.length).toBeGreaterThanOrEqual(12)
    expect(unknown.environment.walls.length).toBe(known.environment.walls.length)
    expect(known.environment.goal.x).toBeCloseTo(5, 1)
    expect(known.environment.goal.y).toBeCloseTo(2.95, 1)
    expect(known.environment.valueField).toBeDefined()
    expect(unknown.environment.valueField?.values.length).toBe(known.environment.valueField?.values.length)
    expect(known.humans).toEqual([])
    expect(unknown.humans).toEqual([])
    expect(known.belief.estimatedWalls.filter((wall) => wall.confidence > 0.6).length).toBeGreaterThan(
      known.environment.walls.length / 2,
    )
    expect(unknown.belief.estimatedWalls.filter((wall) => wall.confidence > 0.1).length).toBeLessThan(3)
  })

  it('keeps spiral corridor lanes wider than the wall collision band on both sides', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const wallById = new Map(state.environment.walls.map((wall) => [wall.id, wall]))
    const verticalLaneGaps = [
      ['spiral-outer-bottom', 'spiral-lane-bottom'],
      ['spiral-lane-bottom', 'spiral-inner-bottom'],
      ['spiral-inner-bottom', 'spiral-center-bottom'],
      ['spiral-inner-top', 'spiral-lane-top'],
      ['spiral-lane-top', 'spiral-outer-top'],
    ] as const
    const minimumLaneWidth = defaultParameters.wallCollisionDistance * 2 + 0.08

    for (const [lowerId, upperId] of verticalLaneGaps) {
      const lower = wallById.get(lowerId)
      const upper = wallById.get(upperId)
      expect(lower, lowerId).toBeDefined()
      expect(upper, upperId).toBeDefined()
      const laneWidth = Math.abs((upper?.a.y ?? 0) - (lower?.a.y ?? 0))
      expect(laneWidth, `${lowerId} ↔ ${upperId}`).toBeGreaterThanOrEqual(minimumLaneWidth - 1e-9)
    }
  })

  it('adds an ambiguous parallel corridor scenario with finite initial metrics', () => {
    const state = createSimulationStateForScenario('ambiguous-parallel-corridor')

    expect(state.environment.walls.filter((wall) => wall.id.includes('lower-wall')).length).toBeGreaterThanOrEqual(2)
    expect(state.metrics.wallAssociationAccuracy).toBeGreaterThanOrEqual(0)
    expect(state.metrics.wallAssociationAccuracy).toBeLessThanOrEqual(1)
    expect(
      Object.values(state.metrics).every(
        (value) => value === null || typeof value === 'boolean' || Number.isFinite(value),
      ),
    ).toBe(true)
  })
})
