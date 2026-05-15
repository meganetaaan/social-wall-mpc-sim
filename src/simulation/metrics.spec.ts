import { describe, expect, it } from 'vitest'
import { defaultParameters } from './environment'
import { createInitialMetrics, estimatedMapCoverage, updateSimulationMetrics } from './metrics'
import { createSimulationStateForScenario } from './scenarios'

describe('simulation metrics', () => {
  it('detects social violation and stop duration while tracking selected cost', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const blocked = {
      ...state,
      humans: [{ id: 'close-human', x: state.robot.x + 0.2, y: state.robot.y, vx: 0, vy: 0, radius: 0.22 }],
    }
    const metrics = updateSimulationMetrics({
      previous: createInitialMetrics(blocked, defaultParameters),
      previousRobot: blocked.robot,
      robot: blocked.robot,
      humans: blocked.humans,
      environment: blocked.environment,
      belief: blocked.belief,
      control: { v: 0, omega: 0 },
      selectedCost: 12.5,
      parameters: defaultParameters,
    })

    expect(metrics.socialViolationCount).toBe(1)
    expect(metrics.stopDuration).toBeCloseTo(defaultParameters.dt)
    expect(metrics.selectedCost).toBe(12.5)
  })

  it('computes estimated map coverage from observed wall intervals', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const emptyCoverage = estimatedMapCoverage(state.environment, state.belief)
    const fullFirstWall = {
      ...state.belief,
      estimatedWalls: state.belief.estimatedWalls.map((wall, index) =>
        index === 0 ? { ...wall, tMin: 0, tMax: 1, confidence: 1 } : wall,
      ),
    }

    expect(estimatedMapCoverage(state.environment, fullFirstWall)).toBeGreaterThan(emptyCoverage)
  })
})
