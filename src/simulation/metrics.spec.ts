import { describe, expect, it } from 'vitest'
import { defaultParameters } from './environment'
import { createInitialMetrics, estimatedMapCoverage, updateSimulationMetrics } from './metrics'
import { createSimulationStateForScenario } from './scenarios'

describe('simulation metrics', () => {
  it('tracks goal distance, sticky arrival, first time to goal, and best distance', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const goalState = {
      ...state,
      robot: { x: state.environment.goal.x + 0.1, y: state.environment.goal.y, theta: 0 },
      environment: { ...state.environment, goalRadius: 0.35 },
    }
    const previous = createInitialMetrics(state, defaultParameters)
    const reached = updateSimulationMetrics({
      previous,
      previousRobot: state.robot,
      robot: goalState.robot,
      humans: goalState.humans,
      environment: goalState.environment,
      belief: goalState.belief,
      control: { v: 0.2, omega: 0 },
      selectedCost: 1,
      parameters: defaultParameters,
    })
    const movedAway = updateSimulationMetrics({
      previous: reached,
      previousRobot: goalState.robot,
      robot: { x: state.environment.goal.x + 1, y: state.environment.goal.y, theta: 0 },
      humans: goalState.humans,
      environment: goalState.environment,
      belief: goalState.belief,
      control: { v: 0.2, omega: 0 },
      selectedCost: 1,
      parameters: defaultParameters,
    })

    expect(reached.goalDistance).toBeCloseTo(0.1)
    expect(reached.goalReached).toBe(true)
    expect(reached.timeToGoal).toBeCloseTo(defaultParameters.dt)
    expect(reached.bestGoalDistance).toBeCloseTo(0.1)
    expect(movedAway.goalReached).toBe(true)
    expect(movedAway.timeToGoal).toBeCloseTo(defaultParameters.dt)
    expect(movedAway.bestGoalDistance).toBeCloseTo(0.1)
  })

  it('uses a default goal radius when the scenario omits one', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const atDefaultRadius = {
      ...state,
      robot: { x: state.environment.goal.x + 0.18, y: state.environment.goal.y, theta: 0 },
      environment: { ...state.environment, goalRadius: undefined },
    }

    const metrics = createInitialMetrics(atDefaultRadius, defaultParameters)

    expect(metrics.goalReached).toBe(true)
    expect(metrics.goalDistance).toBeCloseTo(0.18)
    expect(metrics.timeToGoal).toBe(0)
  })

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
