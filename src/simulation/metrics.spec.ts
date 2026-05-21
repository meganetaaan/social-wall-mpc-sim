import { describe, expect, it } from 'vitest'
import { expectedRangeBearingToWall, observeWalls } from '../belief/wallMapBelief'
import { defaultParameters } from './environment'
import { createInitialMetrics, estimatedMapCoverage, updateSimulationMetrics } from './metrics'
import { createSimulationStateForScenario } from './scenarios'

describe('simulation metrics', () => {
  it('reports zero pose inference error when belief mean equals the robot pose', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const aligned = {
      ...state,
      belief: {
        ...state.belief,
        pose: { ...state.belief.pose, mean: state.robot },
      },
    }

    const metrics = createInitialMetrics(aligned, defaultParameters)

    expect(metrics.posePositionError).toBeCloseTo(0)
    expect(metrics.poseHeadingError).toBeCloseTo(0)
    expect(metrics.poseNormalizedError).toBeCloseTo(0)
  })

  it('reports positive pose inference error when belief mean differs from the robot pose', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const offset = {
      ...state,
      belief: {
        ...state.belief,
        pose: {
          ...state.belief.pose,
          mean: { x: state.robot.x + 0.3, y: state.robot.y - 0.4, theta: state.robot.theta + 0.25 },
        },
      },
    }

    const metrics = createInitialMetrics(offset, defaultParameters)

    expect(metrics.posePositionError).toBeGreaterThan(0)
    expect(metrics.poseHeadingError).toBeGreaterThan(0)
    expect(metrics.poseNormalizedError).toBeGreaterThan(0)
  })

  it('increases normalized pose error for the same pose error under smaller covariance', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const mean = { x: state.robot.x + 0.2, y: state.robot.y, theta: state.robot.theta + 0.1 }
    const loose = createInitialMetrics(
      {
        ...state,
        belief: {
          ...state.belief,
          pose: {
            mean,
            covariance: [
              [0.25, 0, 0],
              [0, 0.25, 0],
              [0, 0, 0.04],
            ],
          },
        },
      },
      defaultParameters,
    )
    const tight = createInitialMetrics(
      {
        ...state,
        belief: {
          ...state.belief,
          pose: {
            mean,
            covariance: [
              [0.01, 0, 0],
              [0, 0.01, 0],
              [0, 0, 0.0025],
            ],
          },
        },
      },
      defaultParameters,
    )

    expect(tight.poseNormalizedError).toBeGreaterThan(loose.poseNormalizedError)
  })

  it('decreases map knowledge error as estimated wall coverage and confidence improve', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const poorMetrics = createInitialMetrics(
      {
        ...state,
        belief: {
          ...state.belief,
          estimatedWalls: [],
        },
      },
      defaultParameters,
    )
    const goodMetrics = createInitialMetrics(
      {
        ...state,
        belief: {
          ...state.belief,
          estimatedWalls: state.environment.walls.map((wall) => ({
            wallId: wall.id,
            tMin: 0,
            tMax: 1,
            confidence: 1,
            lastObservedAt: 0,
          })),
        },
      },
      defaultParameters,
    )

    expect(goodMetrics.mapKnowledgeError).toBeGreaterThanOrEqual(0)
    expect(goodMetrics.mapKnowledgeError).toBeLessThan(poorMetrics.mapKnowledgeError)
    expect(poorMetrics.mapKnowledgeError).toBeLessThanOrEqual(1)
  })

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

  it('reports bounded finite wall association accuracy', () => {
    const state = createSimulationStateForScenario('crossing-human')
    const currentObservations = observeWalls(state.robot, state.environment, defaultParameters)

    const metrics = createInitialMetrics({ ...state, currentObservations }, defaultParameters)

    expect(Number.isFinite(metrics.wallAssociationAccuracy)).toBe(true)
    expect(metrics.wallAssociationAccuracy).toBeGreaterThanOrEqual(0)
    expect(metrics.wallAssociationAccuracy).toBeLessThanOrEqual(1)
  })

  it('treats no wall observations as perfect association accuracy because no wrong associations were observed', () => {
    const state = createSimulationStateForScenario('crossing-human')

    const metrics = createInitialMetrics({ ...state, currentObservations: [] }, defaultParameters)

    expect(metrics.wallAssociationAccuracy).toBe(1)
  })

  it('lowers wall association accuracy for a bad observation that fits another wall better', () => {
    const state = createSimulationStateForScenario('ambiguous-parallel-corridor')
    const trueWall = state.environment.walls.find((wall) => wall.id === 'near-lower-wall')
    const wrongWall = state.environment.walls.find((wall) => wall.id === 'far-lower-wall')
    if (!trueWall || !wrongWall) throw new Error('Missing ambiguous corridor walls')
    const wrongExpected = expectedRangeBearingToWall(state.robot, wrongWall)
    const badObservation = {
      wallId: trueWall.id,
      tMin: 0.2,
      tMax: 0.8,
      confidence: 0.9,
      strength: 0.8,
      range: wrongExpected.range,
      bearing: wrongExpected.bearing,
      rangeStdDev: 0.03,
      bearingStdDev: 0.01,
      rayTarget: wrongExpected.point,
      sensorPose: state.robot,
    }

    const metrics = createInitialMetrics({ ...state, currentObservations: [badObservation] }, defaultParameters)

    expect(metrics.wallAssociationAccuracy).toBeLessThan(1)
  })
})
