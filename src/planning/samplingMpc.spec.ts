import { describe, expect, it } from 'vitest'
import { createDefaultSimulationState, defaultParameters } from '../simulation/environment'
import { createSimulationStateForScenario } from '../simulation/scenarios'
import { planSamplingMpc } from './samplingMpc'

describe('sampling MPC planner', () => {
  it('selects a deterministic best trajectory with candidate rollouts and cost breakdown', () => {
    const state = createDefaultSimulationState()
    const resultA = planSamplingMpc({
      state,
      parameters: { ...defaultParameters, sampleCount: 24, horizonSteps: 8 },
      seed: 1234,
    })
    const resultB = planSamplingMpc({
      state,
      parameters: { ...defaultParameters, sampleCount: 24, horizonSteps: 8 },
      seed: 1234,
    })

    expect(resultA.candidates).toHaveLength(24)
    expect(resultA.selected.trajectory.length).toBe(9)
    expect(resultA.selected.controls.length).toBe(8)
    expect(resultA.bestControl.v).toBeGreaterThanOrEqual(0)
    expect(resultA.bestControl.v).toBeLessThanOrEqual(defaultParameters.vMax)
    expect(resultA.bestControl).toEqual(resultB.bestControl)
    expect(resultA.selected.cost.total).toBeLessThanOrEqual(resultA.candidates[0].cost.total)
  })

  it('normalizes direct programmatic horizon and sample counts to at least one rollout', () => {
    const state = createDefaultSimulationState()
    const result = planSamplingMpc({
      state,
      parameters: { ...defaultParameters, sampleCount: 0, horizonSteps: 0 },
      seed: 5,
    })

    expect(result.candidates).toHaveLength(1)
    expect(result.selected.trajectory.length).toBe(2)
    expect(result.selected.controls.length).toBe(1)
  })

  it('slows down when object belief estimates a likely human blocker ahead', () => {
    const state = createDefaultSimulationState()
    const blocked = {
      ...state,
      humans: [],
      belief: {
        ...state.belief,
        objectBeliefs: [
          {
            id: 'belief-blocker',
            centroid: { x: state.robot.x + 0.75, y: state.robot.y },
            velocity: { x: 0, y: 0 },
            radius: 0.25,
            observedCount: 3,
            lastObservedAt: state.time,
            pStatic: 0.1,
            pDynamic: 0.9,
            pHuman: 0.95,
          },
        ],
      },
    }

    const clear = planSamplingMpc({
      state,
      parameters: { ...defaultParameters, sampleCount: 36, horizonSteps: 6, dMin: 0.9, wHuman: 16 },
      seed: 8,
    })
    const result = planSamplingMpc({
      state: blocked,
      parameters: { ...defaultParameters, sampleCount: 36, horizonSteps: 6, dMin: 0.9, wHuman: 16 },
      seed: 8,
    })

    expect(result.bestControl.v).toBeLessThan(clear.bestControl.v)
  })

  it('can reverse away from a likely human blocker when reverse motion is allowed', () => {
    const state = createDefaultSimulationState()
    const blocked = {
      ...state,
      humans: [],
      belief: {
        ...state.belief,
        objectBeliefs: [
          {
            id: 'belief-blocker',
            centroid: { x: state.robot.x + 0.75, y: state.robot.y },
            velocity: { x: 0, y: 0 },
            radius: 0.25,
            observedCount: 3,
            lastObservedAt: state.time,
            pStatic: 0.1,
            pDynamic: 0.9,
            pHuman: 0.95,
          },
        ],
      },
    }

    const result = planSamplingMpc({
      state: blocked,
      parameters: { ...defaultParameters, sampleCount: 48, horizonSteps: 6, dMin: 0.9 },
      seed: 8,
    })

    expect(result.bestControl.v).toBeLessThan(0)
  })

  it('keeps moving along a spiral value field instead of stalling on an early plateau', () => {
    const state = createSimulationStateForScenario('spiral-known')
    const earlyPlateau = {
      ...state,
      robot: { x: 1.7, y: 0.7, theta: 0 },
      previousControl: { v: 0, omega: 0 },
    }

    const result = planSamplingMpc({
      state: earlyPlateau,
      parameters: { ...defaultParameters, sampleCount: 90, horizonSteps: 13 },
      seed: 500,
    })

    expect(result.bestControl.v).toBeGreaterThan(0.08)
  })
})
