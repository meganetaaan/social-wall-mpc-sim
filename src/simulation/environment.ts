import { createSimulationStateForScenario } from './scenarios'
import type { PlannerParameters, SimulationState } from './types'

export const defaultParameters: PlannerParameters = {
  dt: 0.12,
  horizonSteps: 13,
  sampleCount: 90,
  seed: 42,
  dMin: 0.7,
  dPref: 1.55,
  dTolerance: 0.3,
  dWallTarget: 0.85,
  wWall: 11,
  wWallHeading: 2.6,
  wHuman: 10,
  wCollision: 16,
  wControlV: 0.09,
  wControlOmega: 0.05,
  wSmooth: 1.8,
  wUncertainty: 1.4,
  wProgress: 3.5,
  vMin: 0,
  vMax: 0.85,
  omegaMin: -1.6,
  omegaMax: 1.6,
  robotRadius: 0.22,
  wallCollisionDistance: 0.36,
  sensorRadius: 2.25,
  sensorFov: Math.PI * 0.82,
}

export function createDefaultSimulationState(): SimulationState {
  return createSimulationStateForScenario('crossing-human')
}
