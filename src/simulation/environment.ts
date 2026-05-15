import type { CostBreakdown, PlannerParameters, PlanningResult, SimulationState } from './types'

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
}

const emptyCost: CostBreakdown = {
  terms: { wall: 0, wallHeading: 0, human: 0, collision: 0, control: 0, smoothness: 0, progress: 0, uncertainty: 0 },
  total: 0,
}
const emptyPlan: PlanningResult = {
  candidates: [],
  selected: { controls: [], trajectory: [], predictedHumans: [], cost: emptyCost },
  bestControl: { v: 0, omega: 0 },
}

export function createDefaultSimulationState(): SimulationState {
  return {
    time: 0,
    robot: { x: 1.0, y: 0.86, theta: 0 },
    previousControl: { v: 0, omega: 0 },
    humans: [
      { id: 'crossing-human', x: 4.0, y: 2.45, vx: 0, vy: -0.26, radius: 0.23 },
      { id: 'standing-human', x: 6.7, y: 1.16, vx: 0.03, vy: 0, radius: 0.23 },
    ],
    environment: {
      walls: [
        { id: 'follow-wall', a: { x: 0.3, y: 0.2 }, b: { x: 9.8, y: 0.2 } },
        { id: 'right-wall', a: { x: 9.8, y: 0.2 }, b: { x: 9.8, y: 4.7 } },
        { id: 'left-short-wall', a: { x: 0.3, y: 0.2 }, b: { x: 0.3, y: 4.2 } },
      ],
      obstacles: [{ id: 'pillar', x: 7.7, y: 2.15, radius: 0.25 }],
    },
    belief: { sigmaX: 0.12, sigmaY: 0.12, sigmaTheta: 0.04, mapConfidence: 0.86 },
    trace: [{ x: 1.0, y: 0.86 }],
    plan: emptyPlan,
    costBreakdown: emptyCost,
  }
}
