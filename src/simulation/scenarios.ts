import { createInitialMetrics } from './metrics'
import type {
  BeliefState,
  CostBreakdown,
  Environment,
  HumanState,
  PlanningResult,
  RobotState,
  SimulationState,
} from './types'

export type ScenarioId =
  | 'crossing-human'
  | 'standing-human'
  | 'head-on-human'
  | 'blocked-corridor'
  | 'partial-map-bend'
  | 'multi-human'

export type ScenarioDefinition = {
  id: ScenarioId
  name: string
  description: string
  seed: number
  initialRobot: RobotState
  humans: HumanState[]
  environment: Environment
  initialBelief?: BeliefState | ((environment: Environment) => BeliefState)
}

const defaultWallTargetDistance = 0.85

const baseEnvironment: Environment = {
  goal: { x: 8.9, y: 3.85 },
  walls: [
    { id: 'follow-wall-start', a: { x: 0.3, y: 0.2 }, b: { x: 2.4, y: 0.2 } },
    { id: 'lower-alcove-left', a: { x: 2.4, y: 0.2 }, b: { x: 2.4, y: 0.95 } },
    { id: 'lower-alcove-back', a: { x: 2.4, y: 0.95 }, b: { x: 3.7, y: 0.95 } },
    { id: 'lower-alcove-right', a: { x: 3.7, y: 0.95 }, b: { x: 3.7, y: 0.2 } },
    { id: 'follow-wall-middle', a: { x: 3.7, y: 0.2 }, b: { x: 6.0, y: 0.2 } },
    { id: 'inner-baffle', a: { x: 6.0, y: 0.2 }, b: { x: 6.0, y: 1.45 } },
    { id: 'upper-bend', a: { x: 6.0, y: 1.45 }, b: { x: 8.5, y: 1.45 } },
    { id: 'right-wall', a: { x: 9.8, y: 0.2 }, b: { x: 9.8, y: 4.7 } },
    { id: 'top-wall', a: { x: 9.8, y: 4.7 }, b: { x: 5.4, y: 4.7 } },
    { id: 'upper-pocket', a: { x: 5.4, y: 4.7 }, b: { x: 5.4, y: 3.4 } },
    { id: 'left-short-wall', a: { x: 0.3, y: 0.2 }, b: { x: 0.3, y: 4.2 } },
  ],
  obstacles: [{ id: 'pillar', x: 7.7, y: 2.15, radius: 0.25 }],
}

const bendEnvironment: Environment = {
  ...baseEnvironment,
  goal: { x: 8.7, y: 3.45 },
  walls: [
    { id: 'entry-wall', a: { x: 0.3, y: 0.25 }, b: { x: 3.2, y: 0.25 } },
    { id: 'bend-wall', a: { x: 3.2, y: 0.25 }, b: { x: 3.2, y: 1.6 } },
    { id: 'upper-corridor-wall', a: { x: 3.2, y: 1.6 }, b: { x: 7.4, y: 1.6 } },
    { id: 'alcove-left', a: { x: 5.0, y: 1.6 }, b: { x: 5.0, y: 2.7 } },
    { id: 'alcove-back', a: { x: 5.0, y: 2.7 }, b: { x: 6.5, y: 2.7 } },
    { id: 'alcove-right', a: { x: 6.5, y: 2.7 }, b: { x: 6.5, y: 1.6 } },
    { id: 'right-wall', a: { x: 9.4, y: 0.3 }, b: { x: 9.4, y: 4.6 } },
    { id: 'top-wall', a: { x: 9.4, y: 4.6 }, b: { x: 0.3, y: 4.6 } },
    { id: 'left-wall', a: { x: 0.3, y: 4.6 }, b: { x: 0.3, y: 0.25 } },
  ],
  obstacles: [{ id: 'bend-pillar', x: 7.7, y: 1.85, radius: 0.28 }],
}

const scenarioBelief = (environment: Environment, firstWallCoverage = { tMin: 0.18, tMax: 0.46 }): BeliefState => ({
  sigmaX: 0.12,
  sigmaY: 0.12,
  sigmaTheta: 0.04,
  mapConfidence: 0.38,
  wallBeliefs: environment.walls.map((wall, index) => ({
    wallId: wall.id,
    confidence: index === 0 ? 0.72 : Math.max(0.12, 0.34 - index * 0.02),
    lastObservedAt: index === 0 ? 0 : -1,
  })),
  estimatedWalls: environment.walls.map((wall, index) => ({
    wallId: wall.id,
    tMin: index === 0 ? firstWallCoverage.tMin : 0,
    tMax: index === 0 ? firstWallCoverage.tMax : 0,
    confidence: index === 0 ? 0.72 : 0,
    lastObservedAt: index === 0 ? 0 : -1,
  })),
})

export const scenarioDefinitions: ScenarioDefinition[] = [
  {
    id: 'crossing-human',
    name: 'Crossing human',
    description: 'Default wall-following scene with a crossing person and a second person near the wall.',
    seed: 42,
    initialRobot: { x: 1.0, y: 0.86, theta: 0 },
    humans: [
      { id: 'crossing-human', x: 4.0, y: 2.45, vx: 0, vy: -0.26, radius: 0.23 },
      { id: 'standing-human', x: 6.7, y: 1.16, vx: 0.03, vy: 0, radius: 0.23 },
    ],
    environment: baseEnvironment,
  },
  {
    id: 'standing-human',
    name: 'Standing human',
    description: 'A person stands close to the followed wall, encouraging a slow yield or deviation.',
    seed: 73,
    initialRobot: { x: 1.1, y: 0.86, theta: 0 },
    humans: [{ id: 'standing-human', x: 4.25, y: 0.92, vx: 0, vy: 0, radius: 0.23 }],
    environment: baseEnvironment,
  },
  {
    id: 'head-on-human',
    name: 'Head-on human',
    description: 'A person walks toward the robot along the same wall corridor.',
    seed: 91,
    initialRobot: { x: 1.1, y: 0.86, theta: 0 },
    humans: [{ id: 'head-on-human', x: 5.4, y: 0.92, vx: -0.22, vy: 0, radius: 0.23 }],
    environment: baseEnvironment,
  },
  {
    id: 'blocked-corridor',
    name: 'Blocked corridor',
    description: 'A near-wall blockage makes stopping the best comparison behavior for several steps.',
    seed: 124,
    initialRobot: { x: 1.0, y: 0.86, theta: 0 },
    humans: [{ id: 'blocking-human', x: 2.05, y: 0.88, vx: 0, vy: 0, radius: 0.3 }],
    environment: {
      ...baseEnvironment,
      obstacles: [...baseEnvironment.obstacles, { id: 'low-block', x: 2.45, y: 0.65, radius: 0.24 }],
    },
  },
  {
    id: 'partial-map-bend',
    name: 'Partial-map bend',
    description: 'A bent and alcove-like wall layout where estimated wall coverage changes planning value.',
    seed: 205,
    initialRobot: { x: 0.95, y: 0.88, theta: 0 },
    humans: [{ id: 'bend-human', x: 5.6, y: 2.25, vx: 0.04, vy: -0.06, radius: 0.23 }],
    environment: bendEnvironment,
    initialBelief: (environment) => scenarioBelief(environment, { tMin: 0.05, tMax: 0.24 }),
  },
  {
    id: 'multi-human',
    name: 'Multi-human',
    description: 'Two moving people create competing social-distance costs near the wall.',
    seed: 312,
    initialRobot: { x: 1.0, y: 0.86, theta: 0 },
    humans: [
      { id: 'crossing-left', x: 3.4, y: 2.35, vx: 0, vy: -0.22, radius: 0.23 },
      { id: 'crossing-right', x: 5.5, y: 0.75, vx: -0.04, vy: 0.2, radius: 0.23 },
      { id: 'slow-near-wall', x: 7.0, y: 1.12, vx: -0.03, vy: 0, radius: 0.23 },
    ],
    environment: baseEnvironment,
  },
]

export function createSimulationStateForScenario(scenarioId: ScenarioId): SimulationState {
  const scenario = scenarioDefinitions.find((candidate) => candidate.id === scenarioId)
  if (!scenario) throw new Error(`Unknown scenario: ${scenarioId}`)
  const environment = clone(scenario.environment)
  const initialBelief =
    typeof scenario.initialBelief === 'function'
      ? scenario.initialBelief(environment)
      : (scenario.initialBelief ?? scenarioBelief(environment))
  const state: SimulationState = {
    time: 0,
    robot: clone(scenario.initialRobot),
    previousControl: { v: 0, omega: 0 },
    humans: clone(scenario.humans),
    environment,
    belief: clone(initialBelief),
    trace: [{ x: scenario.initialRobot.x, y: scenario.initialRobot.y }],
    currentObservations: [],
    plan: emptyPlan(),
    costBreakdown: emptyCost(),
    metrics: emptyMetrics(),
  }
  return { ...state, metrics: createInitialMetrics(state, { dWallTarget: defaultWallTargetDistance }) }
}

function emptyCost(): CostBreakdown {
  return {
    terms: {
      wall: 0,
      wallHeading: 0,
      human: 0,
      collision: 0,
      control: 0,
      smoothness: 0,
      progress: 0,
      uncertainty: 0,
      mapUncertainty: 0,
      observationGain: 0,
      wallBeliefConsistency: 0,
    },
    total: 0,
  }
}

function emptyPlan(): PlanningResult {
  return {
    candidates: [],
    selected: { controls: [], trajectory: [], predictedHumans: [], cost: emptyCost() },
    bestControl: { v: 0, omega: 0 },
  }
}

function emptyMetrics() {
  return {
    elapsedTime: 0,
    meanWallDistanceError: 0,
    maxWallDistanceError: 0,
    minHumanDistance: Number.POSITIVE_INFINITY,
    socialViolationCount: 0,
    nearCollisionCount: 0,
    stopDuration: 0,
    progressAlongWall: 0,
    uncertaintyTrace: 0,
    estimatedMapCoverage: 0,
    selectedCost: 0,
  }
}

function clone<T>(value: T): T {
  return structuredClone(value)
}
