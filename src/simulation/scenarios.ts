import { poseGaussianFromSigmas } from '../belief/poseBelief'
import { environmentWithBeliefValueField, spiralValueFieldOptions } from '../planning/beliefValueField'
import { createGridValueField } from '../planning/valueField'
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
  | 'follow-behind-human'
  | 'overtaking-human'
  | 'yielding-blocker'
  | 'ambiguous-parallel-corridor'
  | 'occluded-corner-human'
  | 'kidnapped-pose-bend'
  | 'spiral-known'
  | 'spiral-unknown'

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
const defaultPoseMean: RobotState = { x: 0, y: 0, theta: 0 }

const baseEnvironment: Environment = {
  goal: { x: 7.1, y: 2.7 },
  goalRadius: 0.21,
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

const scenarioBelief = (
  environment: Environment,
  firstWallCoverage = { tMin: 0.18, tMax: 0.46 },
  mean = defaultPoseMean,
): BeliefState => {
  const sigmaX = 0.12
  const sigmaY = 0.12
  const sigmaTheta = 0.04
  return {
    pose: poseGaussianFromSigmas(mean, sigmaX, sigmaY, sigmaTheta),
    sigmaX,
    sigmaY,
    sigmaTheta,
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
    objectBeliefs: [],
  }
}

const knownMapBelief = (environment: Environment, mean: RobotState = { x: 0.85, y: 0.55, theta: 0 }): BeliefState => {
  const sigmaX = 0.08
  const sigmaY = 0.08
  const sigmaTheta = 0.03
  return {
    pose: poseGaussianFromSigmas(mean, sigmaX, sigmaY, sigmaTheta),
    sigmaX,
    sigmaY,
    sigmaTheta,
    mapConfidence: 0.86,
    wallBeliefs: environment.walls.map((wall) => ({ wallId: wall.id, confidence: 0.88, lastObservedAt: 0 })),
    estimatedWalls: environment.walls.map((wall) => ({
      wallId: wall.id,
      tMin: 0,
      tMax: 1,
      confidence: 0.88,
      lastObservedAt: 0,
    })),
    objectBeliefs: [],
  }
}

const unknownMapBelief = (environment: Environment, mean: RobotState = { x: 0.85, y: 0.55, theta: 0 }): BeliefState => {
  const sigmaX = 0.18
  const sigmaY = 0.18
  const sigmaTheta = 0.08
  return {
    pose: poseGaussianFromSigmas(mean, sigmaX, sigmaY, sigmaTheta),
    sigmaX,
    sigmaY,
    sigmaTheta,
    mapConfidence: 0.12,
    wallBeliefs: environment.walls.map((wall, index) => ({
      wallId: wall.id,
      confidence: index === 0 ? 0.32 : 0.04,
      lastObservedAt: index === 0 ? 0 : -1,
    })),
    estimatedWalls: environment.walls.map((wall, index) => ({
      wallId: wall.id,
      tMin: index === 0 ? 0.08 : 0,
      tMax: index === 0 ? 0.22 : 0,
      confidence: index === 0 ? 0.32 : 0,
      lastObservedAt: index === 0 ? 0 : -1,
    })),
    objectBeliefs: [],
  }
}

const spiralEnvironmentBase: Environment = {
  goal: { x: 5.0, y: 2.95 },
  goalRadius: 0.24,
  walls: [
    { id: 'spiral-outer-bottom', a: { x: 0.35, y: 0.15 }, b: { x: 9.45, y: 0.15 } },
    { id: 'spiral-outer-right', a: { x: 9.45, y: 0.15 }, b: { x: 9.45, y: 4.95 } },
    { id: 'spiral-outer-top', a: { x: 9.45, y: 4.95 }, b: { x: 0.35, y: 4.95 } },
    { id: 'spiral-outer-left', a: { x: 0.35, y: 4.95 }, b: { x: 0.35, y: 0.95 } },
    { id: 'spiral-entry-left-lip', a: { x: 0.35, y: 0.95 }, b: { x: 1.25, y: 0.95 } },
    { id: 'spiral-lane-bottom', a: { x: 1.25, y: 0.95 }, b: { x: 8.35, y: 0.95 } },
    { id: 'spiral-lane-right', a: { x: 8.35, y: 0.95 }, b: { x: 8.35, y: 4.15 } },
    { id: 'spiral-lane-top', a: { x: 8.35, y: 4.15 }, b: { x: 1.65, y: 4.15 } },
    { id: 'spiral-lane-left', a: { x: 1.65, y: 4.15 }, b: { x: 1.65, y: 1.75 } },
    { id: 'spiral-inner-bottom', a: { x: 1.65, y: 1.75 }, b: { x: 7.25, y: 1.75 } },
    { id: 'spiral-inner-right', a: { x: 7.25, y: 1.75 }, b: { x: 7.25, y: 3.35 } },
    { id: 'spiral-inner-top', a: { x: 7.25, y: 3.35 }, b: { x: 2.55, y: 3.35 } },
    { id: 'spiral-inner-left', a: { x: 2.55, y: 3.35 }, b: { x: 2.55, y: 2.55 } },
    { id: 'spiral-center-bottom', a: { x: 2.55, y: 2.55 }, b: { x: 5.0, y: 2.55 } },
  ],
  obstacles: [],
}

const spiralEnvironment: Environment = {
  ...spiralEnvironmentBase,
  valueField: createGridValueField(spiralEnvironmentBase, spiralValueFieldOptions),
}

const ambiguousParallelCorridorEnvironment: Environment = {
  goal: { x: 7.7, y: 2.05 },
  goalRadius: 0.24,
  walls: [
    { id: 'near-lower-wall', a: { x: 0.4, y: 0.45 }, b: { x: 8.2, y: 0.45 } },
    { id: 'near-upper-wall', a: { x: 0.4, y: 1.55 }, b: { x: 8.2, y: 1.55 } },
    { id: 'far-lower-wall', a: { x: 0.4, y: 2.15 }, b: { x: 8.2, y: 2.15 } },
    { id: 'far-upper-wall', a: { x: 0.4, y: 3.25 }, b: { x: 8.2, y: 3.25 } },
    { id: 'left-connect-lower', a: { x: 0.4, y: 0.45 }, b: { x: 0.4, y: 1.55 } },
    { id: 'right-connect-lower', a: { x: 8.2, y: 0.45 }, b: { x: 8.2, y: 1.55 } },
    { id: 'left-connect-far', a: { x: 0.4, y: 2.15 }, b: { x: 0.4, y: 3.25 } },
    { id: 'right-connect-far', a: { x: 8.2, y: 2.15 }, b: { x: 8.2, y: 3.25 } },
  ],
  obstacles: [],
}

const occludedCornerEnvironment: Environment = {
  ...bendEnvironment,
  goal: { x: 8.7, y: 3.55 },
  walls: [
    ...bendEnvironment.walls,
    { id: 'corner-occluder-short-wall', a: { x: 4.15, y: 1.6 }, b: { x: 4.15, y: 2.35 } },
  ],
  obstacles: [...bendEnvironment.obstacles, { id: 'corner-occluder-pillar', x: 4.35, y: 1.95, radius: 0.26 }],
}

const kidnappedPoseBelief = (environment: Environment): BeliefState => {
  const sigmaX = 0.48
  const sigmaY = 0.44
  const sigmaTheta = 0.22
  return {
    ...scenarioBelief(environment, { tMin: 0.04, tMax: 0.18 }, { x: 1.78, y: 1.28, theta: 0.16 }),
    pose: poseGaussianFromSigmas({ x: 1.78, y: 1.28, theta: 0.16 }, sigmaX, sigmaY, sigmaTheta),
    sigmaX,
    sigmaY,
    sigmaTheta,
    mapConfidence: 0.18,
  }
}

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
    initialBelief: (environment) =>
      scenarioBelief(environment, { tMin: 0.05, tMax: 0.24 }, { x: 0.95, y: 0.88, theta: 0 }),
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
  {
    id: 'follow-behind-human',
    name: 'Followed from behind',
    description: 'A person appears behind the robot and keeps following at a close but moving social distance.',
    seed: 411,
    initialRobot: { x: 1.25, y: 0.86, theta: 0 },
    humans: [
      {
        id: 'following-human',
        x: 0.65,
        y: 0.95,
        vx: 0,
        vy: 0,
        radius: 0.23,
        motion: { kind: 'follow-robot', distanceBehind: 0.85, lateralOffset: 0.04, speed: 0.36 },
      },
    ],
    environment: baseEnvironment,
  },
  {
    id: 'overtaking-human',
    name: 'Overtaking human',
    description: 'A faster person starts behind the robot, passes on the outside, and merges ahead in the corridor.',
    seed: 512,
    initialRobot: { x: 1.15, y: 0.86, theta: 0 },
    humans: [
      {
        id: 'overtaking-human',
        x: 0.75,
        y: 1.2,
        vx: 0,
        vy: 0,
        radius: 0.23,
        motion: {
          kind: 'scripted',
          waypoints: [
            { at: 0, x: 0.75, y: 1.2 },
            { at: 4, x: 2.8, y: 1.55 },
            { at: 8, x: 5.2, y: 1.28 },
            { at: 14, x: 7.4, y: 1.1 },
          ],
        },
      },
    ],
    environment: baseEnvironment,
  },
  {
    id: 'yielding-blocker',
    name: 'Yielding blocker',
    description: 'A person blocks the wall corridor at first, then steps aside so the robot can resume.',
    seed: 613,
    initialRobot: { x: 1.0, y: 0.86, theta: 0 },
    humans: [
      {
        id: 'yielding-blocker',
        x: 2.15,
        y: 0.88,
        vx: 0,
        vy: 0,
        radius: 0.3,
        motion: { kind: 'yield-after', yieldAt: 5.5, target: { x: 2.25, y: 1.75 }, speed: 0.36 },
      },
    ],
    environment: baseEnvironment,
  },
  {
    id: 'ambiguous-parallel-corridor',
    name: 'Ambiguous parallel corridor',
    description: 'Two similar parallel lanes expose anonymous wall-observation association ambiguity.',
    seed: 662,
    initialRobot: { x: 1.05, y: 1.0, theta: 0 },
    humans: [{ id: 'corridor-pedestrian', x: 4.8, y: 1.05, vx: -0.03, vy: 0, radius: 0.22 }],
    environment: ambiguousParallelCorridorEnvironment,
    initialBelief: (environment) =>
      scenarioBelief(environment, { tMin: 0.06, tMax: 0.22 }, { x: 1.05, y: 1.0, theta: 0 }),
  },
  {
    id: 'occluded-corner-human',
    name: 'Occluded corner human',
    description: 'A pedestrian is hidden just past a bend, exposing object-belief latency around occlusions.',
    seed: 684,
    initialRobot: { x: 0.95, y: 0.88, theta: 0 },
    humans: [{ id: 'occluded-corner-human', x: 4.95, y: 2.18, vx: -0.02, vy: -0.08, radius: 0.23 }],
    environment: occludedCornerEnvironment,
    initialBelief: (environment) =>
      scenarioBelief(environment, { tMin: 0.04, tMax: 0.18 }, { x: 0.95, y: 0.88, theta: 0 }),
  },
  {
    id: 'kidnapped-pose-bend',
    name: 'Kidnapped pose bend',
    description: 'The true robot starts in a bend while pose belief is offset and broad, stressing E[V] recovery.',
    seed: 696,
    initialRobot: { x: 0.95, y: 0.88, theta: 0 },
    humans: [{ id: 'kidnapped-bend-pedestrian', x: 5.8, y: 2.25, vx: 0.02, vy: -0.04, radius: 0.23 }],
    environment: bendEnvironment,
    initialBelief: kidnappedPoseBelief,
  },
  {
    id: 'spiral-known',
    name: 'Spiral corridor known map',
    description: 'A rectangular spiral corridor with the goal in the center and most walls already known in belief.',
    seed: 714,
    initialRobot: { x: 0.85, y: 0.55, theta: 0 },
    humans: [],
    environment: spiralEnvironment,
    initialBelief: knownMapBelief,
  },
  {
    id: 'spiral-unknown',
    name: 'Spiral corridor unknown map',
    description: 'The same center-goal spiral, but only the entry wall is initially believed with low confidence.',
    seed: 815,
    initialRobot: { x: 0.85, y: 0.55, theta: 0 },
    humans: [],
    environment: spiralEnvironment,
    initialBelief: unknownMapBelief,
  },
]

export function createSimulationStateForScenario(scenarioId: ScenarioId): SimulationState {
  const scenario = scenarioDefinitions.find((candidate) => candidate.id === scenarioId)
  if (!scenario) throw new Error(`Unknown scenario: ${scenarioId}`)
  const baseEnvironment = clone(scenario.environment)
  const initialBelief =
    typeof scenario.initialBelief === 'function'
      ? scenario.initialBelief(baseEnvironment)
      : (scenario.initialBelief ?? scenarioBelief(baseEnvironment, undefined, scenario.initialRobot))
  const environment = environmentWithBeliefValueField(baseEnvironment, initialBelief)
  const state: SimulationState = {
    time: 0,
    robot: clone(scenario.initialRobot),
    previousControl: { v: 0, omega: 0 },
    humans: clone(scenario.humans),
    environment,
    belief: clone(initialBelief),
    trace: [{ x: scenario.initialRobot.x, y: scenario.initialRobot.y }],
    currentObservations: [],
    currentPointObservations: [],
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
      goalProgress: 0,
      goalTerminal: 0,
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
    posePositionError: 0,
    poseHeadingError: 0,
    poseNormalizedError: 0,
    mapKnowledgeError: 0,
    wallAssociationAccuracy: 1,
    selectedCost: 0,
    goalDistance: 0,
    goalReached: false,
    timeToGoal: null,
    bestGoalDistance: Number.POSITIVE_INFINITY,
  }
}

function clone<T>(value: T): T {
  return structuredClone(value)
}
