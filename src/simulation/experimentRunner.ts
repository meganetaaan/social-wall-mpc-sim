import { defaultPlannerMode } from '../planning/policies'
import { defaultParameters } from './environment'
import { createSimulationStateForScenario, type ScenarioId, scenarioDefinitions } from './scenarios'
import { stepSimulation } from './simulator'
import type { PlannerMode, PlannerParameters, SimulationState } from './types'

export type ScenarioExperimentResult = {
  finalState: SimulationState
  reachedGoal: boolean
  timeToGoal: number | null
  finalGoalDistance: number
  bestGoalDistance: number
  steps: number
}

export type ScenarioExperimentSummary = {
  scenarioId: ScenarioId
  scenarioName: string
  plannerMode: PlannerMode
  reachedGoal: boolean
  timeToGoal: number | null
  finalGoalDistance: number
  bestGoalDistance: number
  steps: number
  elapsedTime: number
  minHumanDistance: number
  socialViolationCount: number
  nearCollisionCount: number
  stopDuration: number
  meanWallDistanceError: number
  maxWallDistanceError: number
  uncertaintyTrace: number
  estimatedMapCoverage: number
  posePositionError: number
  poseHeadingError: number
  poseNormalizedError: number
  mapKnowledgeError: number
}

export type ScenarioBatchExperimentResult = {
  maxSteps: number
  summaries: ScenarioExperimentSummary[]
}

const defaultPlannerModes: PlannerMode[] = ['belief-mpc', 'wall-only', 'reactive-stop']

export function runScenarioExperiment(args: {
  scenarioId: ScenarioId
  plannerMode?: PlannerMode
  parameters?: Partial<PlannerParameters>
  maxSteps: number
}): ScenarioExperimentResult {
  const scenario = scenarioDefinitions.find((candidate) => candidate.id === args.scenarioId)
  if (!scenario) throw new Error(`Unknown scenario: ${args.scenarioId}`)

  const parameters = { ...defaultParameters, ...args.parameters }
  const plannerMode = args.plannerMode ?? defaultPlannerMode
  let state = createSimulationStateForScenario(args.scenarioId)
  let seed = scenario.seed
  let steps = 0

  while (steps < args.maxSteps && !state.metrics.goalReached) {
    state = stepSimulation(state, parameters, seed, plannerMode)
    seed += 1
    steps += 1
  }

  return {
    finalState: state,
    reachedGoal: state.metrics.goalReached,
    timeToGoal: state.metrics.timeToGoal,
    finalGoalDistance: state.metrics.goalDistance,
    bestGoalDistance: state.metrics.bestGoalDistance,
    steps,
  }
}

export function summarizeScenarioExperiment(
  result: ScenarioExperimentResult,
  scenarioId: ScenarioId,
  scenarioName: string,
  plannerMode: PlannerMode,
): ScenarioExperimentSummary {
  const metrics = result.finalState.metrics
  return {
    scenarioId,
    scenarioName,
    plannerMode,
    reachedGoal: result.reachedGoal,
    timeToGoal: result.timeToGoal,
    finalGoalDistance: finiteNumber(result.finalGoalDistance),
    bestGoalDistance: finiteNumber(result.bestGoalDistance),
    steps: result.steps,
    elapsedTime: finiteNumber(metrics.elapsedTime),
    minHumanDistance: finiteNumber(metrics.minHumanDistance),
    socialViolationCount: metrics.socialViolationCount,
    nearCollisionCount: metrics.nearCollisionCount,
    stopDuration: finiteNumber(metrics.stopDuration),
    meanWallDistanceError: finiteNumber(metrics.meanWallDistanceError),
    maxWallDistanceError: finiteNumber(metrics.maxWallDistanceError),
    uncertaintyTrace: finiteNumber(metrics.uncertaintyTrace),
    estimatedMapCoverage: finiteNumber(metrics.estimatedMapCoverage),
    posePositionError: finiteNumber(metrics.posePositionError),
    poseHeadingError: finiteNumber(metrics.poseHeadingError),
    poseNormalizedError: finiteNumber(metrics.poseNormalizedError),
    mapKnowledgeError: finiteNumber(metrics.mapKnowledgeError),
  }
}

export function runScenarioBatch(args: {
  scenarioIds?: readonly ScenarioId[]
  plannerModes?: readonly PlannerMode[]
  parameters?: Partial<PlannerParameters>
  maxSteps: number
}): ScenarioBatchExperimentResult {
  const scenarioIds = args.scenarioIds ?? scenarioDefinitions.map((scenario) => scenario.id)
  const plannerModes = args.plannerModes ?? defaultPlannerModes
  const summaries = scenarioIds.flatMap((scenarioId) => {
    const scenario = scenarioDefinitions.find((candidate) => candidate.id === scenarioId)
    if (!scenario) throw new Error(`Unknown scenario: ${scenarioId}`)

    return plannerModes.map((plannerMode) => {
      const result = runScenarioExperiment({
        scenarioId,
        plannerMode,
        parameters: args.parameters,
        maxSteps: args.maxSteps,
      })
      return summarizeScenarioExperiment(result, scenario.id, scenario.name, plannerMode)
    })
  })

  return { maxSteps: args.maxSteps, summaries }
}

function finiteNumber(value: number): number {
  return Number.isFinite(value) ? value : 0
}
