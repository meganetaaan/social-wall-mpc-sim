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
