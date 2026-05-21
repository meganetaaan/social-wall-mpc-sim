import { observePointReturns } from '../belief/pointObjectBelief'
import { updateBelief } from '../belief/simpleBelief'
import { observeWalls } from '../belief/wallMapBelief'
import { environmentWithBeliefValueField } from '../planning/beliefValueField'
import { planWithPolicy } from '../planning/policies'
import { predictHumans, stepRobot } from './dynamics'
import { updateSimulationMetrics } from './metrics'
import type { PlannerMode, PlannerParameters, SimulationState } from './types'

export function stepSimulation(
  state: SimulationState,
  parameters: PlannerParameters,
  seed = parameters.seed,
  plannerMode: PlannerMode = 'belief-mpc',
): SimulationState {
  const plan = planWithPolicy({ mode: plannerMode, state, parameters, seed: seed + Math.floor(state.time * 1000) })
  const robot = stepRobot(state.robot, plan.bestControl, parameters.dt)
  const humans = predictHumans(state.humans, parameters.dt, undefined, robot, state.time)
  const currentPointObservations = observePointReturns({
    robot,
    environment: state.environment,
    humans,
    parameters,
    time: state.time + parameters.dt,
  })
  const belief = updateBelief(
    state.belief,
    robot,
    state.environment,
    parameters,
    state.time + parameters.dt,
    currentPointObservations,
  )
  const environment = environmentWithBeliefValueField(state.environment, belief)
  const currentObservations = observeWalls(robot, environment, parameters)
  const trace = [...state.trace, { x: robot.x, y: robot.y }].slice(-650)
  const metrics = updateSimulationMetrics({
    previous: state.metrics,
    previousRobot: state.robot,
    robot,
    humans,
    environment,
    belief,
    control: plan.bestControl,
    selectedCost: plan.selected.cost.total,
    currentObservations,
    parameters,
  })
  return {
    ...state,
    time: state.time + parameters.dt,
    robot,
    humans,
    environment,
    belief,
    trace,
    currentObservations,
    currentPointObservations,
    previousControl: plan.bestControl,
    plan,
    costBreakdown: plan.selected.cost,
    metrics,
  }
}
