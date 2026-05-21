import { transitionBelief } from '../belief/beliefTransition'
import { observePointReturns } from '../belief/pointObjectBelief'
import { observeWalls } from '../belief/wallMapBelief'
import { environmentWithBeliefValueField } from '../planning/beliefValueField'
import { planWithPolicy } from '../planning/policies'
import { predictHumans, stepRobotInEnvironment } from './dynamics'
import { updateSimulationMetrics } from './metrics'
import type { PlannerMode, PlannerParameters, SimulationState } from './types'

export function stepSimulation(
  state: SimulationState,
  parameters: PlannerParameters,
  seed = parameters.seed,
  plannerMode: PlannerMode = 'belief-mpc',
): SimulationState {
  const plan = planWithPolicy({ mode: plannerMode, state, parameters, seed: seed + Math.floor(state.time * 1000) })
  const robot = stepRobotInEnvironment(state.robot, plan.bestControl, parameters.dt, state.environment, parameters)
  const humans = predictHumans(state.humans, parameters.dt, undefined, robot, state.time)
  const currentPointObservations = observePointReturns({
    robot,
    environment: state.environment,
    humans,
    parameters,
    time: state.time + parameters.dt,
  })
  const belief = transitionBelief({
    belief: state.belief,
    control: plan.bestControl,
    observedRobot: robot,
    environment: state.environment,
    parameters,
    time: state.time + parameters.dt,
    pointObservations: currentPointObservations,
  })
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
