import { updateBelief } from '../belief/simpleBelief'
import { observeWalls } from '../belief/wallMapBelief'
import { planSamplingMpc } from '../planning/samplingMpc'
import { predictHumans, stepRobot } from './dynamics'
import type { PlannerParameters, SimulationState } from './types'

export function stepSimulation(
  state: SimulationState,
  parameters: PlannerParameters,
  seed = parameters.seed,
): SimulationState {
  const plan = planSamplingMpc({ state, parameters, seed: seed + Math.floor(state.time * 1000) })
  const robot = stepRobot(state.robot, plan.bestControl, parameters.dt)
  const humans = predictHumans(state.humans, parameters.dt)
  const belief = updateBelief(state.belief, robot, state.environment, parameters, state.time + parameters.dt)
  const currentObservations = observeWalls(robot, state.environment, parameters)
  const trace = [...state.trace, { x: robot.x, y: robot.y }].slice(-650)
  return {
    ...state,
    time: state.time + parameters.dt,
    robot,
    humans,
    belief,
    trace,
    currentObservations,
    previousControl: plan.bestControl,
    plan,
    costBreakdown: plan.selected.cost,
  }
}
