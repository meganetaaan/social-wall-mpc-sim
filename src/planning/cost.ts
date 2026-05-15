import { mapUncertaintyTrace, traceSigma, updateBelief, wallConfidence } from '../belief/simpleBelief'
import { distance, nearestWall, normAngle, wallTangentAngle } from '../simulation/math'
import type {
  BeliefState,
  ControlInput,
  CostBreakdown,
  Environment,
  HumanState,
  PlannerParameters,
  RobotState,
  WallSegment,
} from '../simulation/types'

const emptyTerms = () => ({
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
})

export function wallFollowingCost(robot: RobotState, wall: WallSegment, p: PlannerParameters) {
  const point = { x: robot.x, y: robot.y }
  const dWall = nearestWall(point, [wall]).distance
  const distanceCost = p.wWall * (dWall - p.dWallTarget) ** 2
  const headingError = normAngle(robot.theta - wallTangentAngle(wall))
  const headingCost = p.wWallHeading * headingError ** 2
  return { distance: distanceCost, heading: headingCost, dWall, headingError }
}

export function humanSocialDistanceCost(robot: RobotState, humans: HumanState[], p: PlannerParameters) {
  let total = 0
  for (const human of humans) {
    const r = distance({ x: robot.x, y: robot.y }, { x: human.x, y: human.y })
    if (r < p.dMin) {
      total += 100_000 * p.wHuman * (1 + (p.dMin - r) ** 2)
    } else {
      total += (p.wHuman * 0.25) / Math.max(0.03, r - p.dMin) ** 2
      total += p.wHuman * Math.max(0, Math.abs(r - p.dPref) - p.dTolerance) ** 2
    }
  }
  return { total }
}

export function collisionCost(robot: RobotState, environment: Environment, humans: HumanState[], p: PlannerParameters) {
  const point = { x: robot.x, y: robot.y }
  const nearest = nearestWall(point, environment.walls)
  let cost = 0
  if (nearest.distance < p.robotRadius) cost += p.wCollision * 1_000 * (p.robotRadius - nearest.distance + 1)
  if (nearest.distance < p.wallCollisionDistance)
    cost += p.wCollision / Math.max(0.03, nearest.distance - p.robotRadius + 0.03) ** 2
  for (const obstacle of environment.obstacles) {
    const clearance = distance(point, obstacle) - obstacle.radius - p.robotRadius
    if (clearance < 0) cost += p.wCollision * 1_000 * (1 - clearance)
    else if (clearance < 0.4) cost += p.wCollision / Math.max(0.03, clearance) ** 2
  }
  for (const human of humans) {
    const clearance = distance(point, human) - human.radius - p.robotRadius
    if (clearance < 0) cost += p.wCollision * 2_000 * (1 - clearance)
  }
  return cost
}

export function evaluateStageCost(args: {
  robot: RobotState
  humans: HumanState[]
  environment: Environment
  belief: BeliefState
  control: ControlInput
  previousControl: ControlInput
  parameters: PlannerParameters
}): CostBreakdown {
  const { robot, humans, environment, belief, control, previousControl, parameters } = args
  const terms = emptyTerms()
  const nearest = nearestWall({ x: robot.x, y: robot.y }, environment.walls)
  const wall = wallFollowingCost(robot, nearest.wall, parameters)
  terms.wall = wall.distance
  terms.wallHeading = wall.heading
  terms.human = humanSocialDistanceCost(robot, humans, parameters).total
  terms.collision = collisionCost(robot, environment, humans, parameters)
  terms.control = parameters.wControlV * control.v ** 2 + parameters.wControlOmega * control.omega ** 2
  terms.smoothness =
    parameters.wSmooth * ((control.v - previousControl.v) ** 2 + (control.omega - previousControl.omega) ** 2)
  terms.progress = -parameters.wProgress * control.v * Math.cos(normAngle(robot.theta - wallTangentAngle(nearest.wall)))
  terms.uncertainty = parameters.wUncertainty * traceSigma(belief)
  terms.mapUncertainty = parameters.wUncertainty * mapUncertaintyTrace(belief)
  terms.observationGain = -expectedObservationGain(robot, control, environment, belief, parameters)
  terms.wallBeliefConsistency = wallBeliefConsistencyCost(robot, environment, belief, parameters)
  return { terms, total: Object.values(terms).reduce((sum, value) => sum + value, 0) }
}

export function expectedObservationGain(
  robot: RobotState,
  control: ControlInput,
  environment: Environment,
  belief: BeliefState,
  p: PlannerParameters,
) {
  const nearest = nearestWall({ x: robot.x, y: robot.y }, environment.walls)
  const confidenceGap = 1 - wallConfidence(belief, nearest.wall.id)
  const sensorOpportunity = Math.max(0, 1 - nearest.distance / 1.8)
  const lookingAlongWall = Math.max(0, Math.cos(normAngle(robot.theta - wallTangentAngle(nearest.wall))))
  return p.wUncertainty * 0.45 * confidenceGap * sensorOpportunity * (0.4 + control.v) * (0.4 + lookingAlongWall)
}

export function wallBeliefConsistencyCost(
  robot: RobotState,
  environment: Environment,
  belief: BeliefState,
  p: PlannerParameters,
) {
  const nearest = nearestWall({ x: robot.x, y: robot.y }, environment.walls)
  const confidenceGap = 1 - wallConfidence(belief, nearest.wall.id)
  return p.wUncertainty * confidenceGap * (nearest.distance - p.dWallTarget) ** 2
}

export function addCost(a: CostBreakdown, b: CostBreakdown): CostBreakdown {
  const terms = emptyTerms()
  for (const key of Object.keys(terms) as (keyof typeof terms)[]) terms[key] = a.terms[key] + b.terms[key]
  return { terms, total: a.total + b.total }
}

export function nextBeliefForCost(
  belief: BeliefState,
  robot: RobotState,
  environment: Environment,
  p: PlannerParameters,
) {
  return updateBelief(belief, robot, environment, p)
}
