export type Vec2 = { x: number; y: number }
export type RobotState = { x: number; y: number; theta: number }
export type ControlInput = { v: number; omega: number }
export type HumanState = { id: string; x: number; y: number; vx: number; vy: number; radius: number }
export type WallSegment = { id: string; a: Vec2; b: Vec2 }
export type StaticObstacle = { id: string; x: number; y: number; radius: number }
export type Environment = { walls: WallSegment[]; obstacles: StaticObstacle[] }
export type BeliefState = { sigmaX: number; sigmaY: number; sigmaTheta: number; mapConfidence: number }
export type CostTerms = {
  wall: number
  wallHeading: number
  human: number
  collision: number
  control: number
  smoothness: number
  progress: number
  uncertainty: number
}
export type CostBreakdown = { terms: CostTerms; total: number }
export type PlannerParameters = {
  dt: number
  horizonSteps: number
  sampleCount: number
  seed: number
  dMin: number
  dPref: number
  dTolerance: number
  dWallTarget: number
  wWall: number
  wWallHeading: number
  wHuman: number
  wCollision: number
  wControlV: number
  wControlOmega: number
  wSmooth: number
  wUncertainty: number
  wProgress: number
  vMin: number
  vMax: number
  omegaMin: number
  omegaMax: number
  robotRadius: number
  wallCollisionDistance: number
}
export type CandidateRollout = {
  controls: ControlInput[]
  trajectory: RobotState[]
  predictedHumans: HumanState[][]
  cost: CostBreakdown
}
export type PlanningResult = {
  candidates: CandidateRollout[]
  selected: CandidateRollout
  bestControl: ControlInput
}
export type SimulationState = {
  time: number
  robot: RobotState
  previousControl: ControlInput
  humans: HumanState[]
  environment: Environment
  belief: BeliefState
  trace: Vec2[]
  plan: PlanningResult
  costBreakdown: CostBreakdown
}
