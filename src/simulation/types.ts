export type Vec2 = { x: number; y: number }
export type RobotState = { x: number; y: number; theta: number }
export type ControlInput = { v: number; omega: number }
export type PoseCovariance = [[number, number, number], [number, number, number], [number, number, number]]
export type PoseGaussian = {
  mean: RobotState
  covariance: PoseCovariance
}
export type HumanMotionPattern =
  | { kind: 'linear' }
  | { kind: 'follow-robot'; distanceBehind: number; lateralOffset: number; speed: number }
  | { kind: 'scripted'; waypoints: Array<{ at: number; x: number; y: number }>; loop?: boolean }
  | { kind: 'yield-after'; yieldAt: number; target: Vec2; speed: number }
export type HumanState = {
  id: string
  x: number
  y: number
  vx: number
  vy: number
  radius: number
  motion?: HumanMotionPattern
}
export type WallSegment = { id: string; a: Vec2; b: Vec2 }
export type WallBelief = { wallId: string; confidence: number; lastObservedAt: number }
export type EstimatedWallSegment = {
  wallId: string
  tMin: number
  tMax: number
  confidence: number
  lastObservedAt: number
}
export type EstimatedLineFeature = {
  id: string
  a: Vec2
  b: Vec2
  confidence: number
  lastObservedAt: number
  observationCount: number
}
export type MapUpdateMode = 'true-id-coverage' | 'anonymous-line-features'
export type WallObservation = {
  wallId: string
  tMin: number
  tMax: number
  confidence: number
  strength: number
  range: number
  bearing: number
  rangeStdDev: number
  bearingStdDev: number
  rayTarget: Vec2
  sensorPose: RobotState
}
export type PointObservation = {
  id: string
  point: Vec2
  range: number
  bearing: number
  sensorPose: RobotState
  time: number
  source: 'point-sensor'
}
export type ObjectBelief = {
  id: string
  centroid: Vec2
  velocity: Vec2
  radius: number
  observedCount: number
  lastObservedAt: number
  pStatic: number
  pDynamic: number
  pHuman: number
}
export type StaticObstacle = { id: string; x: number; y: number; radius: number }
export type GridValueField = {
  origin: Vec2
  width: number
  height: number
  resolution: number
  values: number[]
  unreachableCost: number
  sourceKey?: string
  sourceWallLength?: number
}
export type Environment = {
  walls: WallSegment[]
  obstacles: StaticObstacle[]
  goal: Vec2
  goalRadius?: number
  valueField?: GridValueField
}
export type BeliefState = {
  pose: PoseGaussian
  sigmaX: number
  sigmaY: number
  sigmaTheta: number
  mapConfidence: number
  wallBeliefs: WallBelief[]
  estimatedWalls: EstimatedWallSegment[]
  estimatedFeatures?: EstimatedLineFeature[]
  objectBeliefs?: ObjectBelief[]
}
export type CostTerms = {
  wall: number
  wallHeading: number
  human: number
  collision: number
  control: number
  smoothness: number
  progress: number
  goalProgress: number
  goalTerminal: number
  uncertainty: number
  mapUncertainty: number
  observationGain: number
  wallBeliefConsistency: number
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
  wGoalProgress: number
  wGoalTerminal: number
  vMin: number
  vMax: number
  omegaMin: number
  omegaMax: number
  robotRadius: number
  wallCollisionDistance: number
  sensorRadius: number
  sensorFov: number
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
export type PlannerMode = 'belief-mpc' | 'wall-only' | 'reactive-stop' | 'state-lattice'
export type SimulationMetrics = {
  elapsedTime: number
  meanWallDistanceError: number
  maxWallDistanceError: number
  minHumanDistance: number
  socialViolationCount: number
  nearCollisionCount: number
  stopDuration: number
  progressAlongWall: number
  uncertaintyTrace: number
  estimatedMapCoverage: number
  posePositionError: number
  poseHeadingError: number
  poseNormalizedError: number
  mapKnowledgeError: number
  wallAssociationAccuracy: number
  selectedCost: number
  goalDistance: number
  goalReached: boolean
  timeToGoal: number | null
  bestGoalDistance: number
}
export type SimulationState = {
  time: number
  robot: RobotState
  previousControl: ControlInput
  humans: HumanState[]
  environment: Environment
  belief: BeliefState
  trace: Vec2[]
  currentObservations: WallObservation[]
  currentPointObservations: PointObservation[]
  plan: PlanningResult
  costBreakdown: CostBreakdown
  metrics: SimulationMetrics
}
