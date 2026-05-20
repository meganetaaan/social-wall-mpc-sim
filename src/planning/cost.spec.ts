import { describe, expect, it } from 'vitest'
import { poseGaussianFromSigmas } from '../belief/poseBelief'
import { defaultParameters } from '../simulation/environment'
import type { BeliefState, ControlInput, HumanState, RobotState, WallSegment } from '../simulation/types'
import { evaluateStageCost, humanSocialDistanceCost, wallFollowingCost } from './cost'

const wall: WallSegment = { id: 'north-wall', a: { x: 0, y: 0 }, b: { x: 10, y: 0 } }
const robot: RobotState = { x: 1, y: 1, theta: 0 }
const control: ControlInput = { v: 0.6, omega: 0 }
const belief: BeliefState = {
  pose: poseGaussianFromSigmas(robot, 0.1, 0.2, 0.05),
  sigmaX: 0.1,
  sigmaY: 0.2,
  sigmaTheta: 0.05,
  mapConfidence: 0.9,
  wallBeliefs: [{ wallId: 'north-wall', confidence: 0.9, lastObservedAt: 0 }],
  estimatedWalls: [{ wallId: 'north-wall', tMin: 0, tMax: 0.8, confidence: 0.9, lastObservedAt: 0 }],
}

describe('modular MPC cost terms', () => {
  it('keeps wall distance and heading costs inspectable', () => {
    const params = { ...defaultParameters, dWallTarget: 1, wWall: 8, wWallHeading: 3 }

    const onTarget = wallFollowingCost(robot, wall, params)
    const tooFar = wallFollowingCost({ ...robot, y: 1.8 }, wall, params)
    const wrongHeading = wallFollowingCost({ ...robot, theta: Math.PI / 2 }, wall, params)

    expect(onTarget.distance).toBeCloseTo(0)
    expect(tooFar.distance).toBeGreaterThan(onTarget.distance)
    expect(wrongHeading.heading).toBeGreaterThan(onTarget.heading)
  })

  it('penalizes humans asymmetrically and extremely inside minimum distance', () => {
    const params = { ...defaultParameters, dMin: 0.7, dPref: 1.6, wHuman: 10 }
    const farHuman: HumanState = { id: 'far', x: 1, y: 3.5, vx: 0, vy: 0, radius: 0.22 }
    const preferredHuman: HumanState = { id: 'pref', x: 1, y: 2.6, vx: 0, vy: 0, radius: 0.22 }
    const tooCloseHuman: HumanState = { id: 'near', x: 1, y: 1.3, vx: 0, vy: 0, radius: 0.22 }

    expect(humanSocialDistanceCost(robot, [preferredHuman], params).total).toBeLessThan(
      humanSocialDistanceCost(robot, [farHuman], params).total,
    )
    expect(humanSocialDistanceCost(robot, [tooCloseHuman], params).total).toBeGreaterThan(10_000)
  })

  it('returns a complete cost breakdown for one optimization stage', () => {
    const breakdown = evaluateStageCost({
      robot,
      humans: [{ id: 'h1', x: 2, y: 1, vx: 0, vy: 0, radius: 0.22 }],
      environment: { walls: [wall], obstacles: [], goal: { x: 8.9, y: 3.85 } },
      belief,
      control,
      previousControl: { v: 0.4, omega: 0.1 },
      parameters: defaultParameters,
    })

    expect(Object.keys(breakdown.terms).sort()).toEqual([
      'collision',
      'control',
      'curiosity',
      'goalProgress',
      'goalTerminal',
      'human',
      'mapUncertainty',
      'observationGain',
      'progress',
      'smoothness',
      'stall',
      'uncertainty',
      'wall',
      'wallBeliefConsistency',
      'wallHeading',
    ])
    expect(breakdown.total).toBeGreaterThan(0)
  })

  it('rewards controls that make local progress toward the goal', () => {
    const environment = { walls: [wall], obstacles: [], goal: { x: 8, y: 1 } }
    const forward = evaluateStageCost({
      robot,
      humans: [],
      environment,
      belief,
      control: { v: 0.6, omega: 0 },
      previousControl: { v: 0.4, omega: 0 },
      parameters: defaultParameters,
    })
    const stopped = evaluateStageCost({
      robot,
      humans: [],
      environment,
      belief,
      control: { v: 0, omega: 0 },
      previousControl: { v: 0.4, omega: 0 },
      parameters: defaultParameters,
    })

    expect(forward.terms.goalProgress).toBeLessThan(stopped.terms.goalProgress)
  })

  it('penalizes stopping while still far from the goal', () => {
    const environment = { walls: [wall], obstacles: [], goal: { x: 8, y: 1 }, goalRadius: 0.25 }
    const stopped = evaluateStageCost({
      robot,
      humans: [],
      environment,
      belief,
      control: { v: 0, omega: 0 },
      previousControl: { v: 0.1, omega: 0 },
      parameters: defaultParameters,
    })
    const moving = evaluateStageCost({
      robot,
      humans: [],
      environment,
      belief,
      control: { v: 0.35, omega: 0 },
      previousControl: { v: 0.1, omega: 0 },
      parameters: defaultParameters,
    })

    expect(stopped.terms.stall).toBeGreaterThan(0)
    expect(stopped.terms.stall).toBeGreaterThan(moving.terms.stall)
  })

  it('rewards observing unknown wall coverage more than already-known wall coverage', () => {
    const unknownBelief: BeliefState = {
      ...belief,
      wallBeliefs: [{ wallId: wall.id, confidence: 0.05, lastObservedAt: -1 }],
      estimatedWalls: [{ wallId: wall.id, tMin: 0, tMax: 0, confidence: 0, lastObservedAt: -1 }],
      mapConfidence: 0.05,
    }
    const knownBelief: BeliefState = {
      ...belief,
      wallBeliefs: [{ wallId: wall.id, confidence: 1, lastObservedAt: 0 }],
      estimatedWalls: [{ wallId: wall.id, tMin: 0, tMax: 1, confidence: 1, lastObservedAt: 0 }],
      mapConfidence: 1,
    }
    const environment = { walls: [wall], obstacles: [], goal: { x: 8, y: 1 } }

    const unknown = evaluateStageCost({
      robot,
      humans: [],
      environment,
      belief: unknownBelief,
      control: { v: 0.4, omega: 0 },
      previousControl: { v: 0.2, omega: 0 },
      parameters: defaultParameters,
    })
    const known = evaluateStageCost({
      robot,
      humans: [],
      environment,
      belief: knownBelief,
      control: { v: 0.4, omega: 0 },
      previousControl: { v: 0.2, omega: 0 },
      parameters: defaultParameters,
    })

    expect(unknown.terms.curiosity).toBeLessThan(known.terms.curiosity)
  })
})
