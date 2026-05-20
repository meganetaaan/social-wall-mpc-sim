import { describe, expect, it } from 'vitest'
import { poseGaussianFromSigmas } from '../belief/poseBelief'
import { drawScene } from './draw'

function createRecordingContext() {
  const calls: string[] = []
  const ctx = {
    set fillStyle(value: string) {
      calls.push(`fillStyle:${value}`)
    },
    get fillStyle() {
      return ''
    },
    set strokeStyle(value: string) {
      calls.push(`strokeStyle:${value}`)
    },
    get strokeStyle() {
      return ''
    },
    set lineWidth(_value: number) {},
    get lineWidth() {
      return 1
    },
    set lineCap(_value: CanvasLineCap) {},
    get lineCap() {
      return 'round' as CanvasLineCap
    },
    arc: () => calls.push('arc'),
    beginPath: () => calls.push('beginPath'),
    clearRect: () => {},
    fill: () => calls.push('fill'),
    fillRect: () => {},
    fillText: (text: string) => calls.push(`fillText:${text}`),
    lineTo: () => {},
    moveTo: () => {},
    stroke: () => calls.push('stroke'),
  } as unknown as CanvasRenderingContext2D
  return { ctx, calls }
}

describe('drawScene', () => {
  it('draws the goal radius and reached status from simulation state', () => {
    const { ctx, calls } = createRecordingContext()

    drawScene({
      ctx,
      width: 800,
      height: 480,
      environment: {
        goal: { x: 8.9, y: 3.85 },
        goalRadius: 0.45,
        walls: [{ id: 'wall', a: { x: 0, y: 0 }, b: { x: 10, y: 0 } }],
        obstacles: [],
      },
      robot: { x: 1, y: 1, theta: 0 },
      humans: [],
      trace: [],
      candidates: [],
      dMin: 0.7,
      dPref: 1.5,
      sensorRadius: 2,
      sensorFov: Math.PI / 2,
      goalReached: true,
    })

    expect(calls).toContain('fillStyle:#22c55e')
    expect(calls).toContain('fillText:GOAL REACHED')
  })

  it('draws wall map confidence as a belief layer', () => {
    const { ctx, calls } = createRecordingContext()

    drawScene({
      ctx,
      width: 800,
      height: 480,
      environment: {
        goal: { x: 8.9, y: 3.85 },
        walls: [{ id: 'wall', a: { x: 0, y: 0 }, b: { x: 10, y: 0 } }],
        obstacles: [],
      },
      belief: {
        pose: poseGaussianFromSigmas({ x: 1, y: 1, theta: 0 }, 0.1, 0.1, 0.02),
        sigmaX: 0.1,
        sigmaY: 0.1,
        sigmaTheta: 0.02,
        mapConfidence: 0.24,
        wallBeliefs: [{ wallId: 'wall', confidence: 0.24, lastObservedAt: 0 }],
        estimatedWalls: [{ wallId: 'wall', tMin: 0.2, tMax: 0.55, confidence: 0.72, lastObservedAt: 0 }],
      },
      currentObservations: [
        {
          wallId: 'wall',
          tMin: 0.25,
          tMax: 0.5,
          confidence: 0.8,
          strength: 0.7,
          range: 1,
          bearing: 0,
          rangeStdDev: 0.03,
          bearingStdDev: 0.01,
          rayTarget: { x: 1, y: 0 },
          sensorPose: { x: 1, y: 1, theta: 0 },
        },
      ],
      robot: { x: 1, y: 1, theta: 0 },
      humans: [],
      trace: [],
      candidates: [],
      dMin: 0.7,
      dPref: 1.5,
      sensorRadius: 2,
      sensorFov: Math.PI / 2,
    })

    expect(calls).toContain('fillText:estimated wall coverage')
    expect(calls.some((call) => call.startsWith('strokeStyle:rgba(125, 211, 252'))).toBe(true)
    expect(calls).toContain('strokeStyle:rgba(34, 211, 238, 0.78)')
  })
})
