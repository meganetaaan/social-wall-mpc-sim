import { describe, expect, it } from 'vitest'
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
  it('draws the goal as a bright green marker with a GOAL label', () => {
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
      robot: { x: 1, y: 1, theta: 0 },
      humans: [],
      trace: [],
      candidates: [],
      dMin: 0.7,
      dPref: 1.5,
    })

    expect(calls).toContain('fillStyle:#22c55e')
    expect(calls).toContain('fillText:GOAL')
  })
})
