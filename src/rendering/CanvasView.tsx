import { useEffect, useRef } from 'react'
import type { PlannerParameters, SimulationState } from '../simulation/types'
import { drawScene } from './draw'

type Props = { state: SimulationState; parameters: PlannerParameters }

export function CanvasView({ state, parameters }: Props) {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  useEffect(() => {
    const canvas = canvasRef.current
    const ctx = canvas?.getContext('2d')
    if (!canvas || !ctx) return
    const ratio = window.devicePixelRatio || 1
    const rect = canvas.getBoundingClientRect()
    canvas.width = Math.floor(rect.width * ratio)
    canvas.height = Math.floor(rect.height * ratio)
    ctx.setTransform(ratio, 0, 0, ratio, 0, 0)
    drawScene({
      ctx,
      width: rect.width,
      height: rect.height,
      environment: state.environment,
      belief: state.belief,
      robot: state.robot,
      humans: state.humans,
      trace: state.trace,
      candidates: state.plan.candidates,
      selected: state.plan.selected,
      dMin: parameters.dMin,
      dPref: parameters.dPref,
      currentObservations: state.currentObservations,
      sensorRadius: parameters.sensorRadius,
      sensorFov: parameters.sensorFov,
    })
  }, [state, parameters])

  return <canvas ref={canvasRef} className="sim-canvas" aria-label="Top-down socially aware wall following simulator" />
}
