import { useCallback, useEffect, useRef, useState } from 'react'
import './App.css'
import { CanvasView } from './rendering/CanvasView'
import { createDefaultSimulationState, defaultParameters } from './simulation/environment'
import { stepSimulation } from './simulation/simulator'
import type { PlannerParameters, SimulationState } from './simulation/types'
import { Controls } from './ui/Controls'
import { CostPanel } from './ui/CostPanel'
import { ParameterPanel } from './ui/ParameterPanel'

export default function App() {
  const [parameters, setParameters] = useState<PlannerParameters>(defaultParameters)
  const [state, setState] = useState<SimulationState>(() => createDefaultSimulationState())
  const [running, setRunning] = useState(true)
  const seedRef = useRef(100)
  const step = useCallback(
    () => setState((current) => stepSimulation(current, parameters, seedRef.current++)),
    [parameters],
  )

  useEffect(() => {
    if (!running) return
    let frame = 0
    let last = 0
    const loop = (time: number) => {
      if (time - last > 90) {
        step()
        last = time
      }
      frame = requestAnimationFrame(loop)
    }
    frame = requestAnimationFrame(loop)
    return () => cancelAnimationFrame(frame)
  }, [running, step])

  const reset = () => {
    seedRef.current = 100
    setState(createDefaultSimulationState())
  }
  return (
    <main>
      <header>
        <p>Belief-space sampling MPC experiment</p>
        <h1>Socially-aware wall-following robot simulator</h1>
        <p>
          Wall following, human distance, collision, smoothness, progress, and uncertainty are evaluated in one
          receding-horizon cost—no planner mode switch.
        </p>
      </header>
      <div className="layout">
        <section className="stage">
          <CanvasView state={state} parameters={parameters} />
          <Controls running={running} onToggle={() => setRunning((value) => !value)} onReset={reset} onStep={step} />
        </section>
        <aside>
          <ParameterPanel parameters={parameters} onChange={setParameters} />
          <CostPanel cost={state.costBreakdown} state={state} />
        </aside>
      </div>
    </main>
  )
}
