import { useCallback, useEffect, useRef, useState } from 'react'
import './App.css'
import { defaultPlannerMode } from './planning/policies'
import { CanvasView } from './rendering/CanvasView'
import { createDefaultSimulationState, defaultParameters } from './simulation/environment'
import { createSimulationStateForScenario, type ScenarioId, scenarioDefinitions } from './simulation/scenarios'
import { stepSimulation } from './simulation/simulator'
import type { PlannerMode, PlannerParameters, SimulationState } from './simulation/types'
import { Controls } from './ui/Controls'
import { CostPanel } from './ui/CostPanel'
import { ExperimentSummaryPanel } from './ui/ExperimentSummaryPanel'
import { ParameterPanel } from './ui/ParameterPanel'

export default function App() {
  const [parameters, setParameters] = useState<PlannerParameters>(defaultParameters)
  const [scenarioId, setScenarioId] = useState<ScenarioId>('crossing-human')
  const [plannerMode, setPlannerMode] = useState<PlannerMode>(defaultPlannerMode)
  const [state, setState] = useState<SimulationState>(() => createDefaultSimulationState())
  const [running, setRunning] = useState(true)
  const seedRef = useRef(scenarioDefinitions[0].seed)
  const step = useCallback(
    () => setState((current) => stepSimulation(current, parameters, seedRef.current++, plannerMode)),
    [parameters, plannerMode],
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
    const scenario = scenarioDefinitions.find((candidate) => candidate.id === scenarioId) ?? scenarioDefinitions[0]
    seedRef.current = scenario.seed
    setState(createSimulationStateForScenario(scenario.id))
  }
  const changeScenario = (nextScenarioId: ScenarioId) => {
    const scenario = scenarioDefinitions.find((candidate) => candidate.id === nextScenarioId) ?? scenarioDefinitions[0]
    setScenarioId(scenario.id)
    seedRef.current = scenario.seed
    setState(createSimulationStateForScenario(scenario.id))
  }
  const scenario = scenarioDefinitions.find((candidate) => candidate.id === scenarioId) ?? scenarioDefinitions[0]
  return (
    <main>
      <header>
        <p>Belief-space sampling MPC experiment</p>
        <h1>Socially-aware wall-following robot simulator</h1>
        <p>Compare deterministic scenarios with the proposed unified-cost belief MPC and small baseline policies.</p>
      </header>
      <div className="layout">
        <section className="stage">
          <CanvasView state={state} parameters={parameters} />
          <Controls running={running} onToggle={() => setRunning((value) => !value)} onReset={reset} onStep={step} />
        </section>
        <aside>
          <section className="panel">
            <h2>Experiment</h2>
            <label className="select-row">
              <span>Scenario</span>
              <select value={scenarioId} onChange={(event) => changeScenario(event.currentTarget.value as ScenarioId)}>
                {scenarioDefinitions.map((definition) => (
                  <option key={definition.id} value={definition.id}>
                    {definition.name}
                  </option>
                ))}
              </select>
            </label>
            <p className="readout">{scenario.description}</p>
            <label className="select-row">
              <span>Policy</span>
              <select
                value={plannerMode}
                onChange={(event) => setPlannerMode(event.currentTarget.value as PlannerMode)}
              >
                <option value="belief-mpc">belief-mpc</option>
                <option value="wall-only">wall-only baseline</option>
                <option value="reactive-stop">reactive-stop hard-switch baseline</option>
              </select>
            </label>
          </section>
          <ParameterPanel parameters={parameters} onChange={setParameters} />
          <CostPanel cost={state.costBreakdown} state={state} />
          <ExperimentSummaryPanel parameters={parameters} />
        </aside>
      </div>
    </main>
  )
}
