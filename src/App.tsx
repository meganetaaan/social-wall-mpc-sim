import { useCallback, useEffect, useRef, useState } from 'react'
import './App.css'
import { defaultPlannerMode } from './planning/policies'
import { getStateLatticePolicyService } from './planning/stateLatticePolicyService'
import { CanvasView } from './rendering/CanvasView'
import { createDefaultSimulationState, defaultParameters } from './simulation/environment'
import { createSimulationStateForScenario, type ScenarioId, scenarioDefinitions } from './simulation/scenarios'
import { stepSimulation } from './simulation/simulator'
import type { PlannerMode, PlannerParameters, SimulationState } from './simulation/types'
import { Controls } from './ui/Controls'
import { CostPanel } from './ui/CostPanel'
import { ExperimentSummaryPanel } from './ui/ExperimentSummaryPanel'
import { ParameterPanel } from './ui/ParameterPanel'

const stateLatticePolicyUiWorkBudget = 65536

export default function App() {
  const [parameters, setParameters] = useState<PlannerParameters>(defaultParameters)
  const [scenarioId, setScenarioId] = useState<ScenarioId>('crossing-human')
  const [plannerMode, setPlannerMode] = useState<PlannerMode>(defaultPlannerMode)
  const [state, setState] = useState<SimulationState>(() => createDefaultSimulationState())
  const [running, setRunning] = useState(true)
  const [policyStats, setPolicyStats] = useState(() => getStateLatticePolicyService().cacheStats())
  const seedRef = useRef(scenarioDefinitions[0].seed)
  const refreshPolicyStats = useCallback(() => {
    setPolicyStats(getStateLatticePolicyService().cacheStats())
  }, [])
  const advanceStateLatticePolicyWork = useCallback(() => {
    if (plannerMode === 'state-lattice')
      getStateLatticePolicyService().advancePendingBuilds(stateLatticePolicyUiWorkBudget)
    refreshPolicyStats()
  }, [plannerMode, refreshPolicyStats])
  const step = useCallback(() => {
    setState((current) => stepSimulation(current, parameters, seedRef.current++, plannerMode))
    advanceStateLatticePolicyWork()
  }, [advanceStateLatticePolicyWork, parameters, plannerMode])

  useEffect(() => {
    if (!running) return
    let active = true
    let frame = 0
    let last = 0
    const loop = (time: number) => {
      if (!active) return
      if (time - last > 90) {
        step()
        last = time
      }
      if (active) frame = requestAnimationFrame(loop)
    }
    frame = requestAnimationFrame(loop)
    return () => {
      active = false
      cancelAnimationFrame(frame)
    }
  }, [running, step])

  useEffect(() => {
    const interval = window.setInterval(advanceStateLatticePolicyWork, 250)
    return () => window.clearInterval(interval)
  }, [advanceStateLatticePolicyWork])

  const reset = () => {
    const scenario = scenarioDefinitions.find((candidate) => candidate.id === scenarioId) ?? scenarioDefinitions[0]
    seedRef.current = scenario.seed
    setState(createSimulationStateForScenario(scenario.id))
    refreshPolicyStats()
  }
  const changeScenario = (nextScenarioId: ScenarioId) => {
    const scenario = scenarioDefinitions.find((candidate) => candidate.id === nextScenarioId) ?? scenarioDefinitions[0]
    setScenarioId(scenario.id)
    seedRef.current = scenario.seed
    setState(createSimulationStateForScenario(scenario.id))
    refreshPolicyStats()
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
                <option value="state-lattice">state-lattice value policy</option>
                <option value="wall-only">wall-only baseline</option>
                <option value="reactive-stop">reactive-stop hard-switch baseline</option>
              </select>
            </label>
          </section>
          <section className="panel policy-status-panel">
            <h2>State-lattice policy service</h2>
            <dl className="metric-grid policy-status-grid">
              <div>
                <dt>backend</dt>
                <dd>{policyStats.backend}</dd>
              </div>
              <div>
                <dt>ready / pending</dt>
                <dd>
                  {policyStats.ready} / {policyStats.pending}
                </dd>
              </div>
              <div>
                <dt>hits / misses</dt>
                <dd>
                  {policyStats.hits} / {policyStats.misses}
                </dd>
              </div>
              <div>
                <dt>worker error</dt>
                <dd>{policyStats.lastError ?? 'none'}</dd>
              </div>
            </dl>
          </section>
          <ParameterPanel parameters={parameters} onChange={setParameters} />
          <CostPanel cost={state.costBreakdown} state={state} />
          <ExperimentSummaryPanel parameters={parameters} />
        </aside>
      </div>
    </main>
  )
}
