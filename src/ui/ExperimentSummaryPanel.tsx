import { useState } from 'react'
import {
  runScenarioBatch,
  type ScenarioBatchExperimentResult,
  type ScenarioExperimentSummary,
} from '../simulation/experimentRunner'
import type { ScenarioId } from '../simulation/scenarios'
import type { PlannerMode, PlannerParameters } from '../simulation/types'

type Props = {
  parameters: PlannerParameters
}

const uiScenarioIds: ScenarioId[] = ['crossing-human', 'standing-human', 'blocked-corridor']
const uiPlannerModes: PlannerMode[] = ['belief-mpc', 'wall-only', 'reactive-stop']
const uiMaxSteps = 240

export function ExperimentSummaryPanel({ parameters }: Props) {
  const [result, setResult] = useState<ScenarioBatchExperimentResult | null>(null)
  const [running, setRunning] = useState(false)

  const runComparison = () => {
    setRunning(true)
    window.setTimeout(() => {
      try {
        setResult(
          runScenarioBatch({
            scenarioIds: uiScenarioIds,
            plannerModes: uiPlannerModes,
            parameters,
            maxSteps: uiMaxSteps,
          }),
        )
      } finally {
        setRunning(false)
      }
    }, 0)
  }

  return (
    <section className="panel experiment-summary">
      <div className="panel-heading">
        <h2>Batch comparison</h2>
        <button type="button" onClick={runComparison} disabled={running}>
          {running ? 'Running...' : 'Run comparison'}
        </button>
      </div>
      <p className="readout">Deterministic headless replay using the same simulator step code as the canvas view.</p>
      {result ? (
        <div className="summary-table-wrap">
          <table className="summary-table">
            <thead>
              <tr>
                <th>Scenario</th>
                <th>Policy</th>
                <th>Goal</th>
                <th>Time</th>
                <th>Best goal dist</th>
                <th>Min human dist</th>
                <th>Social violations</th>
                <th>Near collisions</th>
                <th>Stop duration</th>
              </tr>
            </thead>
            <tbody>
              {result.summaries.map((summary) => (
                <SummaryRow key={`${summary.scenarioId}-${summary.plannerMode}`} summary={summary} />
              ))}
            </tbody>
          </table>
        </div>
      ) : null}
    </section>
  )
}

function SummaryRow({ summary }: { summary: ScenarioExperimentSummary }) {
  return (
    <tr>
      <td>{summary.scenarioName}</td>
      <td>{summary.plannerMode}</td>
      <td>{summary.reachedGoal ? 'yes' : 'no'}</td>
      <td>{summary.timeToGoal === null ? '-' : `${summary.timeToGoal.toFixed(1)}s`}</td>
      <td>{summary.bestGoalDistance.toFixed(2)}m</td>
      <td>{summary.minHumanDistance.toFixed(2)}m</td>
      <td>{summary.socialViolationCount}</td>
      <td>{summary.nearCollisionCount}</td>
      <td>{summary.stopDuration.toFixed(1)}s</td>
    </tr>
  )
}
