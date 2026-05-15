import type { CostBreakdown, SimulationState } from '../simulation/types'

type Props = { cost: CostBreakdown; state: SimulationState }
export function CostPanel({ cost, state }: Props) {
  const max = Math.max(1, ...Object.values(cost.terms).map(Math.abs))
  const coverage =
    state.belief.estimatedWalls.reduce((sum, wall) => sum + Math.max(0, wall.tMax - wall.tMin), 0) /
    Math.max(1, state.belief.estimatedWalls.length)
  return (
    <section className="panel">
      <h2>Unified stage-cost breakdown</h2>
      <dl className="cost-list">
        {Object.entries(cost.terms).map(([key, value]) => (
          <div key={key}>
            <dt>{key}</dt>
            <dd>
              <span style={{ width: `${Math.min(100, (Math.abs(value) / max) * 100)}%` }} />
              {value.toFixed(2)}
            </dd>
          </div>
        ))}
      </dl>
      <p className="readout">
        total {cost.total.toFixed(2)} · v {state.previousControl.v.toFixed(2)} · ω{' '}
        {state.previousControl.omega.toFixed(2)} · tr(Σ){' '}
        {(state.belief.sigmaX + state.belief.sigmaY + state.belief.sigmaTheta).toFixed(2)} · map{' '}
        {state.belief.mapConfidence.toFixed(2)} · cov {coverage.toFixed(2)} · rays {state.currentObservations.length}
      </p>
      <h2>Experiment metrics</h2>
      <dl className="metric-grid">
        <div>
          <dt>elapsed</dt>
          <dd>{state.metrics.elapsedTime.toFixed(1)}s</dd>
        </div>
        <div>
          <dt>mean wall err</dt>
          <dd>{state.metrics.meanWallDistanceError.toFixed(2)}m</dd>
        </div>
        <div>
          <dt>max wall err</dt>
          <dd>{state.metrics.maxWallDistanceError.toFixed(2)}m</dd>
        </div>
        <div>
          <dt>min human dist</dt>
          <dd>{state.metrics.minHumanDistance.toFixed(2)}m</dd>
        </div>
        <div>
          <dt>social violations</dt>
          <dd>{state.metrics.socialViolationCount}</dd>
        </div>
        <div>
          <dt>near collisions</dt>
          <dd>{state.metrics.nearCollisionCount}</dd>
        </div>
        <div>
          <dt>stop duration</dt>
          <dd>{state.metrics.stopDuration.toFixed(1)}s</dd>
        </div>
        <div>
          <dt>wall progress</dt>
          <dd>{state.metrics.progressAlongWall.toFixed(2)}m</dd>
        </div>
        <div>
          <dt>goal distance</dt>
          <dd>{state.metrics.goalDistance.toFixed(2)}m</dd>
        </div>
        <div>
          <dt>goal reached</dt>
          <dd>{state.metrics.goalReached ? 'yes' : 'no'}</dd>
        </div>
        <div>
          <dt>time to goal</dt>
          <dd>{state.metrics.timeToGoal === null ? '-' : `${state.metrics.timeToGoal.toFixed(1)}s`}</dd>
        </div>
        <div>
          <dt>best goal dist</dt>
          <dd>{state.metrics.bestGoalDistance.toFixed(2)}m</dd>
        </div>
        <div>
          <dt>trace sigma</dt>
          <dd>{state.metrics.uncertaintyTrace.toFixed(2)}</dd>
        </div>
        <div>
          <dt>map coverage</dt>
          <dd>{state.metrics.estimatedMapCoverage.toFixed(2)}</dd>
        </div>
        <div>
          <dt>selected cost</dt>
          <dd>{state.metrics.selectedCost.toFixed(2)}</dd>
        </div>
      </dl>
    </section>
  )
}
