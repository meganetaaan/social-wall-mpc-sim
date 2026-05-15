import type { CostBreakdown, SimulationState } from '../simulation/types'

type Props = { cost: CostBreakdown; state: SimulationState }
export function CostPanel({ cost, state }: Props) {
  const max = Math.max(1, ...Object.values(cost.terms).map(Math.abs))
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
        {state.belief.mapConfidence.toFixed(2)}
      </p>
    </section>
  )
}
