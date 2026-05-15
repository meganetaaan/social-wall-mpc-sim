import type { PlannerParameters } from '../simulation/types'

type NumericKey = keyof Pick<
  PlannerParameters,
  | 'dMin'
  | 'dPref'
  | 'dWallTarget'
  | 'wWall'
  | 'wHuman'
  | 'wControlV'
  | 'wSmooth'
  | 'wUncertainty'
  | 'wGoalProgress'
  | 'wGoalTerminal'
  | 'sensorRadius'
  | 'sensorFov'
  | 'horizonSteps'
  | 'sampleCount'
>
const rows: { key: NumericKey; label: string; min: number; max: number; step: number }[] = [
  { key: 'dMin', label: 'd_min', min: 0.25, max: 1.5, step: 0.05 },
  { key: 'dPref', label: 'd_pref', min: 0.6, max: 3, step: 0.05 },
  { key: 'dWallTarget', label: 'd_wall_target', min: 0.35, max: 1.8, step: 0.05 },
  { key: 'wWall', label: 'w_wall', min: 0, max: 30, step: 0.5 },
  { key: 'wHuman', label: 'w_human', min: 0, max: 30, step: 0.5 },
  { key: 'wControlV', label: 'w_control', min: 0, max: 3, step: 0.05 },
  { key: 'wSmooth', label: 'w_smooth', min: 0, max: 8, step: 0.1 },
  { key: 'wUncertainty', label: 'w_uncertainty', min: 0, max: 8, step: 0.1 },
  { key: 'wGoalProgress', label: 'w_goal_progress', min: 0, max: 30, step: 0.1 },
  { key: 'wGoalTerminal', label: 'w_goal_terminal', min: 0, max: 60, step: 0.1 },
  { key: 'sensorRadius', label: 'sensor radius', min: 0.8, max: 4, step: 0.05 },
  { key: 'sensorFov', label: 'sensor FOV rad', min: 0.5, max: Math.PI * 1.6, step: 0.05 },
  { key: 'horizonSteps', label: 'prediction horizon', min: 4, max: 24, step: 1 },
  { key: 'sampleCount', label: 'sampled trajectories', min: 10, max: 180, step: 1 },
]

type Props = { parameters: PlannerParameters; onChange: (next: PlannerParameters) => void }
export function ParameterPanel({ parameters, onChange }: Props) {
  return (
    <section className="panel">
      <h2>Parameters</h2>
      {rows.map((row) => (
        <label className="slider-row" key={row.key}>
          <span>
            {row.label}
            <strong>{parameters[row.key].toFixed(row.step < 1 ? 2 : 0)}</strong>
          </span>
          <input
            type="range"
            min={row.min}
            max={row.max}
            step={row.step}
            value={parameters[row.key]}
            onChange={(event) => onChange({ ...parameters, [row.key]: Number(event.currentTarget.value) })}
          />
        </label>
      ))}
    </section>
  )
}
