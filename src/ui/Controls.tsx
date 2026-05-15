type Props = { running: boolean; onToggle: () => void; onReset: () => void; onStep: () => void }
export function Controls({ running, onToggle, onReset, onStep }: Props) {
  return (
    <section className="panel controls">
      <button type="button" onClick={onToggle}>
        {running ? 'Pause' : 'Start'}
      </button>
      <button type="button" onClick={onStep}>
        Step
      </button>
      <button type="button" onClick={onReset}>
        Reset
      </button>
    </section>
  )
}
