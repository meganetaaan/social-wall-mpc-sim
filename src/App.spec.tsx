import { render, screen } from '@testing-library/react'
import { describe, expect, it } from 'vitest'
import App from './App'

describe('App', () => {
  it('renders the simulator view, controls, tunable parameters, and cost panel', () => {
    render(<App />)

    expect(screen.getByRole('heading', { name: /socially-aware wall-following robot simulator/i })).toBeInTheDocument()
    expect(screen.getByLabelText(/top-down socially aware wall following simulator/i)).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /pause/i })).toBeInTheDocument()
    expect(screen.getByText('d_min')).toBeInTheDocument()
    expect(screen.getByText('sampled trajectories')).toBeInTheDocument()
    expect(screen.getByText(/unified stage-cost breakdown/i)).toBeInTheDocument()
  })
})
