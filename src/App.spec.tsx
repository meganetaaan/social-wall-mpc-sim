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
    expect(screen.getByText(/goal distance/i)).toBeInTheDocument()
    expect(screen.getByText(/goal reached/i)).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /run comparison/i })).toBeInTheDocument()
    expect(screen.getByText(/deterministic headless replay/i)).toBeInTheDocument()
    expect(screen.getByRole('option', { name: /followed from behind/i })).toBeInTheDocument()
    expect(screen.getByRole('option', { name: /overtaking human/i })).toBeInTheDocument()
    expect(screen.getByRole('option', { name: /yielding blocker/i })).toBeInTheDocument()
    expect(screen.getByRole('option', { name: /spiral corridor known map/i })).toBeInTheDocument()
    expect(screen.getByRole('option', { name: /spiral corridor unknown map/i })).toBeInTheDocument()
  })
})
