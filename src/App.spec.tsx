import { render, screen } from '@testing-library/react'
import { afterEach, describe, expect, it, vi } from 'vitest'
import App from './App'

describe('App', () => {
  afterEach(() => {
    vi.restoreAllMocks()
  })

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

  it('does not let a stale animation frame resubscribe after the simulator unmounts', () => {
    const callbacks = new Map<number, FrameRequestCallback>()
    let nextFrameId = 1
    const requestAnimationFrame = vi
      .spyOn(window, 'requestAnimationFrame')
      .mockImplementation((callback: FrameRequestCallback) => {
        const id = nextFrameId++
        callbacks.set(id, callback)
        return id
      })
    vi.spyOn(window, 'cancelAnimationFrame').mockImplementation((id: number) => {
      callbacks.delete(id)
    })

    const { unmount } = render(<App />)
    const firstCallback = callbacks.values().next().value
    if (!firstCallback) throw new Error('Expected simulator to schedule an animation frame')

    unmount()
    firstCallback(100)

    expect(requestAnimationFrame).toHaveBeenCalledTimes(1)
    expect(callbacks).toHaveLength(0)
  })
})
