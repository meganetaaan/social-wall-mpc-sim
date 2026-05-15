# Goal Reaching Evaluation Codex Goal

## Goal

Make it clear, testable, and visible whether the robot reaches the green goal. The user has not yet tried the simulator much and is specifically worried that the robot may not actually arrive at the green `GOAL` marker.

Advance the simulator so goal-reaching is no longer just a visual hope:

1. add explicit goal-arrival state and metrics,
2. add a headless deterministic experiment runner that can simulate a scenario/policy for N steps,
3. tune or extend the unified `belief-mpc` objective so the default scenario can reach the green goal in a reasonable simulated time when not permanently blocked,
4. keep wall-following/social-distance/uncertainty as one optimization problem, not a mode switch,
5. expose the result in the UI and README.

## Repository context

Repository: `/home/openclaw/social-wall-mpc-sim`

Current stack:

- Vite + React + TypeScript
- Canvas rendering
- Vitest
- Biome
- No backend

Current important files:

```text
src/planning/cost.ts
src/planning/rollout.ts
src/planning/samplingMpc.ts
src/planning/policies.ts
src/simulation/scenarios.ts
src/simulation/metrics.ts
src/simulation/simulator.ts
src/simulation/types.ts
src/rendering/draw.ts
src/ui/CostPanel.tsx
src/App.tsx
README.md
```

Current policy modes:

- `belief-mpc` — proposed unified-cost sampling MPC.
- `wall-only` — baseline.
- `reactive-stop` — hard-switch baseline.

Current metrics already include wall error, social violations, stop duration, map coverage, selected cost, etc. Add goal-specific metrics rather than replacing these.

## Requirements

### 1. Add explicit goal reach semantics

Add a goal radius / arrival threshold. Prefer making it part of existing environment/scenario data if clean, for example:

```ts
export type Environment = {
  walls: WallSegment[]
  obstacles: StaticObstacle[]
  goal: Vec2
  goalRadius?: number
}
```

Use a default radius if omitted, e.g. `0.35` or `0.45` meters.

Add metrics/readout fields such as:

```ts
goalDistance: number
goalReached: boolean
timeToGoal: number | null
```

Optional but useful:

```ts
bestGoalDistance: number
```

`goalReached` should become true once the robot is within the goal radius and stay true. `timeToGoal` should record the first simulated time when that happens.

### 2. Visualize goal status

Keep the green goal marker, but make status visible:

- draw the goal radius ring based on the actual threshold,
- visually indicate when reached, e.g. brighter green ring/check label or `GOAL REACHED`,
- show goal distance / reached / time-to-goal in the metrics panel.

Do not put simulation logic in rendering.

### 3. Add a deterministic headless experiment runner

Add a simulation-only utility module such as:

```text
src/simulation/experimentRunner.ts
```

Suggested API:

```ts
export function runScenarioExperiment(args: {
  scenarioId: ScenarioId
  plannerMode?: PlannerMode
  parameters?: Partial<PlannerParameters>
  maxSteps: number
}): {
  finalState: SimulationState
  reachedGoal: boolean
  timeToGoal: number | null
  finalGoalDistance: number
  bestGoalDistance: number
  steps: number
}
```

It should:

- create a deterministic scenario state,
- step with the scenario seed deterministically,
- stop early if the goal is reached,
- work without React/browser rendering.

### 4. Make the default scenario able to reach the goal

The default `crossing-human` scenario should have a realistic path to the green goal and the `belief-mpc` policy should reach it under default parameters within a bounded number of steps.

Acceptance target:

- `runScenarioExperiment({ scenarioId: 'crossing-human', plannerMode: 'belief-mpc', maxSteps: 700 })` should reach the goal.

Keep this computationally reasonable for tests. If 700 steps is too slow, make the test use a slightly smaller sample count/horizon override while preserving behavior.

Important constraints:

- Do not implement a separate global planner.
- Do not add a hard-coded “if near goal then drive to goal” mode switch.
- Do not teleport, clamp, or artificially mark the goal reached.
- The behavior should still come from cost/trajectory optimization.
- A loose goal-progress or terminal goal-distance cost is acceptable because the original formulation includes `goal_progress_cost`.
- It is acceptable to add a goal-progress term to `CostTerms`, or to reinterpret/extend the existing `progress` term if documented clearly. Prefer explicit if that improves inspectability.

Potential implementation directions:

- Add a local goal progress reward based on reduction in distance to `environment.goal` over the rollout.
- Add a terminal goal distance cost at the end of a rollout.
- Blend wall-tangent progress with goal-direction progress, especially when the robot is close enough to the relevant wall corridor.
- Increase sampling diversity or include a few deterministic guide sequences in `samplingMpc` so the optimizer can discover turns toward the goal.
- Adjust scenario goal position if the current goal is visually misleading or unreachable under local wall-following, but do not remove the green goal concept.

If you tune defaults, keep social-distance and wall-following behavior plausible.

### 5. Tests

Use TDD. Add tests before implementation where possible.

Add/update focused tests for:

- goal distance / goal reached / time-to-goal metrics.
- goal radius default behavior.
- experiment runner determinism: same scenario/policy/params yields same goal result.
- default `crossing-human` with `belief-mpc` reaches the goal within the bounded experiment.
- at least one baseline comparison is meaningful, e.g. `wall-only` either fails to reach or accumulates worse social violations in the same bounded run. Do not make this brittle if stochastic; use deterministic seeds.
- rendering/draw test or UI smoke test that goal status text/metric appears.

Keep tests fast enough for `npm test`.

### 6. UI

Show goal progress clearly in the existing metrics/cost area:

- goal distance,
- goal reached yes/no,
- time to goal if reached,
- best goal distance if added.

If the robot reaches the goal, it is okay for the simulator to keep running, but the state should remember the first arrival. Optionally add an obvious celebratory visual state.

### 7. README

Update README with:

- what “goal reached” means,
- how to use the goal metrics,
- how the headless experiment runner verifies default behavior,
- how the goal term fits the unified-cost formulation,
- known limitation: still no global planner; goal reaching is local/receding-horizon and scenario-dependent.

## Verification commands

Run and leave the repository passing:

```bash
npm run lint
npm test
npm run build
```

If visual behavior changed, start the dev server and inspect the page. Check browser console errors if possible.

## Constraints

- Do not commit changes.
- Do not open a PR.
- No backend.
- No heavyweight dependencies.
- Keep algorithm independent from rendering.
- Keep deterministic fixed-seed behavior.
- Keep default `belief-mpc` as the proposed method.
- Baselines remain comparison tools.

## Final report expected from Codex

Report:

- files changed,
- what goal-reaching semantics were added,
- whether the default scenario reaches the goal in the headless experiment,
- tests added,
- verification commands run and results,
- limitations/follow-up suggestions.
