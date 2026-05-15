# Codex Goal: Experiment Suite and Comparison Panel

## Context

Repository: `/home/openclaw/social-wall-mpc-sim`

This is a Vite + React + TypeScript browser simulator for a socially-aware wall-following robot using a unified belief-space sampling MPC objective. The current app already has:

- deterministic scenarios in `src/simulation/scenarios.ts`
- policies: `belief-mpc`, `wall-only`, `reactive-stop`
- a headless single-scenario runner in `src/simulation/experimentRunner.ts`
- metrics in `SimulationMetrics`
- UI controls in `src/App.tsx` and `src/ui/*`
- README sections explaining math, cost terms, and goal reaching

The user asked to complete this six-step next phase:

1. Add `runScenarioBatch`
2. Run multiple scenario × policy combinations headlessly
3. Return summary metrics
4. Add Vitest regression tests
5. Add a browser UI button/result table for comparison
6. Add README measured-results table

## Goal

Turn the simulator from a single-run demo into a small reproducible experiment workbench. Keep the proposed method (`belief-mpc`) unified-cost; baselines are only comparison tools.

## Constraints

- Use strict TDD where practical: add/update focused tests first and verify RED before implementation.
- Keep the algorithm independent from rendering.
- Keep deterministic behavior: same scenario, policy, params, and step bound should produce identical summaries.
- No backend.
- No new heavy dependencies.
- Follow current Biome formatting.
- Do not introduce a global planner or hard-coded goal mode.
- Do not commit; leave changes for the orchestrating Hermes agent to inspect, verify, and commit.

## Required implementation

### 1. Extend `src/simulation/experimentRunner.ts`

Add types and functions for batch experiments.

Suggested API:

```ts
export type ScenarioExperimentSummary = {
  scenarioId: ScenarioId
  scenarioName: string
  plannerMode: PlannerMode
  reachedGoal: boolean
  timeToGoal: number | null
  finalGoalDistance: number
  bestGoalDistance: number
  steps: number
  elapsedTime: number
  minHumanDistance: number
  socialViolationCount: number
  nearCollisionCount: number
  stopDuration: number
  meanWallDistanceError: number
  maxWallDistanceError: number
  uncertaintyTrace: number
  estimatedMapCoverage: number
}

export type ScenarioBatchExperimentResult = {
  maxSteps: number
  summaries: ScenarioExperimentSummary[]
}

export function summarizeScenarioExperiment(
  result: ScenarioExperimentResult,
  scenarioId: ScenarioId,
  scenarioName: string,
  plannerMode: PlannerMode,
): ScenarioExperimentSummary

export function runScenarioBatch(args: {
  scenarioIds?: ScenarioId[]
  plannerModes?: PlannerMode[]
  parameters?: Partial<PlannerParameters>
  maxSteps: number
}): ScenarioBatchExperimentResult
```

Default `scenarioIds`: all `scenarioDefinitions`.
Default `plannerModes`: `['belief-mpc', 'wall-only', 'reactive-stop']`.

`runScenarioBatch` should call the existing `runScenarioExperiment` for each pair and return stable ordered summaries.

### 2. Tests

Update `src/simulation/experimentRunner.spec.ts` or add a separate spec.

Add tests that:

- `runScenarioBatch` returns `scenarioIds.length * plannerModes.length` rows.
- Rows are deterministic for the same input.
- Summaries include readable scenario names and policy names.
- For a focused small batch, `belief-mpc` has a meaningful comparison against at least one baseline: either reaches goal, has no more social violations, or has a better best-goal distance. Avoid overly brittle exact metric values.
- Ensure no `NaN` summary values for numeric fields.

Use reduced params like `{ sampleCount: 8, horizonSteps: 5 }` where possible for speed, but include at least one test that uses the default `belief-mpc` if already present.

### 3. UI result panel

Add a UI component such as `src/ui/ExperimentSummaryPanel.tsx`.

Requirements:

- A button labeled clearly, e.g. `Run comparison`.
- When clicked, run a deterministic batch in the browser using current parameters.
- To avoid freezing too long, use a focused default UI batch:
  - scenarios: `crossing-human`, `standing-human`, `blocked-corridor` or similar representative subset
  - policies: `belief-mpc`, `wall-only`, `reactive-stop`
  - maxSteps: around `240` or a value that keeps browser responsive
- Show a table with columns:
  - Scenario
  - Policy
  - Goal
  - Time
  - Best goal dist
  - Min human dist
  - Social violations
  - Near collisions
  - Stop duration
- Show a short note that this is deterministic headless replay using the same simulator step code.
- Keep it modeless and compact.
- It is okay if the UI run is synchronous for this prototype, but set a `running`/`busy` state so the button label changes while computing.

Wire it into `src/App.tsx` below/near the current `CostPanel`.

### 4. README measured-results table

Update `README.md` with an “Experiment comparison” or “Measured batch results” section.

Include:

- How to interpret the batch comparison.
- Mention that the UI button runs deterministic headless replays in-browser.
- Add a small measured table from a local run. If exact values differ after implementation, generate them using the code and paste stable rounded values.
- Include limitations: sample-based stochastic optimizer with fixed seeds, simplified human prediction, simplified belief update, not a benchmark suite.

### 5. Verification commands

After implementation, run:

```bash
npm run lint
npm test
npm run build
```

Also start dev server and visually verify:

```bash
npm run dev -- --host 127.0.0.1
```

Check the browser console and confirm the comparison button and result table render.

## Acceptance criteria

- `runScenarioBatch` exists and is exported from `src/simulation/experimentRunner.ts`.
- Batch summaries are deterministic and contain all core metrics.
- Browser UI has a `Run comparison` button and a result table.
- README documents the batch experiment and includes measured results.
- `npm run lint`, `npm test`, and `npm run build` pass.
- No commits are made by Codex.
