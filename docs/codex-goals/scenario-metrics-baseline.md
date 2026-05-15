# Scenario Metrics and Baseline Comparison Codex Goal

## Goal

Advance the socially-aware wall-following belief-space MPC simulator from a single visual demo into a reproducible experiment workbench.

Implement:

1. deterministic scenario selection,
2. experiment/evaluation metrics,
3. a small baseline-policy comparison lane,
4. clearer documentation of how to use these pieces.

The point is to make the simulator useful for comparing behavior, not just watching one default run.

## Repository context

This is a Vite + React + TypeScript browser simulator in `/home/openclaw/social-wall-mpc-sim`.

Current architecture:

```text
src/
  App.tsx
  belief/
  planning/
  rendering/
  simulation/
  ui/
```

Important existing design principles:

- Keep algorithm/simulation code independent from rendering.
- Keep deterministic behavior when a fixed seed is used.
- Keep cost terms inspectable.
- Use TypeScript types clearly.
- Keep dependencies minimal.
- Follow existing Biome-compatible formatting.
- Do not add a backend.
- Do not implement ROS, production SLAM, global planning, or a physics engine.

## Required feature 1: deterministic scenario selection

Add a scenario model and selector.

Suggested module:

```text
src/simulation/scenarios.ts
```

Define a `ScenarioDefinition` type with at least:

- `id`
- `name`
- `description`
- `seed`
- `initialRobot`
- `humans`
- `environment`
- `initialBelief` or a function to construct it from the environment

Add a function such as:

```ts
export function createSimulationStateForScenario(scenarioId: ScenarioId): SimulationState
export const scenarioDefinitions: ScenarioDefinition[]
```

Scenarios to include:

1. `crossing-human` — close to the current default: robot follows a wall while a human crosses near the wall.
2. `standing-human` — a human stands near the wall and the robot should slow/yield/deviate.
3. `head-on-human` — a human moves toward the robot along the wall corridor.
4. `blocked-corridor` — a human or obstacle makes stopping the best option for a while.
5. `partial-map-bend` — a bent/alcove wall layout where estimated wall coverage matters.
6. `multi-human` — two or more humans create competing social costs.

Do not hard-code scenario logic in rendering. The scenario should just instantiate state/environment/humans/belief.

UI requirement:

- Add a `Scenario` select control.
- Changing scenario resets the simulation to that scenario.
- Show a short scenario description in the UI.

## Required feature 2: experiment/evaluation metrics

Add experiment metrics that update with simulation steps and can be displayed.

Suggested module:

```text
src/simulation/metrics.ts
```

Suggested type:

```ts
export type SimulationMetrics = {
  elapsedTime: number
  meanWallDistanceError: number
  maxWallDistanceError: number
  minHumanDistance: number
  socialViolationCount: number
  nearCollisionCount: number
  stopDuration: number
  progressAlongWall: number
  uncertaintyTrace: number
  estimatedMapCoverage: number
  selectedCost: number
}
```

You may adjust names if clearer, but include the same ideas.

Update `SimulationState` to carry metrics, or compute metrics from a history object. Keep it simple.

Metrics semantics:

- wall distance error: distance to selected/nearest wall minus `dWallTarget`.
- min human distance: minimum robot-human center distance observed so far.
- social violation: count frames/steps where robot-human distance is below `dMin`.
- near collision: count frames/steps where robot is too close to walls or static obstacles.
- stop duration: accumulated time where selected/applied `v` is near zero, e.g. `< 0.05`.
- progress along wall: accumulated forward movement along the local wall tangent or corridor direction.
- uncertainty trace: current `sigmaX + sigmaY + sigmaTheta`.
- estimated map coverage: fraction of true wall length covered by estimated wall segments.
- selectedCost: current selected trajectory total cost.

UI requirement:

- Add a metrics panel, or extend the existing cost panel if cleaner.
- Metrics must be visible and formatted compactly.

## Required feature 3: baseline-policy comparison lane

Add a planner/policy mode selector:

- `belief-mpc` — current sampling MPC implementation.
- `wall-only` — a simple wall follower that ignores humans and uncertainty.
- `reactive-stop` — wall follower plus a deliberately hard if/else style stop when any human is within `dMin` or a small buffer.

This is for comparison, not as the proposed method.

Suggested modules:

```text
src/planning/policies.ts
```

or similar.

Requirements:

- Keep existing MPC as default.
- The selected policy should produce the same `PlanningResult`-shaped data where practical so rendering and cost panels do not break.
- For baseline policies, candidate trajectories may be empty or contain one rollout, but selected control/trajectory should be visible if possible.
- Clearly label in the UI that `reactive-stop` is a hard-switch baseline.
- The baseline code should be small and readable.

Behavior expectations:

- `wall-only` should visibly ignore humans and therefore may violate social distance.
- `reactive-stop` should stop abruptly when a human is too close.
- `belief-mpc` should still optimize all costs together.

## Required feature 4: tests

Use TDD. Add or update tests before implementation when possible.

Add focused tests for:

- scenario creation returns deterministic state for the same scenario id.
- all required scenarios exist and have humans/walls as appropriate.
- metrics update detects social violation and stop duration.
- estimated map coverage metric increases or is computed from estimated wall segments.
- policy mode selection preserves current MPC default.
- `reactive-stop` returns zero velocity when a human is too close.
- `wall-only` returns a nonzero forward velocity in the same setup, showing that it ignores humans.

Use existing Vitest setup. Do not use Jest-only flags such as `--runInBand`.

## Required feature 5: documentation

Update `README.md` with:

- how to run the simulator,
- scenario selector overview,
- metric definitions,
- policy/baseline comparison explanation,
- why `belief-mpc` is the proposed unified-cost formulation and why baselines are only comparison tools,
- known limitations.

Also mention that deterministic scenarios make repeated tuning/comparison possible.

## Verification commands

Run and leave the repository passing:

```bash
npm run lint
npm test
npm run build
```

If visual behavior changes significantly, start the dev server and inspect in browser if possible, but do not block on browser availability.

## Constraints

- Do not commit changes.
- Do not open a PR.
- Do not add heavyweight dependencies.
- Keep changes small enough to review.
- Prefer plain functions and immutable-ish state updates.
- Avoid magic numbers where parameters already exist.
- Preserve the existing simulator behavior as the default scenario and default planner mode.

## Final report expected from Codex

Report:

- files changed,
- tests added,
- verification commands run and results,
- any known limitations or follow-up suggestions.
