# Codex Goal: Ideal-form socially-aware belief-space wall navigation

## Context

Repository: `/home/openclaw/social-wall-mpc-sim`
Stack: Vite + React + TypeScript + Canvas, minimal dependencies, Biome, Vitest.
Current prototype already has:

- Sampling MPC over robot controls.
- Static true walls and moving humans.
- Pose covariance-style belief.
- Per-wall confidence values in `BeliefState.wallBeliefs`.
- Cost terms for wall following, humans, collision, control, smoothness, progress, pose uncertainty, map uncertainty, observation gain, and wall-belief consistency.
- Canvas visualization of true walls, cyan map-belief overlay, candidate trajectories, selected trajectory, trace, humans/social zones, and cost panel.

## Ideal direction

Move the simulator closer to a unified belief-space navigation formulation where map estimation, localization confidence, social navigation, and motion planning are one receding-horizon optimization problem — not separate SLAM/global/local modules.

The next implementation slice should make the map belief more concrete and inspectable:

1. Separate **true wall map** from **estimated wall map**.
2. Simulate range observations from the robot to nearby wall segments.
3. Update estimated wall segment coverage/confidence from those observations.
4. Use estimated wall belief in planning/costs, while retaining true walls for collision/sensor simulation.
5. Visualize sensor rays, observed wall segments, estimated wall coverage, and remaining uncertainty.
6. Add tests that prove the algorithmic behavior, not just rendering.

## Required implementation

### 1. Types and belief model

Extend types cleanly without overcomplicating:

- Add an `EstimatedWallSegment` or equivalent that connects to a true wall id and records:
  - `wallId`
  - observed interval or coverage amount along the wall segment, e.g. `[tMin, tMax]` in segment parameter space, or multiple intervals if you choose.
  - `confidence`
  - `lastObservedAt`
- Keep the implementation simple. One merged interval per wall is acceptable for this slice.
- Add helper functions in `src/belief/simpleBelief.ts` or a new `src/belief/wallMapBelief.ts`:
  - initialize estimated wall belief from environment and current belief.
  - update a wall belief from observations.
  - compute observed coverage ratio for a wall.
  - compute map uncertainty from uncovered/low-confidence wall intervals.

### 2. Observation model

Add a simple deterministic range-observation approximation:

- Robot has a limited sensor radius and field of view.
- A wall segment is observable if it is within range and roughly inside FOV.
- Observation should produce the visible interval on the wall in a simplified way. It does not need ray casting against occlusion yet.
- Observation output should include wall id, confidence/strength, and observed interval.
- Add clear tests for:
  - nearby wall in FOV produces an observation.
  - wall behind robot or out of range does not produce an observation.
  - repeated observation expands/raises estimated coverage/confidence.

### 3. Planning/cost integration

Update planning cost to use the estimated wall belief, not only true wall geometry:

- Wall-following may still use the nearest true wall as a fallback, but add a clear cost term that prefers trajectories where the nearest/followed wall has high estimated coverage/confidence.
- Map uncertainty cost should increase when the followed wall or nearby walls have low coverage/confidence.
- Observation gain should reward controls/trajectories expected to observe uncertain wall intervals.
- Keep everything inside the unified stage cost. Do not add hand-coded navigation modes.

### 4. Rendering/UI

Visualize the belief-space idea clearly:

- True walls: dim gray base layer.
- Estimated/observed wall coverage: bright cyan overlay only on observed intervals, not the full wall.
- Sensor range/FOV: subtle cone or rays from the robot.
- Current observations: short brighter rays or wall highlights.
- Cost panel should still show the unified cost terms, including map uncertainty / observation gain / wall belief consistency.
- Add a compact legend or labels if needed.

### 5. Tests and docs

Follow TDD as much as practical:

- Add focused Vitest specs for observation generation and wall coverage update before implementation.
- Update existing tests for cost term names if needed.
- Add/adjust rendering tests for estimated wall coverage / sensor FOV visual markers.
- Update README to explain:
  - true map vs estimated map.
  - observation model.
  - how map belief affects the MPC objective.
  - limitations.

## Constraints

- Keep dependencies minimal; do not add a physics engine or backend.
- Preserve deterministic simulation for fixed seed.
- Keep algorithm independent from rendering.
- Keep cost terms modular and inspectable.
- Use Biome-compatible formatting.
- Do not commit automatically; leave changes for Hermes/orchestrator review and commit.
- Do not open PRs or publish anywhere.

## Verification commands

Run before finishing:

```bash
npm run lint
npm test
npm run build
```

If a browser dev server is already running, do not kill unrelated processes. The orchestrator will do final visual verification.

## Expected outcome

A user opening the browser should be able to see:

- the robot following a wall while considering humans,
- a dim true wall map,
- only observed portions of walls appearing as the estimated map,
- sensor field/rays,
- map uncertainty and observation gain changing in the cost panel,
- behavior still emerging from one unified sampling-MPC cost.
