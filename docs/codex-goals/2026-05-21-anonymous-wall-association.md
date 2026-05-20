# Codex Goal: Add anonymous wall-observation association metrics and an ambiguous-corridor scenario

## Context

Repository: `/home/openclaw/social-wall-mpc-sim`

This is a browser-based TypeScript/Vite/React simulator for socially-aware wall-following navigation. The research target is a unified belief/map/robot/human/control optimization, but the implementation is intentionally a lightweight belief-MPC approximation.

Current status:

- `PoseGaussian` and pose covariance helpers exist.
- Wall observations include deterministic noisy `range`/`bearing`, `sensorPose`, and Gaussian `wallObservationLikelihood()` / bounded `wallObservationAffinity()`.
- Wall visibility includes sampled field-of-view, range, and line-of-sight/occlusion checks.
- Inference-quality metrics exist: `posePositionError`, `poseHeadingError`, `poseNormalizedError`, `mapKnowledgeError`.
- Important remaining limitation: wall observations still use true `wallId` directly for map updates. This means there is no visible data-association challenge yet.

## Goal

Implement the next bounded roadmap slice toward SLAM/POMDP realism: make wall-observation association explicit and measurable, without replacing the current lightweight map representation.

Add an anonymous-association evaluation path that scores each wall observation against all candidate true walls using the existing range/bearing likelihood and reports how often the best likelihood recovers the observation's true `wallId`. This is an evaluation/diagnostic bridge, not a full SLAM data-association refactor.

Also add a deterministic ambiguous-corridor scenario where association is nontrivial enough to exercise the metric.

## Acceptance Criteria

### Product behavior

1. Add a metric named `wallAssociationAccuracy` or similarly clear.
   - It should be in `[0, 1]`.
   - It should be `1` when every current wall observation's best-likelihood candidate wall matches its true `wallId`.
   - It should be lower when at least one observation is more likely under a different wall.
   - If there are no observations, choose a stable documented value. Prefer `1` only if treating “no wrong associations” as vacuously correct, or `0` if treating it as “no evidence”; document the choice in README.

2. Add this metric to:
   - `SimulationMetrics` in `src/simulation/types.ts`.
   - `createInitialMetrics()` / `updateSimulationMetrics()` in `src/simulation/metrics.ts`.
   - The live Experiment metrics UI in `src/ui/CostPanel.tsx`.
   - `ScenarioExperimentSummary` in `src/simulation/experimentRunner.ts`.
   - Numeric finite metric assertions in `src/simulation/experimentRunner.spec.ts`.
   - README Metrics / Known limitations.

3. Add an ambiguous-corridor scenario to `src/simulation/scenarios.ts` and UI options.
   - Suggested id/name: `ambiguous-parallel-corridor` / `Ambiguous parallel corridor`.
   - Make two or more parallel/similar wall features such that range/bearing likelihood association can be nontrivial from some poses.
   - It does not need to solve full loop closure or anonymous SLAM; it just needs to expose the ambiguity and be deterministic.
   - Include this scenario in `ExperimentSummaryPanel` batch list if that list is intentionally focused on roadmap scenarios.
   - Add README scenario documentation.

### Implementation guidance

Preferred implementation:

- In `src/belief/wallMapBelief.ts`, add a helper such as:
  - `associateWallObservation(observation, robot, candidateWalls)` returning the best wall id plus likelihood/affinity.
  - `wallAssociationAccuracy(observations, robotOrObservationSensorPose, candidateWalls)` or similar.
- Use each observation's `sensorPose` when scoring; do not use the current robot pose if the observation already carries its producing pose.
- Reuse `wallObservationLikelihood()` / `wallObservationAffinity()`; do not invent a separate scoring model.
- Keep existing true-wall-id map update behavior intact for this slice. The goal is to expose/measure the dependency before removing it.

### TDD requirements

Start tests before implementation.

Add or update tests in `src/belief/wallMapBelief.spec.ts`:

1. A focused RED for the new association helper:
   - Given an observation from `front-wall` and candidate walls including a distant/wrong wall, the best association is `front-wall`.
2. A focused RED showing the metric drops below 1 when an observation's range/bearing is deliberately made more consistent with a different candidate wall than with its `wallId`.
3. A no-observation behavior test documenting the chosen stable value.

Add or update tests in `src/simulation/metrics.spec.ts`:

4. `wallAssociationAccuracy` is finite and bounded.
5. A crafted bad-observation state/metric case lowers association accuracy.

Add/update scenario/UI tests as appropriate:

6. `src/simulation/scenarios.spec.ts` should assert the new scenario exists and initializes with finite metrics.
7. `src/App.spec.tsx` should assert the new scenario option and metric label are rendered, if the existing test covers those lists.

Run focused tests after adding REDs to confirm they fail for missing code, then implement the minimal code.

### Constraints

- Do not implement full EKF-SLAM, particle SLAM, anonymous feature-map replacement, POMCP, or human belief in this slice.
- Do not remove or break existing `wallId` fields or map update behavior.
- Do not add dependencies.
- Do not add console logs, scratch files, or generated artifacts.
- Do not commit; leave changes for the orchestrating Hermes agent to inspect, test, commit, push, and verify.

### Verification commands

Run at least:

```bash
npm test -- src/belief/wallMapBelief.spec.ts
npm test -- src/simulation/metrics.spec.ts
npm test -- src/simulation/scenarios.spec.ts
npm test -- src/App.spec.tsx
npm test
npm run lint
npm run build
```

## Final response expected from Codex

Summarize:

- Tests added/updated and whether RED/GREEN was followed.
- Files changed.
- Verification commands run and their result.
- Remaining roadmap items intentionally not implemented.
