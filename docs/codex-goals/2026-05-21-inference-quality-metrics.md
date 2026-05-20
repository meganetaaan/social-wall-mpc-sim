# Codex Goal: Add inference-quality metrics for the belief-SLAM approximation

## Context

Repository: `/home/openclaw/social-wall-mpc-sim`

This browser simulator is a TypeScript/Vite/React prototype for socially-aware wall-following navigation. It is intentionally a belief-MPC approximation, not full POMDP/SLAM.

A prior roadmap message identified these gaps:

1. Belief was not a true probability distribution.
2. Observation model was not likelihood-based.
3. Map belief still uses true wall ids.
4. Human state is not a belief.
5. Planner is sampled MPC, not a POMDP planner.
6. Metrics do not measure inference quality.
7. More difficult scenarios are needed.

Current implementation status as of this goal:

- `PoseGaussian` exists in `src/simulation/types.ts`.
- `tracePoseCovariance()`, `propagatePoseGaussian()`, and `correctPoseGaussianWithObservation()` exist in `src/belief/poseBelief.ts`.
- `BeliefState.pose` exists and legacy scalar sigmas are kept for compatibility.
- Wall observations include deterministic noisy range/bearing measurements and `wallObservationLikelihood()`.
- Line-of-sight/occlusion sampling exists in `src/belief/wallMapBelief.ts`.
- Metrics currently include social/goal/wall/map-coverage readouts but not pose/map inference-quality errors.

## Goal

Implement the next bounded roadmap slice: add inference-quality metrics that make it explicit how well the current approximation estimates pose and map state.

This should remain a lightweight approximation. Do not attempt full FastSLAM, POMCP, data association without ids, or human-belief inference in this goal.

## Acceptance Criteria

### Product behavior

Add metrics that are visible in the live UI, included in headless experiment summaries, and documented in README:

- `posePositionError`: Euclidean distance between the true robot pose and `belief.pose.mean`.
- `poseHeadingError`: absolute wrapped heading error between true robot pose and `belief.pose.mean`.
- `poseNormalizedError`: a simple diagonal-covariance normalized error, roughly `sqrt(dx^2/varX + dy^2/varY + dtheta^2/varTheta)` with safe lower bounds.
- `mapKnowledgeError`: a true-map-backed score in `[0, 1]` that is high when estimated wall coverage/confidence is poor and low when known wall length is covered with high confidence.

Use names that are clear in TypeScript and UI. If you choose slightly different names, keep the same meaning.

### Implementation shape

Prefer these files:

- Modify `src/simulation/types.ts` to extend `SimulationMetrics`.
- Modify `src/simulation/metrics.ts` to compute the new metrics from `robot`, `environment`, and `belief`.
- Modify `src/ui/CostPanel.tsx` to display the new metrics in the Experiment metrics panel.
- Modify `src/simulation/experimentRunner.ts` so summaries include the new metrics.
- Modify `src/simulation/experimentRunner.spec.ts` and `src/simulation/metrics.spec.ts` with TDD-style tests for the new behavior.
- Modify `src/App.spec.tsx` only if UI assertions need to cover the new labels.
- Modify `README.md` to document these metrics and the current position against the roadmap.

### TDD requirements

Start with tests before implementation:

1. Add/adjust focused tests in `src/simulation/metrics.spec.ts` proving:
   - position and heading errors are zero or near-zero when belief mean equals robot pose;
   - position/heading errors are positive when belief mean differs;
   - normalized error increases for the same pose error under smaller covariance;
   - map knowledge error decreases when estimated wall coverage/confidence improves.
2. Run the focused test and observe it fail for the missing fields/functions.
3. Implement minimal code to pass.
4. Add/adjust `src/simulation/experimentRunner.spec.ts` so summaries expose finite new metrics.
5. Add a small UI/readme assertion if appropriate.

### Constraints

- Keep the existing public API and scenarios working.
- Keep legacy `sigmaX`, `sigmaY`, `sigmaTheta` compatibility.
- Do not remove existing metrics.
- Do not add dependencies.
- Do not add temporary console logs or scratch files.
- Do not commit; leave changes for the orchestrating Hermes agent to inspect, test, commit, push, and verify.

### Verification commands

Run these before finishing:

```bash
npm test -- src/simulation/metrics.spec.ts
npm test -- src/simulation/experimentRunner.spec.ts
npm test
npm run lint
npm run build
```

If a deterministic experiment threshold changes, explain why in the final summary and keep the threshold product-faithful rather than simply weakening it.

## Final response expected from Codex

Summarize:

- Tests added/updated.
- Files changed.
- Verification commands run and their result.
- Any remaining roadmap items intentionally not implemented in this slice.
