# Codex Goal: Introduce explicit Gaussian pose belief

## Goal

Move the simulator one concrete step closer to full POMDP/SLAM by introducing an explicit probabilistic pose belief representation and routing uncertainty metrics/costs through it.

This is the first implementation slice only. Do not attempt particle filters, full SLAM, POMCP, or human intent inference in this goal.

## Repository context

Repository: `/home/openclaw/social-wall-mpc-sim`
Stack: Vite + React + TypeScript + Canvas, Vitest, Biome.

Current architecture:

```text
src/
  belief/
    simpleBelief.ts
    wallMapBelief.ts
  planning/
    cost.ts
    rollout.ts
    samplingMpc.ts
  simulation/
    types.ts
    metrics.ts
    simulator.ts
  rendering/
  ui/
```

Current `BeliefState` in `src/simulation/types.ts` has scalar fields:

- `sigmaX`
- `sigmaY`
- `sigmaTheta`
- `mapConfidence`
- `wallBeliefs`
- `estimatedWalls`

The objective is to add an explicit Gaussian pose belief while keeping backward compatibility with the scalar sigma fields for existing UI/tests.

## Required behavior

### 1. Add typed Gaussian pose belief

In `src/simulation/types.ts`, add:

```ts
export type PoseCovariance = [[number, number, number], [number, number, number], [number, number, number]]

export type PoseGaussian = {
  mean: RobotState
  covariance: PoseCovariance
}
```

Then extend `BeliefState` with:

```ts
pose: PoseGaussian
```

Keep existing scalar sigma fields for compatibility. They should remain synchronized with the covariance diagonal where practical.

### 2. Add pose belief helper module

Create `src/belief/poseBelief.ts` with small, tested helpers:

- `diagonalPoseCovariance(sigmaX: number, sigmaY: number, sigmaTheta: number): PoseCovariance`
- `poseGaussianFromSigmas(mean: RobotState, sigmaX: number, sigmaY: number, sigmaTheta: number): PoseGaussian`
- `sigmasFromPoseGaussian(pose: PoseGaussian): { sigmaX: number; sigmaY: number; sigmaTheta: number }`
- `tracePoseCovariance(pose: PoseGaussian): number`
- `propagatePoseGaussian(pose: PoseGaussian, control: ControlInput, parameters: Pick<PlannerParameters, 'dt'>): PoseGaussian`
- `correctPoseGaussianWithObservation(pose: PoseGaussian, observationStrength: number): PoseGaussian`

Keep these helpers intentionally simple:

- Propagation should move `mean` using the same differential-drive equations as `stepRobot` or by calling `stepRobot`.
- Propagation should increase covariance diagonal by a deterministic amount based on `dt`, `abs(v)`, and `abs(omega)`.
- Correction should reduce covariance diagonal as `observationStrength` increases.
- Clamp covariance diagonal to reasonable minimums: e.g. `0.02`, `0.02`, `0.01`.
- It is acceptable for off-diagonal covariance terms to remain zero in this slice.

### 3. Update belief construction/update

Modify `src/belief/simpleBelief.ts` so `updateBelief` updates `belief.pose` through the new helpers.

Important:

- Existing scalar fields must continue to be returned.
- `traceSigma` should prefer `tracePoseCovariance(belief.pose)`.
- If needed for migration, add a helper that normalizes an older belief shape into one with `pose`; but all scenario initial beliefs should be updated to include `pose`.

### 4. Update initial beliefs/scenarios/tests

Find all places constructing `BeliefState` and update them to include `pose`.

Likely files:

- `src/simulation/scenarios.ts`
- test files under `src/**/*.spec.ts*`

Prefer helper constructors instead of duplicating covariance object literals everywhere.

### 5. TDD requirements

Follow strict TDD:

1. Add `src/belief/poseBelief.spec.ts` first.
2. Run the focused spec and observe a failure due to missing module/functions.
3. Implement minimal helper code.
4. Run the focused spec and make it pass.
5. Add/update integration tests around `updateBelief` / `traceSigma`.
6. Run full verification.

Required test coverage:

- `tracePoseCovariance` returns sum of covariance diagonal.
- `propagatePoseGaussian` moves the mean and increases covariance with motion.
- `correctPoseGaussianWithObservation` reduces covariance more for stronger observations and respects lower bounds.
- `updateBelief` returns scalar sigma fields matching the pose covariance diagonal.
- `traceSigma` reads from `belief.pose` rather than only scalar fields.

### 6. No scope creep

Do not add:

- particle filters
- FastSLAM
- EKF Jacobians
- POMCP
- new UI panels unless necessary
- new runtime dependencies
- backend code

### 7. Verification commands

Run before finishing:

```bash
npm test -- src/belief/poseBelief.spec.ts
npm test
npm run lint
npm run build
```

If Biome formatting fails, apply the minimal formatting fix.

## Expected outcome

After this goal:

- The simulator still behaves like the existing belief-MPC demo.
- The belief state now contains a real typed Gaussian pose belief.
- Existing sigma-based UI/costs remain compatible.
- Uncertainty cost/metrics can now be grounded in a covariance trace.
- This creates a clear next step toward EKF/particle localization and eventually full POMDP/SLAM.

## Constraints

- Do not commit.
- Do not open PRs.
- Leave changes for Hermes/orchestrator review.
- Keep implementation small and deterministic.
