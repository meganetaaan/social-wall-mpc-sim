# Codex Goal: Add wall observation likelihood `p(z | x, m)`

## Goal

Implement the next small probabilistic slice: evaluate a noisy wall range/bearing observation against a candidate robot pose and map wall as a likelihood score.

This follows the previous goals:

1. Gaussian pose belief exists.
2. Wall observations now carry deterministic noisy `range`/`bearing` measurements.
3. Now add a pure likelihood helper `p(z | x, m)` and use it lightly in observation confidence/map update.
4. Do **not** implement anonymous data association, EKF, particle filtering, or full SLAM in this slice.

## Repository context

Repository: `/home/openclaw/social-wall-mpc-sim`
Stack: Vite + React + TypeScript + Canvas, Vitest, Biome.

Relevant files:

- `src/simulation/types.ts`
  - `WallObservation` has `wallId`, interval, confidence, strength, noisy `range`/`bearing`, stddevs, `rayTarget`.
  - `EstimatedWallSegment` stores observed wall interval and confidence for a known wall id.
- `src/belief/wallMapBelief.ts`
  - `observeWalls` returns noisy measurements from visible true-wall samples.
  - `updateEstimatedWallBelief` merges observed intervals and confidence.
- `src/belief/wallMapBelief.spec.ts`
  - existing observation tests.
- `src/simulation/math.ts`
  - geometry helpers: `nearestPointOnSegment`, `distance`, `normAngle`, `clamp`.
- `README.md`
  - update docs.

## Required behavior

### 1. Add likelihood helpers

Add pure exported helpers, preferably in `src/belief/wallMapBelief.ts` unless a focused module is cleaner:

```ts
export type ObservationLikelihood = {
  rangeResidual: number
  bearingResidual: number
  rangeSigma: number
  bearingSigma: number
  likelihood: number
  logLikelihood: number
}

export function expectedRangeBearingToWall(robot: RobotState, wall: WallSegment): { range: number; bearing: number; point: Vec2 }

export function wallObservationLikelihood(
  observation: Pick<WallObservation, 'range' | 'bearing' | 'rangeStdDev' | 'bearingStdDev'>,
  robot: RobotState,
  wall: WallSegment,
): ObservationLikelihood
```

Semantics:

- `expectedRangeBearingToWall` uses the nearest point on the candidate wall from the robot pose.
- Expected bearing is relative to robot heading and normalized to `[-pi, pi]`.
- Residuals:
  - `rangeResidual = observed range - expected range`
  - `bearingResidual = normAngle(observed bearing - expected bearing)`
- Use independent Gaussian log likelihood over range and bearing:

```text
log p = -0.5 * ((dr/sigma_r)^2 + (db/sigma_b)^2)
        - log(2*pi*sigma_r*sigma_b)
```

- Use safe positive sigmas:
  - `rangeSigma = max(observation.rangeStdDev, 1e-6)`
  - `bearingSigma = max(observation.bearingStdDev, 1e-6)`
- `likelihood = exp(logLikelihood)` may be very small but should remain finite for normal inputs.

### 2. Add a normalized affinity helper for confidence updates

Raw Gaussian PDF can exceed 1 for small sigmas, so add a bounded helper:

```ts
export function wallObservationAffinity(...): number
```

Semantics:

- Returns `exp(-0.5 * mahalanobisSquared)` or equivalent.
- Range is `[0, 1]`.
- Equals near `1` for a measurement matching the candidate wall.
- Falls toward `0` for strongly inconsistent range/bearing.

### 3. Use affinity in map-belief update, but keep behavior stable

In `updateEstimatedWallBelief`, use the affinity for the observed wall id and the current true wall segment to slightly gate confidence growth.

Important:

- Existing tests should remain semantically true.
- Do not require anonymous data association. The observation still has a `wallId`.
- The `tMin/tMax` coverage merge should continue as before.
- Confidence should increase for self-consistent observations.
- A grossly inconsistent observation for the same wall should not raise confidence much.

A suggested formula:

```ts
const wall = environment.walls.find((candidate) => candidate.id === estimated.wallId)
const affinity = wall ? wallObservationAffinity(observation, observationPoseOrRayTarget?, wall) : 1
const effectiveConfidence = observation.confidence * (0.35 + 0.65 * affinity)
const effectiveStrength = observation.strength * (0.35 + 0.65 * affinity)
```

But note `updateEstimatedWallBelief` currently does not receive the robot pose. Prefer the cleaner approach below:

- Extend `WallObservation` with optional or required `sensorPose: RobotState` captured in `observeWalls`.
- Use `observation.sensorPose` in likelihood evaluation.
- Update test fixtures that directly construct `WallObservation`.

If adding `sensorPose` is too invasive, add a separate update helper that accepts robot pose. Prefer the `sensorPose` field because it makes each observation self-contained.

### 4. TDD requirements

Use strict TDD inside this Codex goal:

1. Add failing tests first.
2. Run the focused test and observe failure.
3. Implement minimal code.
4. Run focused and full verification.

Required tests in `src/belief/wallMapBelief.spec.ts`:

1. `expectedRangeBearingToWall` returns nearest-wall range and relative bearing.
2. `wallObservationLikelihood` gives a higher log-likelihood for a matching wall pose than for a wrong/distant wall pose.
3. `wallObservationAffinity` is bounded in `[0, 1]`, near `1` for matching measurement, and much lower for an inconsistent measurement.
4. `observeWalls` includes `sensorPose` matching the robot pose used for observation.
5. `updateEstimatedWallBelief` uses likelihood/affinity to reduce confidence growth for a deliberately inconsistent observation while still merging coverage.

Use real geometry and real helpers, not mocks.

### 5. Documentation

Update README:

- Mention that observations can now be evaluated with a Gaussian `p(z | x, m)` likelihood.
- Limitations should still state that data association is by known `wallId`; no anonymous feature map / EKF / particle SLAM yet.

### 6. No scope creep

Do not add:

- anonymous feature-map data association;
- estimated feature map independent from true wall IDs;
- EKF, particle filter, Graph-SLAM, FastSLAM;
- observation branching / belief rollout;
- new dependencies;
- planner retuning beyond preserving existing tests;
- UI redesign.

## Verification commands

Run before finishing:

```bash
npm test -- src/belief/wallMapBelief.spec.ts
npm test
npm run lint
npm run build
```

If Biome formatting fails, apply the minimal formatting fix.

## Expected outcome

After this goal:

- A noisy observation can be scored against a candidate pose and wall as `p(z | x, m)`.
- A bounded affinity derived from that likelihood gates wall confidence updates.
- Observations remain deterministic and self-contained.
- Existing simulator behavior remains stable.
- The next natural slice becomes: true-wall-id-independent estimated feature map / data association.

## Constraints for Codex

- Do not commit.
- Do not open PRs.
- Leave changes for Hermes/orchestrator review.
- Keep implementation small and deterministic.
