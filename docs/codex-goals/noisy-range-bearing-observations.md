# Codex Goal: Add deterministic noisy range/bearing wall observations

## Goal

Move the simulator one step closer to a probabilistic observation model by upgrading wall observations from purely geometric sample intervals to explicit range/bearing measurements with deterministic pseudo-noise.

This is the recommended next slice after Gaussian pose belief and line-of-sight occlusion:

1. Gaussian pose belief is already present.
2. Line-of-sight wall visibility is already present.
3. Now observations should carry noisy sensor measurements (`range`, `bearing`) rather than only exact true-map sample positions.
4. A later goal can use these fields for `p(z | x, m)` likelihood and feature association.

Keep this slice small: add measurement fields and deterministic noise, wire them into confidence/strength, and document it. Do not implement a full likelihood model yet.

## Repository context

Repository: `/home/openclaw/social-wall-mpc-sim`
Stack: Vite + React + TypeScript + Canvas, Vitest, Biome.

Relevant files:

- `src/simulation/types.ts`
  - `WallObservation` currently has `wallId`, interval, confidence, strength, and `rayTarget`.
- `src/belief/wallMapBelief.ts`
  - `observeWalls` samples true wall points, checks range/FOV/line-of-sight, and merges visible samples into one observation per wall.
- `src/belief/wallMapBelief.spec.ts`
  - existing observation model tests, including occlusion regressions.
- `src/simulation/math.ts`
  - deterministic helpers and `mulberry32` PRNG already exist.
- `src/rendering/draw.ts`
  - observation rays/highlights are rendered from `rayTarget`; keep compatibility.
- `README.md`
  - update map-belief docs and limitations.

## Required behavior

### 1. Extend observation types

Extend `WallObservation` in `src/simulation/types.ts` with explicit measurement fields:

```ts
range: number
bearing: number
rangeStdDev: number
bearingStdDev: number
```

Semantics:

- `range`: noisy measured range from robot pose to the representative observed wall point.
- `bearing`: noisy measured bearing relative to robot heading, normalized to `[-pi, pi]`.
- `rangeStdDev`: deterministic parameter-derived sensor noise scale in meters.
- `bearingStdDev`: deterministic parameter-derived sensor noise scale in radians.

Keep existing `rayTarget` and interval fields for rendering/planning compatibility.

### 2. Add deterministic pseudo-noise helpers

Add small helpers in `wallMapBelief.ts` or a focused helper module. Suggested names:

- `observationNoiseScales(parameters)`
- `noisyRangeBearingMeasurement(robot, point, parameters, key)`

The noise must be deterministic for the same robot/point/wall/sample inputs. Do **not** use `Math.random()`.

Acceptable implementation options:

- Use a tiny deterministic hash/sine noise helper based on robot pose, point, and wall/sample key.
- Or use `mulberry32` with a stable integer seed derived from those values.

Noise requirements:

- `rangeStdDev` should be small relative to `sensorRadius`, for example `max(0.015, sensorRadius * 0.015)`.
- `bearingStdDev` should be small relative to `sensorFov`, for example `max(0.005, sensorFov * 0.01)`.
- Noise should be bounded or effectively bounded so tests are stable; e.g. within `±2 * stdDev`.
- `range` should never be negative.
- `bearing` should be normalized with `normAngle`.

### 3. Wire measurement into `observeWalls`

For each wall observation, compute the measurement from the same representative point used for `rayTarget`.

Suggested flow:

1. Continue gathering visible samples exactly as now.
2. Choose `rayTarget` as now (`nearest` when visible, otherwise middle visible sample point).
3. Compute noisy range/bearing for that representative target.
4. Include measurement fields in the returned `WallObservation`.

Confidence/strength should still be based on geometric visibility, but it should lightly account for noise quality. For this slice, a simple deterministic factor is enough:

- lower noise relative to range/FOV => confidence remains similar;
- do not make behavior wildly different or break existing scenario tests.

### 4. Tests first (strict TDD)

Add/update tests before production code.

Required tests in `src/belief/wallMapBelief.spec.ts`:

1. Existing observation includes measurement fields:
   - `range` is close to the true representative distance, within a small bound such as `2 * rangeStdDev + 1e-6`.
   - `bearing` is close to the true relative bearing, within `2 * bearingStdDev + 1e-6`.
   - std dev fields are positive.

2. Measurements are deterministic:
   - calling `observeWalls` twice with the same robot/environment/parameters returns equal `range` and `bearing` for the same wall.

3. Bearing is normalized:
   - use a robot heading / wall point configuration near the ±π wrap boundary and assert `bearing >= -Math.PI && bearing <= Math.PI`.

Update any existing tests/fixtures that construct `WallObservation` directly (likely rendering tests) to include the new fields.

### 5. Documentation

Update README:

- In “Map belief and observations”, mention that visible wall samples produce deterministic noisy range/bearing measurements.
- In “Known limitations”, clarify that the model still does not perform likelihood-based data association or continuous ray-cast SLAM.

### 6. No scope creep

Do not add:

- observation likelihood `p(z | x, m)` yet;
- data association against anonymous estimated features;
- EKF/FastSLAM/Graph-SLAM;
- particle filters;
- POMCP;
- new runtime dependencies;
- major UI redesign;
- backend code;
- broad planner retuning unless tests require it.

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

- `WallObservation` represents both visible wall intervals and explicit noisy sensor measurements.
- Observations remain deterministic for fixed state/parameters.
- Existing rendering and planning continue to work.
- This creates the direct foundation for the next recommended slice: observation likelihood `p(z | x, m)`.

## Constraints

- Do not commit.
- Do not open PRs.
- Leave changes for Hermes/orchestrator review.
- Keep implementation small and deterministic.
