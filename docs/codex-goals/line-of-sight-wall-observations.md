# Codex Goal: Add line-of-sight occlusion to wall observations

## Goal

Make the wall observation model physically plausible when multiple walls overlap in the robot's field of view: nearer wall segments must occlude farther wall samples. This is the next recommended implementation slice because it fixes a visible map-belief bug and makes the current belief-space approximation more consistent before adding heavier SLAM/POMDP machinery.

## Repository context

Repository: `/home/openclaw/social-wall-mpc-sim`
Stack: Vite + React + TypeScript + Canvas, Vitest, Biome.

Relevant files:

- `src/belief/wallMapBelief.ts` contains `observeWalls`, which currently tests only sensor range and FOV.
- `src/belief/wallMapBelief.spec.ts` contains observation-model tests.
- `src/simulation/math.ts` contains geometry helpers.
- `src/rendering/draw.ts` renders current observations and estimated wall coverage; rendering should not fake occlusion if model data is wrong.
- `README.md` currently says wall visibility ignores occlusion; update this once the model has line-of-sight occlusion.

Root-cause hypothesis already established by orchestrator inspection:

- `observeWalls` samples points on every wall and accepts each sample when it is within `sensorRadius` and `sensorFov`.
- It never checks whether the ray from robot pose to that sample intersects a nearer wall segment.
- Therefore a far wall behind a closer wall is incorrectly observed and rendered as visible/estimated.

## Required behavior

### 1. Add geometric line-of-sight occlusion

In `observeWalls`, a sampled wall point is observable only if:

1. it is inside sensor range;
2. it is inside the robot FOV; and
3. the open ray segment from the robot position to the sampled point is not blocked by a different wall segment at a smaller distance.

Important edge cases:

- A wall must not self-occlude its own sample.
- Endpoint/touching cases should be tolerant: observing a point on a connected wall joint should not be rejected merely because the ray touches the same physical corner.
- A blocker should count only when its intersection lies meaningfully between the robot and sample, not at the robot origin or exactly at the target point.
- Keep this deterministic and dependency-free.

Implementation suggestion:

- Add small helper functions in `src/belief/wallMapBelief.ts` or `src/simulation/math.ts`, such as segment/ray intersection with epsilons.
- Keep helpers unit-tested through behavior tests; exporting a tiny helper is acceptable if that makes focused tests clearer, but prefer testing through `observeWalls` if possible.

### 2. Regression tests first (strict TDD)

Add/update tests in `src/belief/wallMapBelief.spec.ts` before production changes.

Required RED test:

- Environment has two parallel vertical walls in front of the robot, for example:
  - near wall: `x = 1`, `y = -1..1`
  - far wall: `x = 2`, `y = -1..1`
  - robot at `(0, 0)` facing `+x`, with enough range/FOV to include both by range+bearing.
- `observeWalls` must observe the near wall.
- `observeWalls` must not observe the far wall because the near wall blocks line of sight.
- Before implementation, this test should fail because the current model observes both.

Recommended second test:

- A side-by-side or offset far wall sample that is not behind the blocker remains observable when the ray does not cross the near wall.
- This prevents over-broad “nearest wall only” hacks.

### 3. Update documentation

Update README sections:

- In “Map belief and observations”, mention line-of-sight occlusion in addition to range/FOV.
- In “Known limitations”, remove or revise “Wall visibility ignores occlusion…” so it no longer claims the fixed behavior is missing. It may still mention sampled wall points/geometric clipping limitations.

### 4. No scope creep

Do not add:

- particle filters
- EKF/FastSLAM/Graph-SLAM
- POMCP
- new runtime dependencies
- major UI redesign
- backend code
- broad planner retuning unless tests require it

This goal is a focused model correctness fix plus documentation.

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

- Far walls hidden behind nearer walls are not included in current wall observations.
- Estimated wall coverage no longer appears to penetrate through stacked/overlapping walls.
- Visible but non-occluded walls remain observable.
- The README accurately describes the upgraded sampled line-of-sight observation model.

## Constraints

- Do not commit.
- Do not open PRs.
- Leave changes for Hermes/orchestrator review.
- Keep implementation small and deterministic.
