# POMDP/SLAM Alignment Implementation Plan

> **For Hermes:** Use subagent-driven-development skill to implement this plan task-by-task.

**Goal:** Evolve `social-wall-mpc-sim` from a belief-MPC demo into a more principled Ueda-style SLAM/POMDP approximation where mapping, localization, object perception, value estimation, and control remain parts of one belief-state decision loop.

**Architecture:** Keep the current browser simulator and sampling MPC working, but refactor the internals around explicit belief transitions: `predictBelief`, `observeWorld`, `updateBeliefWithObservation`, and `evaluateBeliefAction`. Replace hidden-oracle dependencies step by step: first anonymous observation association, then local estimated map features, then expected value over pose/map/object belief. Preserve deterministic scenarios, inspectable visualizations, and full `npm test` / `npm run lint` / `npm run build` gates after every slice.

**Tech Stack:** TypeScript, Vite/React, Vitest, Canvas rendering, deterministic seeded simulation, existing `src/belief`, `src/planning`, `src/simulation`, and `src/rendering` modules.

---

## Current baseline

As of commit `eab0e09 Add point-derived object belief for social cost`:

- Spiral unknown map can reach the goal using a value field derived from observed wall belief, not hidden ground-truth walls.
- `routeWaypoints` has been removed from the planning path.
- Human/social cost no longer reads true `HumanState[]` directly for the `human` term; it reads `belief.objectBeliefs` and `pHuman`.
- Synthetic point returns exist, but point observation generation still uses true world geometry internally, which is acceptable as the simulator sensor model.
- Map belief still depends on true wall IDs through `WallObservation.wallId` and `EstimatedWallSegment.wallId`.
- Rollouts still use one mostly deterministic future belief path, so this is not yet a POMDP planner.

## Design north star

Ueda-style interpretation for this simulator:

- Avoid “global route first, local avoidance second.” Use state/belief value fields inside one objective.
- Treat exploration as a means for control, not a separate behavior module.
- Avoid hidden-oracle semantics: the planner should not know true wall IDs, true human labels, or future observations.
- Move toward the POMDP tuple incrementally:
  - state: true simulator world, used only by dynamics/sensor generation/evaluation
  - action: robot control input
  - observation: anonymous point/range returns and later feature observations
  - transition: robot/object/map belief prediction
  - reward/cost: unified wall/social/control/information/value objective over belief

---

## Phase 1: Split true sensor generation from planner-facing anonymous observations

### Task 1: Introduce a unified observation envelope

**Objective:** Make wall and object observations flow through a common anonymous sensor-observation type before belief updates.

**Files:**
- Modify: `src/simulation/types.ts`
- Create: `src/belief/observations.ts`
- Test: `src/belief/observations.spec.ts`

**Step 1: Write failing test**

Add tests proving the planner-facing observation type does not expose `wallId`, `humanId`, `class`, or `kind`.

```ts
import { describe, expect, it } from 'vitest'
import type { PointObservation } from '../simulation/types'
import { plannerFacingObservationKeys } from './observations'

describe('planner-facing observations', () => {
  it('does not expose true feature or semantic labels', () => {
    const keys = plannerFacingObservationKeys()
    expect(keys).not.toContain('wallId')
    expect(keys).not.toContain('humanId')
    expect(keys).not.toContain('class')
    expect(keys).not.toContain('kind')
  })
})
```

**Step 2: Run RED**

```bash
npm test -- --run src/belief/observations.spec.ts
```

Expected: fail because `plannerFacingObservationKeys` does not exist.

**Step 3: Implement minimal code**

Create `src/belief/observations.ts`:

```ts
export type PlannerObservationKind = 'point-return' | 'range-bearing'

export type PlannerObservation = {
  id: string
  kind: PlannerObservationKind
  point: { x: number; y: number }
  range: number
  bearing: number
  time: number
  source: 'point-sensor'
}

export const plannerFacingObservationKeys = () => ['id', 'kind', 'point', 'range', 'bearing', 'time', 'source']
```

Then either alias or migrate `PointObservation` toward this shape without adding truth labels.

**Step 4: Verify**

```bash
npm test -- --run src/belief/observations.spec.ts src/belief/pointObjectBelief.spec.ts
npm run lint
```

**Step 5: Commit**

```bash
git add src/belief/observations.ts src/belief/observations.spec.ts src/simulation/types.ts
git commit -m "Introduce anonymous planner observations"
```

---

## Phase 2: Add anonymous wall association diagnostics before removing true wall IDs

### Task 2: Add wall observation likelihood / association scoring

**Objective:** Measure whether an anonymous observation would associate to the correct wall before changing the map update path.

**Files:**
- Modify: `src/belief/wallMapBelief.ts`
- Create: `src/belief/wallAssociation.ts`
- Test: `src/belief/wallAssociation.spec.ts`

**Step 1: Write failing tests**

Test cases:

1. A range/bearing observation near one wall associates to that wall.
2. A crafted ambiguous observation associates to the wrong parallel wall.
3. No-candidate behavior returns `null`.

```ts
it('selects the wall with the highest observation likelihood', () => {
  const observation = {
    point: { x: 2, y: 0.1 },
    range: 2,
    bearing: 0,
    sensorPose: { x: 0, y: 0, theta: 0 },
    time: 0,
  }
  const walls = [
    { id: 'near', a: { x: 0, y: 0 }, b: { x: 4, y: 0 } },
    { id: 'far', a: { x: 0, y: 2 }, b: { x: 4, y: 2 } },
  ]

  expect(associateWallObservation(observation, walls)?.wallId).toBe('near')
})
```

**Step 2: Run RED**

```bash
npm test -- --run src/belief/wallAssociation.spec.ts
```

**Step 3: Implement**

Create `associateWallObservation(observation, candidateWalls)` using distance from observed point to wall and bearing consistency from `observation.sensorPose`. Return:

```ts
type WallAssociation = {
  wallId: string
  score: number
  distanceResidual: number
  bearingResidual: number
}
```

**Step 4: Verify**

```bash
npm test -- --run src/belief/wallAssociation.spec.ts src/belief/wallMapBelief.spec.ts
npm test
npm run lint
npm run build
```

**Step 5: Commit**

```bash
git add src/belief/wallAssociation.ts src/belief/wallAssociation.spec.ts src/belief/wallMapBelief.ts
git commit -m "Add anonymous wall association diagnostics"
```

---

## Phase 3: Surface association quality as an inference metric

### Task 3: Add wall association accuracy metric

**Objective:** Make the current true-ID dependency measurable in live and headless runs.

**Files:**
- Modify: `src/simulation/metrics.ts`
- Modify: `src/simulation/types.ts`
- Modify: UI metric panel file, likely `src/ui/MetricsPanel.tsx` or `src/App.tsx`
- Test: `src/simulation/metrics.spec.ts`

**Step 1: Write failing tests**

- No observations returns `1` with documented semantics: “no wrong associations observed.”
- Clear association returns `1`.
- Deliberately ambiguous/wrong association returns `< 1`.

**Step 2: Implement**

Add `wallAssociationAccuracy` to `SimulationMetrics`. Compute by comparing diagnostic association result with the simulator-only true `wallId` on existing wall observations. This is an evaluation metric only; do not use it to update belief yet.

**Step 3: Verify**

```bash
npm test -- --run src/simulation/metrics.spec.ts src/simulation/experimentRunner.spec.ts
npm test
npm run lint
npm run build
```

**Step 4: Commit**

```bash
git add src/simulation/metrics.ts src/simulation/metrics.spec.ts src/simulation/types.ts src/ui src/App.tsx
git commit -m "Report wall association accuracy"
```

---

## Phase 4: Introduce local estimated map features independent of true wall IDs

### Task 4: Add estimated line-feature map type

**Objective:** Prepare map belief to represent features with local IDs instead of true wall IDs.

**Files:**
- Modify: `src/simulation/types.ts`
- Create: `src/belief/lineFeatureMap.ts`
- Test: `src/belief/lineFeatureMap.spec.ts`

**Step 1: Write failing tests**

Test that a local estimated feature can be created from a short observed segment without storing a true `wallId`.

```ts
expect(feature.id).toMatch(/^feature-/)
expect(Object.keys(feature)).not.toContain('wallId')
```

**Step 2: Implement type**

```ts
export type EstimatedLineFeature = {
  id: string
  a: Vec2
  b: Vec2
  confidence: number
  lastObservedAt: number
  observationCount: number
}
```

Add optional `estimatedFeatures?: EstimatedLineFeature[]` to `BeliefState` while keeping `estimatedWalls` during migration.

**Step 3: Verify**

```bash
npm test -- --run src/belief/lineFeatureMap.spec.ts
npm test
npm run lint
npm run build
```

**Step 4: Commit**

```bash
git add src/belief/lineFeatureMap.ts src/belief/lineFeatureMap.spec.ts src/simulation/types.ts
git commit -m "Add local estimated line features"
```

---

## Phase 5: Update map belief through association results, not true IDs

### Task 5: Add experimental anonymous map-update mode

**Objective:** Route wall map belief updates through association/feature creation while preserving the old true-ID update as a baseline.

**Files:**
- Modify: `src/belief/wallMapBelief.ts`
- Modify: `src/belief/simpleBelief.ts`
- Modify: `src/simulation/types.ts`
- Test: `src/belief/simpleBelief.spec.ts`
- Test: `src/belief/wallMapBelief.spec.ts`

**Step 1: Write failing test**

Given an anonymous observation with no matching local feature, the update creates a new `EstimatedLineFeature`. Given a second nearby observation, it updates the same local feature instead of creating another one.

**Step 2: Implement**

Add an option:

```ts
type MapUpdateMode = 'true-id-coverage' | 'anonymous-line-features'
```

Default to existing mode. Add a scenario or parameter switch for the experimental mode.

**Step 3: Verify**

```bash
npm test -- --run src/belief/simpleBelief.spec.ts src/belief/wallMapBelief.spec.ts
npm test
npm run lint
npm run build
```

**Step 4: Commit**

```bash
git add src/belief src/simulation/types.ts
git commit -m "Add anonymous line-feature map updates"
```

---

## Phase 6: Build value fields from estimated local features

### Task 6: Remove true-wall dependency from unknown-map value field

**Objective:** In anonymous-map mode, compute `environment.valueField` from local estimated line features instead of true `environment.walls` or true-ID `estimatedWalls`.

**Files:**
- Modify: `src/planning/beliefValueField.ts`
- Modify: `src/planning/valueField.ts` if helper extraction is needed
- Test: `src/planning/valueField.spec.ts`

**Step 1: Write failing test**

Create a belief with `estimatedFeatures` and no `estimatedWalls`. Expect `environmentWithBeliefValueField(...)` to produce a field whose reachable/blocked structure follows the local features.

**Step 2: Implement**

Add helper:

```ts
export function wallSegmentsFromEstimatedFeatures(features: EstimatedLineFeature[]): WallSegment[]
```

Prefer `estimatedFeatures` when present in anonymous-map mode; otherwise fall back to current `estimatedWalls` for existing scenarios.

**Step 3: Verify**

```bash
npm test -- --run src/planning/valueField.spec.ts src/simulation/scenarios.spec.ts
npm test
npm run lint
npm run build
```

**Step 4: Commit**

```bash
git add src/planning/beliefValueField.ts src/planning/valueField.spec.ts
git commit -m "Build value fields from estimated line features"
```

---

## Phase 7: Refactor belief transition APIs for POMDP-style planning

### Task 7: Extract predict / observe / update functions

**Objective:** Make the belief transition explicit enough for future branching or Monte Carlo rollouts.

**Files:**
- Create: `src/belief/beliefTransition.ts`
- Modify: `src/simulation/simulator.ts`
- Modify: `src/planning/rollout.ts`
- Test: `src/belief/beliefTransition.spec.ts`

**Step 1: Write failing test**

Prove that a full step can be expressed as:

```ts
const predicted = predictBelief(belief, control, parameters)
const observations = observeWorld(trueWorld, predicted.pose.mean, parameters)
const updated = updateBeliefWithObservation(predicted, observations, parameters)
```

**Step 2: Implement API**

```ts
export function predictBelief(...): BeliefState
export function updateBeliefWithObservation(...): BeliefState
export function transitionBelief(...): BeliefState
```

Keep existing `updateBelief` as a compatibility wrapper until migration is complete.

**Step 3: Verify**

```bash
npm test -- --run src/belief/beliefTransition.spec.ts src/simulation/simulator.spec.ts src/planning/samplingMpc.spec.ts
npm test
npm run lint
npm run build
```

**Step 4: Commit**

```bash
git add src/belief/beliefTransition.ts src/belief/beliefTransition.spec.ts src/simulation/simulator.ts src/planning/rollout.ts
git commit -m "Extract explicit belief transition API"
```

---

## Phase 8: Add belief-state value expectation

### Task 8: Evaluate terminal cost as expected value over pose belief

**Objective:** Move from `V(x, y)` at the mean pose to `E[V(x, y)]` over pose uncertainty.

**Files:**
- Modify: `src/planning/cost.ts`
- Create: `src/planning/beliefValueExpectation.ts`
- Test: `src/planning/beliefValueExpectation.spec.ts`
- Test: `src/planning/cost.spec.ts`

**Step 1: Write failing test**

A pose belief whose covariance overlaps a wall/unreachable region should have higher expected terminal cost than a tight belief with the same mean.

**Step 2: Implement**

Approximate expectation with deterministic sigma points from the existing Gaussian pose belief:

```ts
export function expectedValueFieldCost(belief: BeliefState, environment: Environment): number
```

Use this in terminal cost when `environment.valueField` and `belief.pose` are available. Keep mean-pose fallback.

**Step 3: Verify**

```bash
npm test -- --run src/planning/beliefValueExpectation.spec.ts src/planning/cost.spec.ts
npm test
npm run lint
npm run build
```

**Step 4: Commit**

```bash
git add src/planning/beliefValueExpectation.ts src/planning/beliefValueExpectation.spec.ts src/planning/cost.ts src/planning/cost.spec.ts
git commit -m "Use expected value-field cost over pose belief"
```

---

## Phase 9: Add future-observation branching for a toy POMDP slice

### Task 9: Add Monte Carlo belief rollouts for candidate controls

**Objective:** Stop assuming a single deterministic future observation path in rollouts.

**Files:**
- Modify: `src/planning/rollout.ts`
- Modify: `src/planning/samplingMpc.ts`
- Create: `src/planning/beliefRollout.ts`
- Test: `src/planning/beliefRollout.spec.ts`

**Step 1: Write failing test**

In a toy partial-map corridor, an action that exposes a high-uncertainty region should have lower expected cost than an action that keeps the robot blind, even if immediate progress is similar.

**Step 2: Implement**

Add bounded stochastic observation samples:

```ts
type BeliefRolloutOptions = {
  observationSamples: number
  seed: number
}
```

For each candidate action sequence, average cost over `N=3` deterministic seeded observation samples. Keep default small to preserve browser performance.

**Step 3: Verify**

```bash
npm test -- --run src/planning/beliefRollout.spec.ts src/planning/samplingMpc.spec.ts
npm test
npm run lint
npm run build
```

**Step 4: Commit**

```bash
git add src/planning/beliefRollout.ts src/planning/beliefRollout.spec.ts src/planning/rollout.ts src/planning/samplingMpc.ts
git commit -m "Add sampled future-observation belief rollouts"
```

---

## Phase 10: Scenarios and evaluation gates

### Task 10: Add scenarios that falsify oracle assumptions

**Objective:** Ensure the simulator demonstrates SLAM/POMDP gaps, not only navigation success.

**Files:**
- Modify: `src/simulation/scenarios.ts`
- Modify: `src/simulation/scenarios.spec.ts`
- Modify: `src/simulation/experimentRunner.spec.ts`

Add scenarios:

1. `ambiguous-loop-closure`
   - repeated rectangular corridor geometry
   - tests association ambiguity and map-feature reuse
2. `occluded-corner-human`
   - dynamic point cluster appears after a turn
   - tests object belief and social risk latency
3. `kidnapped-pose-bend`
   - pose belief mean starts wrong or is disturbed
   - tests `E[V]` and recovery metrics

**Verification:**

```bash
npm test -- --run src/simulation/scenarios.spec.ts src/simulation/experimentRunner.spec.ts
npm test
npm run lint
npm run build
```

**Commit:**

```bash
git add src/simulation/scenarios.ts src/simulation/scenarios.spec.ts src/simulation/experimentRunner.spec.ts
git commit -m "Add scenarios for SLAM POMDP gaps"
```

---

## Preview verification after each phase

After every phase that changes runtime behavior or rendering:

```bash
npm run build
```

Then verify the existing tunnel if running:

```bash
python3 - <<'PY'
import re, urllib.request
url='https://gamma-hamburg-weapon-survey.trycloudflare.com/'
html=urllib.request.urlopen(url, timeout=20).read().decode('utf-8')
asset=re.search(r'/assets/[^"\\']+\\.js', html).group(0)
print(asset)
PY
```

Browser smoke:

- Crossing human: point returns and object belief visible.
- Spiral corridor unknown map: value field still comes from belief, not hidden walls.
- New anonymous-map scenario: estimated local features render separately from dim true walls.
- Console: no JS errors.

---

## What not to claim yet

Until Phases 5–9 land, describe the simulator as:

- “belief-MPC approximation”
- “POMDP/SLAM-aligned architecture in progress”
- “anonymous perception and belief-state planning slices”

Do **not** call it full SLAM or full POMDP while:

- map updates still use true wall IDs,
- rollouts use only one deterministic future observation path,
- value fields are evaluated at mean pose only,
- object identity/classification is heuristic.
