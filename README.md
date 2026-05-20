# Social Wall MPC Simulator

Browser-based experiment for a socially-aware wall-following robot. The default `belief-mpc` policy intentionally avoids a separate global planner / local planner switch: every frame uses one finite-horizon sampling MPC objective that combines wall following, goal progress, human social distance, collision avoidance, control effort, smoothness, pose uncertainty, and wall-map belief uncertainty. Simple baseline policies are included only for comparison.

## Run

```bash
npm install
npm run dev
```

Open the Vite URL printed by `npm run dev`. The app runs fully in the browser; there is no backend.

The public GitHub Pages deployment is configured for:

```text
https://meganetaaan.github.io/social-wall-mpc-sim/
```

Every push to `main` runs lint, tests, build, and then deploys the `dist` artifact through GitHub Pages.

Quality gates:

```bash
npm test
npm run lint
npm run build
```

## Mathematical formulation

The conceptual target is a belief-space optimal-control problem:

```text
π* = argmin_π E[Σ_t ℓ(b_t, u_t) + ℓ_T(b_T)]

b_t = p(s_t | z_1:t, u_0:t-1)
s_t = (x_t, m, h_t)
```

This prototype approximates the stage cost as:

```text
ℓ(b_t, u_t) ≈ ℓ_wall + ℓ_human + ℓ_collision + ℓ_control
            + ℓ_smooth + ℓ_progress + ℓ_goal + ℓ_pose_uncertainty
            + ℓ_map_uncertainty + ℓ_wall_belief_consistency
            - information_gain
```

where `b_t` is represented by a lightweight pose covariance plus an estimated wall map. The simulator keeps the true wall map for collision and sensor simulation, while planning reads from estimated wall coverage and confidence. This is not full EKF-SLAM, but map estimation is part of the same receding-horizon belief state used by the objective.

## Implemented approximation

At each control cycle:

1. Generate `sampleCount` candidate control sequences over `horizonSteps`.
2. Roll out differential-drive dynamics:
   `x += v cos(theta) dt`, `y += v sin(theta) dt`, `theta += omega dt`.
3. Predict humans with a deterministic constant-velocity/bouncing-boundary model.
4. Generate deterministic noisy range/bearing observations from the robot sensor cone against true wall segments.
5. Update a simplified belief: pose uncertainty grows with motion and shrinks near observed walls; observed wall intervals expand their estimated coverage and confidence while unobserved intervals remain uncertain.
6. Evaluate the unified cost breakdown for each rollout.
7. Apply only the first control from the best sequence.
8. Repeat in the next animation frame.

The planner is deterministic for a fixed seed (`mulberry32`) so behavior can be reproduced. The same simulation loop is available without React or canvas through `runScenarioExperiment` in `src/simulation/experimentRunner.ts`.

## Scenarios

The scenario selector resets the simulator into deterministic, named experiment cases. Re-running the same scenario with the same planner parameters and policy mode makes tuning and comparison repeatable.

- **Crossing human**: default wall-following scene with a crossing person, another person near the wall, and a reachable green goal in the upper corridor.
- **Standing human**: a person stands near the followed wall, encouraging slow/yield/deviation behavior.
- **Head-on human**: a person moves toward the robot along the wall corridor.
- **Blocked corridor**: a near-wall blockage makes stopping useful for several steps.
- **Partial-map bend**: a bent and alcove-like wall layout where estimated wall coverage matters.
- **Multi-human**: multiple people create competing social costs.
- **Followed from behind**: a person comes from behind and keeps tracking the robot, testing rear social pressure rather than only frontal obstacles.
- **Overtaking human**: a faster scripted person passes the robot on the outside and merges ahead.
- **Yielding blocker**: a person initially blocks the corridor, then steps aside after a delay so the unified objective can move from stop/yield back to wall following.
- **Spiral corridor known map**: a rectangular spiral corridor with the goal at the center and high initial wall-map confidence.
- **Spiral corridor unknown map**: the same center-goal spiral, but only the entry wall starts with low confidence so observation gain and map uncertainty matter more.

Scenarios are defined in `src/simulation/scenarios.ts` and instantiate only simulation state: robot pose, humans, environment, and initial belief. Rendering does not contain scenario logic.

## Policy comparison

The policy selector compares three modes:

- **belief-mpc**: the proposed mode and default. It optimizes wall following, social distance, collision, smoothness, progress, and belief/map uncertainty together in one receding-horizon cost.
- **wall-only**: a small baseline wall follower that ignores humans and uncertainty. It is expected to keep moving even when that creates social-distance violations.
- **reactive-stop**: a hard-switch baseline that follows the wall until any human is within `d_min` plus a small buffer, then abruptly commands zero velocity.

The baselines are comparison tools, not alternative proposed methods. They intentionally separate simple behaviors so the unified-cost formulation can be evaluated against predictable reference policies.

## Experiment comparison

The app includes a **Run comparison** button below the live metric panel. It runs deterministic headless replays in the browser with the same `stepSimulation` code used by the canvas animation, then shows one row per scenario and policy. The table is meant for quick side-by-side checks: lower social violations and near collisions are better, lower best goal distance means the robot got closer to the goal, and goal/time indicate whether the local receding-horizon controller reached the marker within the step bound.

The headless API is also exported from `src/simulation/experimentRunner.ts` as `runScenarioBatch`, which defaults to every scenario and the three policy modes. The UI uses a focused batch of `crossing-human`, the new human-motion scenarios, and both spiral-map variants with `maxSteps: 240`.

Measured local batch results with default planner parameters and `maxSteps: 240`:

| Scenario | Policy | Goal | Time | Best goal dist | Min human dist | Social violations | Near collisions | Stop duration |
| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |
| Crossing human | belief-mpc | no | - | 0.21m | 0.91m | 0 | 0 | 15.8s |
| Crossing human | wall-only | yes | 23.8s | 0.20m | 0.09m | 20 | 0 | 0.0s |
| Crossing human | reactive-stop | no | - | 0.29m | 0.46m | 34 | 0 | 5.3s |
| Followed from behind | belief-mpc | yes | 16.6s | 0.18m | 0.61m | 0 | 0 | 4.3s |
| Followed from behind | wall-only | yes | 23.3s | 0.19m | 0.61m | 7 | 0 | 0.0s |
| Followed from behind | reactive-stop | yes | 28.1s | 0.19m | 0.45m | 26 | 0 | 4.8s |
| Overtaking human | belief-mpc | no | - | 5.45m | 0.51m | 22 | 0 | 25.6s |
| Overtaking human | wall-only | yes | 23.5s | 0.17m | 0.18m | 32 | 0 | 0.0s |
| Overtaking human | reactive-stop | yes | 25.7s | 0.17m | 0.40m | 14 | 0 | 2.2s |
| Yielding blocker | belief-mpc | no | - | 5.46m | 1.00m | 0 | 0 | 24.0s |
| Yielding blocker | wall-only | yes | 23.8s | 0.20m | 0.21m | 24 | 0 | 0.0s |
| Yielding blocker | reactive-stop | no | - | 5.44m | 0.81m | 0 | 0 | 26.5s |
| Spiral corridor known map | belief-mpc | no | - | 2.09m | 1.49m | 0 | 240 | 18.5s |
| Spiral corridor known map | wall-only | no | - | 0.28m | 0.94m | 0 | 160 | 0.0s |
| Spiral corridor known map | reactive-stop | no | - | 0.28m | 0.94m | 0 | 160 | 0.0s |
| Spiral corridor unknown map | belief-mpc | no | - | 1.84m | 1.92m | 0 | 240 | 16.2s |
| Spiral corridor unknown map | wall-only | no | - | 0.28m | 0.11m | 55 | 160 | 0.0s |
| Spiral corridor unknown map | reactive-stop | no | - | 0.32m | 0.77m | 0 | 240 | 18.1s |

These numbers are deterministic for the fixed scenarios, seeds, parameters, and step bound, but they are not a benchmark suite. The optimizer is sample-based with fixed seeds, human prediction is simplified, the belief update is a compact approximation rather than SLAM, and the selected scenarios are small regression cases for this simulator.

## Goal reaching

The green marker is testable state, not just a visual target. `Environment.goalRadius` defines the arrival threshold; if a scenario omits it, the simulator uses a stricter default radius of `0.20m`. The default scenario sets a `0.21m` threshold so arrival still requires the robot center to approach the green marker closely rather than merely entering a broad visual ring.

Goal arrival means the robot center is within that radius of `Environment.goal`. Once reached, `goalReached` stays true and `timeToGoal` records the first simulated arrival time. The simulator also tracks current `goalDistance` and `bestGoalDistance`.

Headless verification uses:

```ts
runScenarioExperiment({
  scenarioId: 'crossing-human',
  plannerMode: 'belief-mpc',
  parameters: { sampleCount: 12, horizonSteps: 6 },
  maxSteps: 120,
})
```

The runner builds a deterministic scenario state, steps the same simulator and policy code used by the browser, stops early on goal arrival, and returns the final state plus goal metrics. This is covered by tests so the default `belief-mpc` scenario must keep reaching the green marker within the bounded run.

## Metrics

The metrics panel reports compact experiment readouts:

- **Elapsed time**: simulated seconds.
- **Mean/max wall distance error**: observed error from `d_wall_target`.
- **Minimum human distance**: closest robot-human center distance observed so far.
- **Social violations**: steps where robot-human distance is below `d_min`.
- **Near collisions**: steps where the robot is too close to walls or static obstacles.
- **Stop duration**: accumulated time with commanded `v < 0.05`.
- **Wall progress**: accumulated forward motion along the local wall tangent.
- **Goal distance**: current distance from robot center to the green goal.
- **Goal reached**: sticky yes/no arrival state using the scenario goal radius.
- **Time to goal**: first simulated arrival time, shown only after reaching.
- **Best goal distance**: closest distance observed so far.
- **Trace sigma**: current `sigmaX + sigmaY + sigmaTheta`.
- **Map coverage**: fraction of true wall length covered by estimated wall intervals.
- **Pose position error**: Euclidean distance between the true robot pose and `belief.pose.mean`.
- **Pose heading error**: absolute wrapped heading error between the true heading and `belief.pose.mean.theta`.
- **Pose normalized error**: diagonal-covariance normalized pose error, using safe lower bounds for `x`, `y`, and heading variance.
- **Map knowledge error**: true-map-backed score in `[0, 1]`; lower means estimated wall length is covered with higher confidence, while higher means coverage or confidence is poor.
- **Selected cost**: total cost of the selected rollout for the current step.

The pose and map-knowledge errors are inference-quality metrics for this approximation. They intentionally use simulator ground truth for evaluation only; the planner still consumes the lightweight belief state rather than a full SLAM posterior.

## Cost terms

All cost terms are implemented as inspectable functions in `src/planning/cost.ts`.

- **Wall distance**: `w_wall * (d_wall - d_wall_target)^2` against the nearest selected wall.
- **Wall heading**: tangent-alignment error between robot heading and wall tangent.
- **Human social distance**: very large penalty inside `d_min`, plus asymmetric near-distance and preferred-distance penalties around `d_pref`.
- **Collision**: soft barriers for walls, static obstacles, and physical overlap with humans.
- **Control effort**: quadratic penalty on `v` and `omega`.
- **Smoothness**: quadratic penalty on changes from the previous control input.
- **Progress**: negative reward for moving forward along the nearest wall tangent.
- **Goal progress**: negative reward for local motion in the direction of `Environment.goal`.
- **Goal terminal**: terminal rollout penalty on final distance to `Environment.goal`.
- **Pose uncertainty**: `w_uncertainty * trace(Sigma_pose)`.
- **Map uncertainty**: penalty from uncovered or low-confidence intervals in the estimated wall map.
- **Expected observation gain**: negative cost for trajectories whose sensor cone is expected to cover uncertain wall intervals.
- **Wall-belief consistency**: extra wall-following penalty when the nearest or nearby walls have low estimated coverage/confidence, so the optimizer prefers trajectories that maintain useful wall observations rather than treating the map as fully known.

## Map belief and observations

The true map is `Environment.walls`. It is deliberately separate from `BeliefState.estimatedWalls`, which stores one merged observed interval per wall as `[tMin, tMax]` in segment parameter space, plus confidence and `lastObservedAt`.

The observation model is a deterministic approximation, not ray-cast SLAM:

- the robot has a limited `sensorRadius` and `sensorFov`
- sample points on each true wall are observable when they are inside range and bearing limits and have line of sight past nearer walls
- an observation records the visible interval, strength/confidence, a representative ray target, and deterministic noisy `range`/`bearing` measurements with parameter-derived standard deviations
- observations also carry the sensor pose and can be evaluated against a candidate pose and wall with an independent Gaussian `p(z | x, m)` range/bearing likelihood
- repeated observations merge intervals and raise confidence
- wall confidence growth is lightly gated by a bounded affinity derived from that likelihood
- uncovered portions of a wall continue to contribute map uncertainty

## UI

The browser view shows:

- top-down world with static walls and a pillar obstacle
- dim gray true wall map
- cyan estimated-map overlay only on observed wall intervals
- subtle sensor field of view and brighter current observation rays/highlights
- robot pose and heading
- goal radius ring, `GOAL` label, and `GOAL REACHED` state after arrival
- moving humans with `d_min` and `d_pref` social zones
- sampled candidate trajectories
- selected trajectory
- robot trace/history
- live cost breakdown and belief readout, including map-confidence terms
- live experiment metrics for repeated comparisons
- scenario and policy selectors
- start/pause/reset/step controls
- sliders for social, wall, cost, horizon, and sampling parameters

## Project structure

```text
src/
  App.tsx
  belief/simpleBelief.ts
  belief/wallMapBelief.ts
  planning/cost.ts
  planning/policies.ts
  planning/rollout.ts
  planning/samplingMpc.ts
  rendering/CanvasView.tsx
  rendering/draw.ts
  simulation/dynamics.ts
  simulation/environment.ts
  simulation/experimentRunner.ts
  simulation/humans.ts
  simulation/metrics.ts
  simulation/scenarios.ts
  simulation/simulator.ts
  simulation/types.ts
  ui/Controls.tsx
  ui/CostPanel.tsx
  ui/ParameterPanel.tsx
```

Algorithm code is kept independent from rendering. Rendering receives immutable-ish state snapshots.

## Known limitations

- Belief update is a pedagogical scalar covariance plus per-wall interval coverage approximation, not EKF/Graph-SLAM.
- Wall visibility uses sampled wall points with line-of-sight occlusion, not continuous geometric clipping of full visible wall intervals.
- Wall observations expose deterministic noisy measurements and a Gaussian wall likelihood, but data association is still by known `wallId`; there is no anonymous feature map, EKF, particle SLAM, or continuous ray-cast SLAM.
- Candidate generation is simple sampling around a few motion templates, not full MPPI with weighted updates.
- Human prediction is constant-velocity and does not model intent.
- Wall selection is nearest-wall based; there is no global route or topological planner.
- Safety constraints are soft penalties, so extreme parameter choices can still produce unsafe rollouts.
- Baselines are deliberately simple and should not be interpreted as tuned controllers.
- Metrics include simple inference-quality errors for pose and true-map-backed wall knowledge, but they are still lightweight geometric summaries rather than a statistical benchmark suite.
- Goal reaching is still local and receding-horizon. There is no global planner, so reaching remains scenario-dependent and can fail if the goal is permanently blocked or placed behind a route that the local candidate set cannot discover.

Current roadmap position: this version adds pose and map inference-quality readouts for the belief-SLAM approximation. It still does not implement full probability-distribution belief state, anonymous wall data association, human-state belief, FastSLAM, POMCP, or a true POMDP planner.
