# Social Wall MPC Simulator

Browser-based experiment for a socially-aware wall-following robot. The simulator intentionally avoids a separate global planner / local planner switch: every frame uses one finite-horizon sampling MPC objective that combines wall following, human social distance, collision avoidance, control effort, smoothness, progress, pose uncertainty, and wall-map belief uncertainty.

## Run

```bash
npm install
npm run dev
```

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
            + ℓ_smooth + ℓ_progress + ℓ_pose_uncertainty
            + ℓ_map_uncertainty + ℓ_wall_belief_consistency
            - information_gain
```

where `b_t` is represented by a lightweight pose covariance plus per-wall confidence values. This is not full EKF-SLAM, but the wall map itself is part of the belief state and is updated as the robot observes nearby wall segments.

## Implemented approximation

At each control cycle:

1. Generate `sampleCount` candidate control sequences over `horizonSteps`.
2. Roll out differential-drive dynamics:
   `x += v cos(theta) dt`, `y += v sin(theta) dt`, `theta += omega dt`.
3. Predict humans with a deterministic constant-velocity/bouncing-boundary model.
4. Update a simplified belief: pose uncertainty grows with motion and shrinks near observed walls; nearby wall segments gain confidence while unobserved segments remain uncertain.
5. Evaluate the unified cost breakdown for each rollout.
6. Apply only the first control from the best sequence.
7. Repeat in the next animation frame.

The planner is deterministic for a fixed seed (`mulberry32`) so behavior can be reproduced.

## Cost terms

All cost terms are implemented as inspectable functions in `src/planning/cost.ts`.

- **Wall distance**: `w_wall * (d_wall - d_wall_target)^2` against the nearest selected wall.
- **Wall heading**: tangent-alignment error between robot heading and wall tangent.
- **Human social distance**: very large penalty inside `d_min`, plus asymmetric near-distance and preferred-distance penalties around `d_pref`.
- **Collision**: soft barriers for walls, static obstacles, and physical overlap with humans.
- **Control effort**: quadratic penalty on `v` and `omega`.
- **Smoothness**: quadratic penalty on changes from the previous control input.
- **Progress**: negative reward for moving forward along the nearest wall tangent.
- **Pose uncertainty**: `w_uncertainty * trace(Sigma_pose)`.
- **Map uncertainty**: penalty from the average `(1 - wall_confidence)` over the wall-belief map.
- **Expected observation gain**: negative cost for trajectories that keep uncertain walls observable, approximating information-seeking behavior.
- **Wall-belief consistency**: extra wall-following penalty when the nearest wall has low confidence, so the optimizer prefers trajectories that maintain useful wall observations rather than treating the map as fully known.

## UI

The browser view shows:

- top-down world with static walls and a pillar obstacle
- cyan wall-belief overlay: brighter segments are better observed / higher confidence
- robot pose and heading
- moving humans with `d_min` and `d_pref` social zones
- sampled candidate trajectories
- selected trajectory
- robot trace/history
- live cost breakdown and belief readout, including map-confidence terms
- start/pause/reset/step controls
- sliders for social, wall, cost, horizon, and sampling parameters

## Project structure

```text
src/
  App.tsx
  belief/simpleBelief.ts
  planning/cost.ts
  planning/rollout.ts
  planning/samplingMpc.ts
  rendering/CanvasView.tsx
  rendering/draw.ts
  simulation/dynamics.ts
  simulation/environment.ts
  simulation/humans.ts
  simulation/simulator.ts
  simulation/types.ts
  ui/Controls.tsx
  ui/CostPanel.tsx
  ui/ParameterPanel.tsx
```

Algorithm code is kept independent from rendering. Rendering receives immutable-ish state snapshots.

## Known limitations

- Belief update is a pedagogical scalar covariance and per-wall confidence approximation, not EKF/Graph-SLAM.
- Candidate generation is simple sampling around a few motion templates, not full MPPI with weighted updates.
- Human prediction is constant-velocity and does not model intent.
- Wall selection is nearest-wall based; there is no global route or topological planner.
- Safety constraints are soft penalties, so extreme parameter choices can still produce unsafe rollouts.
