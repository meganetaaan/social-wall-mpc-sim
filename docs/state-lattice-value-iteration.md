# State-Lattice Value Iteration Alignment

The experimental `state-lattice` planner now builds an orientation-aware value and policy table over
`(x, y, theta)` cells. Runtime control is selected by policy lookup from that table, then evaluated as
a single-control rollout for UI consistency with the existing planner result shape. The lattice policy
is cached by deterministic environment geometry, goal, grid, action primitive, iteration, and social-risk
parameters so unchanged known-map calls reuse the precomputed field instead of rebuilding every frame.
Cold `state-lattice` planner calls now request policy work through a small service boundary and return a
cheap safe fallback while bounded value-iteration work advances outside the planner call. Once the staged
build completes, the cached policy is used for single-control policy lookup instead of rebuilding
synchronously inside the simulation step. The same service exposes a worker-safe request/response protocol
and worker handler so policy construction can be hosted off the UI thread. Browser bootstrap now installs
the Vite module Worker when Worker construction is available, and falls back silently to the deterministic
in-process service for tests, SSR, and non-browser execution.

The Worker-backed service uses the same staged construction model as the in-process service. A cold request
posts `request-policy` to register or check cache state, then subsequent planner/UI ticks call
`advancePendingBuilds`, which posts `advance-policy-build` with a bounded work budget for each pending
policy. The worker returns `policy-pending` until the value-iteration build is ready, then returns
`policy-ready`; the browser service stores that policy locally for deterministic hits. Worker failures are
captured as `lastError` in the service stats instead of being hidden behind fallback behavior.

The simulator sidebar now includes a compact state-lattice status panel. It reports whether the active
backend is `worker` or `in-process`, ready and pending policy counts, cache hits and misses, and the latest
worker error. This is intentionally observability only: while a policy is pending or a lattice source is not
reliable, `state-lattice` mode continues to use the existing safe baseline control.

The runtime lattice source now goes through belief-derived map selection before policy construction. Known-map
beliefs with broad high-confidence true-ID coverage continue to use the full scenario environment. Unknown-map
beliefs prefer anonymous estimated line features when present, preserving feature IDs instead of falling back to
oracle wall labels. Sparse, short, low-confidence, or weakly observed feature sets return a fallback status and
skip staged policy construction, so `state-lattice` keeps using the safe baseline while observations accumulate.
Reliable partial true-ID wall estimates can still produce a lattice source when anonymous features are absent.

The default primitive set is deliberately small but no longer limited to one forward arc pair: it includes
slow and fast forward motion, small and large left/right arcs, and reverse variants. Social risk can enter
the value backup through generic risk centers from object beliefs or current humans, so a risky cell has
higher value before rollout scoring is considered. Risk centers may also carry velocity and optional
temporal radius uncertainty; transition cost samples each primitive and evaluates the risk center predicted
at that sample's time offset. This lets the cached `V(x, y, theta)` policy distinguish a moving human that
will cross a future primitive path from the same human frozen at the current pose.

Ready policy lookup now has a small pose-belief bridge. When `belief.pose` has usable non-tiny covariance,
`state-lattice` samples a deterministic set of sigma points around the pose mean, looks up the table policy
and value at each sample, and selects the action with the strongest sigma-point consensus with value-based
tie breaking. Degenerate or unavailable pose covariance keeps the previous direct mean/current-pose lookup.
This is only an expected policy-selection bridge over the existing `V(x, y, theta)` table; it is not full
belief-space value iteration.

Remaining gaps versus Ueda et al. are substantial:

- The table is still `V(x, y, theta)`, not a belief-space `V(b)` over pose/map/object distributions.
- Pose covariance affects only the final action lookup over deterministic sigma points, not the dynamic
  programming backup itself.
- Dynamic human motion is represented only as deterministic linear risk-center prediction over each action
  primitive, not as a full belief distribution over future human trajectories.
- Belief-derived lattice maps are still hard geometry once admitted; uncertainty is only used for source
  gating, not represented inside the value table.
- Map and object uncertainty are not branched through future observations; there is no observation tree or
  belief transition inside the lattice value iteration.
- Browser bootstrap installs a Worker-backed service when available, but non-browser runs and Worker
  construction failures still use the in-process service.
- The browser implementation is CPU TypeScript and should move heavier policy construction to the Worker
  adapter, WASM, or GPU path before using finer grids or larger maps interactively.
- The spiral known-map regression now covers a long policy-driven route through multiple bends, but exact
  center-goal convergence from the scenario start remains a tuning/primitive-resolution gap.
