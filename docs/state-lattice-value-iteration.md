# State-Lattice Value Iteration Alignment

The experimental `state-lattice` planner now builds an orientation-aware value and policy table over
`(x, y, theta)` cells. Runtime control is selected by policy lookup from that table, then evaluated as
a single-control rollout for UI consistency with the existing planner result shape. The lattice policy
is cached by deterministic environment geometry, goal, grid, action primitive, iteration, and social-risk
parameters so unchanged known-map calls reuse the precomputed field instead of rebuilding every frame.

The default primitive set is deliberately small but no longer limited to one forward arc pair: it includes
slow and fast forward motion, small and large left/right arcs, and reverse variants. Static social risk can
enter the value backup through generic risk centers from object beliefs or current humans, so a risky cell
has higher value before rollout scoring is considered.

Remaining gaps versus Ueda et al. are substantial:

- The table is still `V(x, y, theta)`, not a belief-space `V(b)` over pose/map/object distributions.
- Dynamic human motion is not propagated through the Bellman backup; current humans are treated as static
  risk centers for the cached policy key.
- The browser implementation is CPU TypeScript and should move policy construction to a Web Worker, WASM,
  or GPU path before using finer grids or larger maps interactively.
- The spiral known-map regression now covers a long policy-driven route through multiple bends, but exact
  center-goal convergence from the scenario start remains a tuning/primitive-resolution gap.
