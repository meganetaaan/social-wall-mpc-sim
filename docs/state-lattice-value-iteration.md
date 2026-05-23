# State-Lattice Value Iteration Alignment

The experimental `state-lattice` planner now builds an orientation-aware value and policy table over
`(x, y, theta)` cells. Runtime control is selected by policy lookup from that table, then evaluated as
a single-control rollout for UI consistency with the existing planner result shape. The lattice policy
is cached by deterministic environment geometry, goal, grid, action primitive, iteration, and social-risk
parameters so unchanged known-map calls reuse the precomputed field instead of rebuilding every frame.
Cold `state-lattice` planner calls now request a staged policy build and return a cheap safe fallback while
bounded value-iteration work advances in later calls. Once the staged build completes, the cached policy is
used for single-control policy lookup instead of rebuilding synchronously inside the simulation step.

The runtime lattice source now goes through belief-derived map selection before policy construction. Known-map
beliefs with broad high-confidence true-ID coverage continue to use the full scenario environment. Unknown-map
beliefs prefer anonymous estimated line features when present, preserving feature IDs instead of falling back to
oracle wall labels. Sparse, short, low-confidence, or weakly observed feature sets return a fallback status and
skip staged policy construction, so `state-lattice` keeps using the safe baseline while observations accumulate.
Reliable partial true-ID wall estimates can still produce a lattice source when anonymous features are absent.

The default primitive set is deliberately small but no longer limited to one forward arc pair: it includes
slow and fast forward motion, small and large left/right arcs, and reverse variants. Static social risk can
enter the value backup through generic risk centers from object beliefs or current humans, so a risky cell
has higher value before rollout scoring is considered.

Remaining gaps versus Ueda et al. are substantial:

- The table is still `V(x, y, theta)`, not a belief-space `V(b)` over pose/map/object distributions.
- Dynamic human motion is not propagated through the Bellman backup; current humans are treated as static
  risk centers for the cached policy key.
- Belief-derived lattice maps are still hard geometry once admitted; uncertainty is only used for source
  gating, not represented inside the value table.
- The browser implementation is CPU TypeScript and should move policy construction to a Web Worker, WASM,
  or GPU path before using finer grids or larger maps interactively.
- The spiral known-map regression now covers a long policy-driven route through multiple bends, but exact
  center-goal convergence from the scenario start remains a tuning/primitive-resolution gap.
