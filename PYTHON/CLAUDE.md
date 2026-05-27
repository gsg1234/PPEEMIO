# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Purpose

A 2D corotational finite element method (FEM) solver for large-deformation analysis of beam structures. Implements incremental loading with Newton-Raphson iterative correction for geometrically non-linear problems.

## Running the Code

```bash
python MEF.py              # Run main solver (hardcoded example: 0.4m beam, 20 elements, 3000 increments)
python runtimeMEF.py       # cProfile performance report
python nincMEF.py          # Parametric study: NINC vs runtime
python maillageMEF.py      # Parametric study: N_ELEM vs runtime and convergence
python moduleY.py          # Parametric study: Young's modulus sensitivity
```

Dependencies: `numpy`, `matplotlib`

## Architecture

Three modules:

**`constants.py` — problem geometry**
Defines the two clamping positions used in the canonical problem:
- `POS_ENCASTREMENT1 = [-0.1, 0]` — position of node 0 (fixed end)
- `POS_ENCASTREMENT2 = [0.1, 0, π]` — target position+angle for node 20 (driven end)

**`Beam.py` — element**
`Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)` — holds all per-beam state: geometry (`u`, `L`, `L0`, `Beta`, `cos`, `sin`), local forces (`ql = [N, M1, M2]` per element), global internal forces (`q`), and matrices (`K`, `kt`, `k_sigma`, `B`, `C_all`).

Physical properties computed from cross-section: `AREA`, `INERTIA`, `radius`, `POID` (total weight), `Mc` (critical buckling moment). Two stiffness matrices `C1` / `C2` are built; `C_all` (3×3×N_ELEM) applies `C2` to elements 9 and 10 to simulate a stiffer arm attachment. `YOUNG_elem` (N_ELEM,) mirrors this per-element.

- `configuration_neutre(gamma, x0, y0)` — initializes nodal positions for a straight beam at angle `gamma` starting at `(x0, y0)`; computes `Beta_0`, `L0`, and corotational frame vectors `z`, `r`
- `forces_externes()` — accumulates distributed self-weight into `dF` (trapezoidal rule: half-weight on end nodes, full-weight on interior nodes)
- `actualiser_conf(u)` — given a displacement vector, recomputes `L`, `Beta`, `cos`, `sin`, `z`, `r`, and the Green-Lagrange axial strain `ul = (L²-L0²)/(L+L0)`; returns `(tita, ul)`
- `actualiser_iforces(tita, ul)` — computes local forces `ql` (axial N and bending moments M1/M2 using per-element `YOUNG_elem`) then assembles global `q` via `B^T * ql`
- `actualiser_ks()` — updates `B` from current orientation, assembles tangent stiffness `kt = einsum(B, C_all, B)` and geometric stiffness `k_sigma` (using `alpha=N/L`, `beta=(M1+M2)/L²` and pre-allocated `zz`, `zr`, `rz` buffers), assembles global `K`

**`MEF.py` — solver**
`MEF(large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, draw_every=1)` — manages a single `Beam` instance (`beam`), boundary conditions, live plotting, and the solve pipeline.

- `solve(type, ddl_bloques, deltaU, noeud, dF, live_plot)` — dispatcher: routes to `solve_increment_deplacement` or `solve_increment_charge`, then calls `_draw()`
- `solve_increment_deplacement(deltaU, noeud, ddl_bloque, live_plot)` — displacement-controlled: prescribes `deltaU/NINC` to `noeud` each increment, then runs Newton-Raphson on the free DOFs
- `solve_increment_charge(ddl_bloque, dF, live_plot)` — force-controlled: applies `dF` (already per-increment) each step, same NR loop
- `position_u(live_plot)` — drives node 20 to `constants.POS_ENCASTREMENT2` via displacement solve; constrains nodes 0 and 20 in all DOFs
- `ajouter_liason_bras(live_plot)` — constrains nodes 9, 10, 11 (x, y; rotation free on 9 and 11) then drives node 10 to align with node 9's y-position; simulates arm attachment
- `condition_initiale(live_plot)` — initializes a vertical beam at `constants.POS_ENCASTREMENT1`, calls `position_u()`, then resets `NINC = 150`
- `montrer_solution()` — prints final nodal state (x, y, tita, L, q, F) and redraws
- `_draw()` — updates live plot: beam line in mm + quiver arrows for non-zero nodal forces colored by magnitude; recreates figure if window was closed
- `_setup_axes()` — configures axis labels, grid, equal aspect ratio

Helper `obtener_gdl_bloqueados_con_nombres(restricciones, numeracion_nodos)` — converts a named node/direction constraint dict into a flat list of global DOF indices.

## Key Data Structures

- **DOF vector** `u`: flat array `[x0, y0, θ0, x1, y1, θ1, ..., xN, yN, θN]` — 3 DOFs per node
- **Local forces** `ql`: flat array `[N0, M1_0, M2_0, N1, M1_1, M2_1, ...]` — 3 per element
- **Stiffness per element** `C_all`: `(3, 3, N_ELEM)` array; elements 9 and 10 use `C2` (stiffer), all others use `C1`
- **Free DOFs** `ddl`: index array obtained by deleting constrained indices from `arange(3*N_NODES)`
- **Evolution history** `evol_u`: `(NINC, 3*N_NODES)` array storing the full displacement state after each increment

## Newton-Raphson Loop Pattern

Both solve methods share the same structure per increment:

1. Apply load/displacement increment to `beam.u`
2. Call `actualiser_conf` → `actualiser_iforces` to get current `q`
3. Compute residual `R = q[ddl] - F[ddl]`
4. Iterate until `||R|| < tol` (max `maxiter` iterations):
   - `actualiser_ks()` to refresh `K`
   - `dUk[ddl] -= K[ddl,ddl]^-1 * R`
   - Update trial `u_cur`, recompute conf/iforces/R
5. On non-convergence: print diagnostic and `quit()`

## Diagnostics

Non-convergence (`Pas de convergence` message) is most often fixed by:
- More increments (`NINC` ↑) — smaller steps
- More NR iterations (`maxiter` ↑)
- Looser tolerance (`tol` ↑, though less accurate)

The assembly loops in `actualiser_ks()` and `actualiser_iforces()` (`for i in range(N_ELEM)`) are the main performance bottleneck for large meshes.
