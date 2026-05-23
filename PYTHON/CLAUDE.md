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

Dependencies: `numpy`, `matplotlib`, `mpl-interactions`

## Architecture

Two classes with a clear separation:

**`Beam.py` — element**
Holds all per-beam state: geometry (`u`, `L`, `L0`, `Beta`, `cos`, `sin`), local forces (`ql = [N, M1, M2]` per element), global internal forces (`q`), and matrices (`K`, `kt`, `k_sigma`, `B`, `C`).

- `configuration_neutre(gamma, x0, y0)` — initializes nodal positions for a straight beam at angle `gamma`; computes `Beta_0`, `L0`, and the corotational frame vectors `z`, `r`
- `actualiser_conf(u)` — given a displacement vector, recomputes `L`, `Beta`, `cos`, `sin`, `z`, `r`, and the Green-Lagrange axial strain `ul = (L²-L0²)/(L+L0)`; returns `(tita, ul)`
- `actualiser_iforces(tita, ul)` — computes local forces `ql` (axial N and bending moments M1/M2) then assembles global `q` via `B^T * ql`
- `actualiser_ks()` — updates `B` from current orientation, assembles tangent stiffness `kt = B^T C B` and geometric stiffness `k_sigma` (using `alpha=N/L`, `beta=(M1+M2)/L²`), assembles global `K`

**`MEF.py` — solver**
Manages a single `Beam` instance (`beam1`), boundary conditions, and visualization.

- `solve_increment_deplacement(deltaU, noeud, ddl_bloque, live_plot)` — displacement-controlled: prescribes `deltaU/NINC` to `noeud` each increment, then runs Newton-Raphson on the free DOFs
- `solve_increment_charge(ddl_bloque, dF, live_plot)` — force-controlled: applies `dF/NINC` each increment, same NR loop
- `condition_initiale(pos_encastrement, pos_finale)` — sets up the canonical problem: vertical beam, node 0 and node 20 constrained, prescribes final position of node 20, then runs displacement solve followed by a force (gravity) solve
- `montrer_solution()` — plots final deformed shape with force arrows (`mode='fs'`) or overlaid incremental snapshots colored by rainbow map (`mode='evol'`)

## Key Data Structures

- **DOF vector** `u`: flat array `[x0, y0, θ0, x1, y1, θ1, ..., xN, yN, θN]` — 3 DOFs per node
- **Local forces** `ql`: flat array `[N0, M1_0, M2_0, N1, M1_1, M2_1, ...]` — 3 per element
- **Free DOFs** `ddl`: index array obtained by deleting constrained indices from `arange(3*N_NODES)`
- **Evolution history** `evol_u`: `(NINC, 3*N_NODES)` array storing the full displacement state after each increment

## Newton-Raphson Loop Pattern

Both solve methods share the same structure per increment:

1. Apply load/displacement increment to `beam1.u`
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

The assembly loop in `actualiser_ks()` (`for i in range(N_ELEM)`) is the main performance bottleneck for large meshes.
