# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Purpose

A 2D corotational finite element method (FEM) solver for large-deformation analysis of a flexible beam (bras bleus) mounted on the EMIO robot. Implements incremental loading with Newton-Raphson iterative correction for geometrically non-linear problems, with live hardware control via camera tracking and motor commands.

## Running the Code

```bash
python MEF.py              # Run main solver with live EMIO hardware connection
python runtimeMEF.py       # cProfile performance report
python nincMEF.py          # Parametric study: NINC vs runtime
python maillageMEF.py      # Parametric study: N_ELEM vs runtime and convergence
python moduleY.py          # Parametric study: Young's modulus sensitivity
```

Dependencies: `numpy`, `matplotlib`, `emioapi`

## Architecture

Four modules:

**`constants.py` — robot geometry**
Physical constants for the EMIO robot:
- `L = 0.1` — distance from robot frame centre to motor centre [m]
- `R = 0.025` — distance from motor centre to arm attachment point [m]
- `POS_ENCASTREMENT3 = [-(L+R), 0, 0]` — initial position of node 0 (motor 3 side)
- `POS_ENCASTREMENT1 = [L+R, 0, π]` — target position+angle for last node (motor 1 side)
- `CENT_MOT1 = [L, 0]`, `CENT_MOT3 = [-L, 0]` — motor centre positions
- `L_LIAISON = 0.04` — length of the rigid link joining the two blue arms [m]
- `DENSITE_TPU = 1220.0` — TPU density [kg/m³]

**`CD.py` — motor kinematics**
Computes attachment point positions at the rim of each motor disk given a rotation angle:
- `get_pos_encastrement1(tita)` → `CENT_MOT1 + R*[cos(tita), sin(tita)]`
- `get_pos_encastrement3(tita)` → `CENT_MOT3 + [-R*cos(tita), R*sin(tita)]`

**`Beam.py` — element**
`Beam(large, haut, L0t, YOUNG, N_ELEM, NINC)` — holds all per-beam state: geometry (`u`, `L`, `L0`, `Beta`, `cos`, `sin`), local forces (`ql = [N, M1, M2]` per element), global internal forces (`q`), and matrices (`K`, `kt`, `k_sigma`, `B`, `C_all`).

Physical properties computed from cross-section: `AREA`, `INERTIA`, `radius`, `POID` (total weight). Two stiffness matrices are built: `C1` (YOUNG1 = YOUNG, for blue arm elements) and `C2` (YOUNG2 = 20×YOUNG, for the central rigid link element at `floor(N_ELEM/2)`). `_C_base` stores the unscaled base; `C_all = _C_base / L0` is computed in `configuration_neutre()` once lengths are known. `YOUNG_elem` (N_ELEM,) mirrors the per-element Young's modulus. `tita1` and `tita3` are scalar state variables tracking the current motor angles.

- `configuration_neutre(gamma, x0, y0)` — initializes a straight beam at angle `gamma` from `(x0, y0)`, split into two halves separated by `L_LIAISON` (the rigid link gap); computes real `Beta_0`, `L0` per element, builds `C_all`, `z`, `r`, and calls `actualiser_b()`
- `actualiser_b()` — updates corotational transformation matrix `B` (3×6×N_ELEM) from current `cos`, `sin`, `L`; called by `configuration_neutre()` and `actualiser_ks()`
- `actualiser_conf(u)` — given a displacement vector, recomputes `L`, `Beta`, `cos`, `sin`, `z`, `r`, and the Green-Lagrange axial strain `ul = (L²-L0²)/(L+L0)`; returns `(tita, ul)`
- `actualiser_iforces(tita, ul)` — computes local forces `ql` (axial N and bending moments M1/M2 using per-element `YOUNG_elem`) then assembles global `q` via loop over `B[:,:,i].T @ ql[3i:3i+3]`
- `actualiser_ks()` — calls `actualiser_b()`, then assembles tangent stiffness `kt = einsum(B, C_all, B)` and geometric stiffness `k_sigma` (using `alpha=N/L`, `beta=(M1+M2)/L²` and pre-allocated `zz`, `zr`, `rz` buffers); assembles global `K` by loop over elements

**`MEF.py` — solver + hardware interface**
`MEF(large, haut, L0t, YOUNG, N_ELEM, NINC, maxiter, tol, draw_every=1)` — manages a `Beam` instance, boundary conditions, EMIO hardware (camera + motors), live plotting with interactive widgets, and the solve pipeline.

On init: connects `EmioCamera(track_markers=True)` and `EmioMotors()`, starts a background daemon thread for continuous camera polling (stores latest marker position in `_tracker_pos` behind `_tracker_lock`), opens a matplotlib figure with interactive widgets.

**Figure layout** (`_init_figure`):
- Main beam axes + colorbar axis (20:1 width ratio)
- Green circle patch (`_point_vert`) for the tracked marker
- Dashed circles showing motor disk boundaries (CENT_MOT1, CENT_MOT3)
- `TextBox` for **Tita 3** (node 0 / motor 3) and **Tita 1** (last node / motor 1) — in degrees
- Button **Telecharger forces**: loads `efforts.json` and runs a force-controlled solve
- Button **Reconection EMIO**: retries camera and motor connections, updates status labels
- Button **Actualiser point**: manually refreshes the green marker position from camera
- Status labels for camera and motor connection state

**Solve methods:**
- `solve(type, liste_ddl_contraintes, U, noeud, F, live_plot)` — dispatcher to `solve_increment_charge` (type='force') or `solve_increment_deplacement` (type='deplacement'), then calls `_draw()`
- `solve_increment_deplacement(U, noeud, liste_ddl_contraintes, live_plot)` — displacement-controlled: prescribes `(U - u_current)/NINC` to `noeud` each increment, then runs Newton-Raphson on the free DOFs
- `solve_increment_charge(liste_ddl_contraintes, F, live_plot)` — force-controlled: applies `(F_target - F_current)/NINC` each step; uses `actualiser_ks()` first to predict `dU`, then NR correction

**Workflow methods:**
- `position_u(live_plot)` — drives last node to `constants.POS_ENCASTREMENT1` in two passes: first with only `tita` locked on last node (x/y free), then with all DOFs locked; sends `motors.angles = [0,0,0,0]` on completion
- `condition_initiale(live_plot)` — initializes a vertical beam from `POS_ENCASTREMENT3`, calls `position_u()`, reads green marker, fixes plot limits to `x∈[-175,175], y∈[-175,30]` mm, resets `NINC=150`, `draw_every=15`
- `montrer_solution()` — prints final nodal state (x, y, tita, L, q, F) and redraws
- `parse_efforts(data)` — converts a JSON dict `{"Node0": {"Fx":…, "Fy":…, "M":…}, …}` to a global force vector of size `3*N_NODES`
- `get_position_point_vert()` — returns latest camera marker position (Nx2 array in mm) or `None`; thread-safe

**TextBox callbacks:**
- `_set_tita3(text)` — updates `beam.tita3`, computes `pos_enc3 = get_pos_encastrement3(tita3)`, sends motor command `motors.angles = [0, -tita1, 0, -tita3]`, runs displacement solve on node 0
- `_set_tita1(text)` — updates `beam.tita1`, computes `pos_enc1 = get_pos_encastrement1(tita1)`, sends motor command, runs displacement solve on last node

**Helper (module-level):**
`obtenir_liste_ddl_contraintes(ddl_contraintes, noeuds_contraintes)` — converts named node/direction constraint dicts into a flat list of global DOF indices. Supports negative node indices.

## Key Data Structures

- **DOF vector** `u`: flat array `[x0, y0, θ0, x1, y1, θ1, ..., xN, yN, θN]` — 3 DOFs per node
- **Local forces** `ql`: flat array `[N0, M1_0, M2_0, N1, M1_1, M2_1, ...]` — 3 per element
- **Stiffness per element** `C_all`: `(3, 3, N_ELEM)` array; element at `floor(N_ELEM/2)` uses `C2` (20× stiffer, rigid link), all others use `C1`
- **Base stiffness** `_C_base`: unscaled `C_all` before dividing by `L0`; rebuilt by `configuration_neutre()`
- **Free DOFs** `ddl`: index array obtained by deleting constrained indices from `arange(3*N_NODES)`
- **Motor angles** `tita1`, `tita3`: scalar floats on `Beam`, updated by TextBox callbacks before each solve
- **Marker position** `_tracker_pos`: latest camera tracker output, updated by background thread; accessed via `_tracker_lock`

## Newton-Raphson Loop Pattern

Both solve methods share the same structure per increment:

1. Apply load/displacement increment to `beam.u`
2. Call `actualiser_conf` → `actualiser_iforces` to get current `q`
3. Compute residual `R = q[ddl] - F[ddl]`
4. Iterate until `||R|| < tol` (max `maxiter` iterations):
   - `actualiser_ks()` to refresh `K`
   - `dUk[ddl] -= K[ddl,ddl]^-1 * R`
   - Update trial `u_cur`, recompute conf/iforces/R
5. On non-convergence: print diagnostic, call `montrer_solution()`, and `quit()`

Note: `solve_increment_charge` calls `actualiser_ks()` once before the NR loop to compute an initial `dU = K^-1 * dF` predictor step.

## Diagnostics

Non-convergence (`Pas de convergence` message) is most often fixed by:
- More increments (`NINC` ↑) — smaller steps
- More NR iterations (`maxiter` ↑)
- Looser tolerance (`tol` ↑, though less accurate)

The assembly loops in `actualiser_ks()` and `actualiser_iforces()` (`for i in range(N_ELEM)`) are the main performance bottleneck for large meshes.
