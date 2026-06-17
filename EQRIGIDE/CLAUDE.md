# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This repository is for the **EQRIGIDE** robot — a 2-DOF parallel robot (PPE EMIO project) with two prismatic-rotational arms that share a common end-effector. All working code lives in `Codes/CodesFinales/`. The `Codes/` root contains older prototype scripts. `PiecesEQRIGIDE/` holds the physical design files (CATIA, STL, 3MF, PDF).

## Environment Setup

All Python scripts must run from within the `Codes/CodesFinales/` directory so that relative imports resolve correctly.

```bash
cd Codes/CodesFinales
source .env/bin/activate
```

Install dependencies:
```bash
pip install -r requirements.txt
```

Key packages: `numpy`, `matplotlib`, `emioapi` (EMIO hardware SDK), `dynamixel-sdk`, `pyserial`, `opencv-python`, `pyrealsense2`.

## Running the Code

**Main GUI (robot control interface):**
```bash
python InterfaceEMIO.py
```
This connects to the physical EMIO robot via `EmioAPI`. If the robot is not plugged in, it falls back to simulation/graphics-only mode.

**Trajectory generation scripts** (compute IK, animate, export CSV):
```bash
python cercle.py   # Circle trajectory
python carre.py    # Square trajectory
```
These generate a `trajectoire_cercle.csv` file loadable by `InterfaceEMIO.py`.

## Architecture

All modules import `parametres as p` for robot geometry constants.

### `parametres.py` — Robot geometry
Defines all physical constants: `L` (half-base width, 100 mm), `r` (proximal link radius, 25 mm), `l_len` (end-effector half-length, 14 mm), `dmax`/`dmin` (prismatic joint travel limits). Derives `r_eq_max`, `r_eq_min`, and workspace boundary corners `S1–S4`.

### `calculer_GD2.py` — Forward kinematics (MGD)
`calculer_GD2(th1, th3)` → `(q1, q3, [x, y, alpha, th])`. Takes joint angles (radians), returns Cartesian end-effector position plus orientation. Raises `ValueError` if the computed point falls outside the workspace. Only accepts scalar inputs, not arrays.

### `calculer_GI2.py` — Inverse kinematics (MGI)
`calculer_GI2(x, y)` → `(q1, q3, [x, y, alpha, th])`. Accepts scalars or arrays. Raises `ValueError` if the input point is outside the workspace.

### `verifierSpTr.py` — Workspace check
`verifierSpTr(x, y)` → `1` (inside) or `0` (outside). Uses the annular workspace defined by `r_eq_min`/`r_eq_max` centred on each motor base, and rejects `y > 0`.

### `verifierSpTr_Trayectoire.py`
`verifierSpTr_Trayectoire(x_arr, y_arr, plot)` — validates an entire trajectory array and optionally plots in/out-of-workspace points.

### `calc_Angles.py`
`calculer_Angles(q1, q3)` → `(alpha, th)`. Computes end-effector orientation `alpha` (azimuth of the arm vector) and absolute angle `th = -th1 - th3 - π/2`.

### `animate.py`
`animate(Q1_total, Q3_total, trajectoire)` — matplotlib animation of the full robot over a sequence of joint states. Pass `trajectoire=1` to trace the end-effector path.

### `graph_temp.py`
Plots joint-space and task-space trajectories as static time-series graphs.

### `InterfaceEMIO.py`
Tkinter GUI with two control modes (joint space θ₁/θ₃ sliders, and Cartesian X/Y sliders). Bidirectional: moving either input recalculates the other via MGD or MGI. Validates every state against `verifierSpTr` before updating the display or sending to hardware. CSV trajectory playback via `root.after()` loop at 50 ms steps. Motor commands use `emio.motors.angles = [0, -th1, 0, -th3]` (index 0 = motor 1, index 2 = motor 3; angles are negated).

## Coordinate Convention

- Origin at centre of the robot base.
- Motor 1 is at `(+L, 0)`, Motor 3 at `(-L, 0)`.
- Y axis points downward in the physical robot (negative Y values are below the base).
- Joint angles `th1`, `th3` are in radians; the GUI sliders display degrees internally but convert before calling kinematics.
- Prismatic joint values `d1`, `d3` are in millimetres.
