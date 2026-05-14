# SRAAM6 — 6-DoF Short-Range Air-to-Air Missile Simulation

A high-fidelity, 6-degrees-of-freedom missile intercept simulation framework developed as a senior capstone project. The repository contains two parallel implementations of the same physical model:

- **Python** — the primary research platform, featuring a modular architecture, multiple autopilot designs, Monte Carlo analysis, and Unity 3D export.
- **MATLAB/Simulink** — a block-diagram implementation of the same equations, used for cross-validation and control-design iteration.

---

## Example Outputs

### Engagement Trajectory — Scenario 4 (Sharp L-Maneuver)
![Trajectory S4](plots/s4_trajectory.png)

3D engagement geometry, top view (N-E plane), side view (N-Alt), and slant range to target over time. Target executes an 8-g right break followed by a 6-g pull-up; intercept at t = 5.31 s with a miss distance of 29.20 m.

### Autopilot Comparison — Accel vs NDI vs INDI (Scenario 3)
![Autopilot Comparison S3](plots/s3_cmp_Accel_vs_NDI_vs_INDI_overview.png)

Side-by-side overlay of all three autopilots on the same scenario: trajectory geometry, angle of attack, pitch rate, fin commands, and acceleration tracking error. INDI shows the tightest tracking error near CPA.

### Monte Carlo Miss-Distance Analysis — Scenario 1
![Monte Carlo S1](plots/s1_mc_dist.png)

Miss-distance histogram + KDE, empirical CDF, and violin plot for 25 runs per variant (NDI vs NDI-CoP). NDI-CoP achieves a CEP of 0.7 m vs NDI's 0.8 m under 15% aerodynamic model uncertainty and IMU noise.

### Weapon Employment Zone (WEZ)
![WEZ](plots/wez.png)

The WEZ diagram shows the missile's engagement envelope from the shooter's perspective (shooter at bottom, target ahead). It is computed by sweeping target position across the forward hemisphere and running two independent simulation passes:

| Region | Colour | Meaning |
|--------|--------|---------|
| **R_MAX** (blue solid line) | — | Maximum effective launch range against a non-maneuvering target. Firing beyond this line results in a miss regardless of target behaviour. |
| **Outer Launch Zone** | Light blue | Target can be engaged if it does not maneuver, but a 5-g evasive break allows escape. Kill probability is moderate (~30–60%). |
| **No Escape Zone (NEZ)** | Red | Target cannot escape even with a 5-g break maneuver. Firing from within the NEZ guarantees a high-probability kill (~85–95%). |
| **R_MIN** (dashed line) | — | Minimum safe launch range — inside this boundary the missile cannot arm or achieve seeker lock in time. |

The NEZ boundary is determined by a second simulation pass in which the target executes a maximum 5-g horizontal break starting at t = 1.5 s. The azimuth range spans ±90° from the shooter's heading, assuming the missile is aimed at the target at launch (off-boresight capability).

**Generated with:**
```bash
python3 lar.py wez --speed 200 --dr 2 --da 15 --rmax 25
```

---

## Table of Contents

1. [Overview](#overview)
2. [Repository Structure](#repository-structure)
3. [Physics & Modeling](#physics--modeling)
4. [Autopilot Designs](#autopilot-designs)
5. [Python Simulation](#python-simulation)
   - [Requirements](#requirements)
   - [Command-Line Usage](#command-line-usage)
   - [Programmatic Usage](#programmatic-usage)
   - [Configuration](#configuration)
   - [Outputs](#outputs)
6. [MATLAB / Simulink Simulation](#matlab--simulink-simulation)
   - [Running the Model](#running-the-model)
   - [Parameters](#parameters)
   - [NDI-CoP Variant](#ndi-cop-variant)
   - [Simulink Outputs](#simulink-outputs)
7. [Target Scenarios](#target-scenarios)
8. [Registered Comparison Tests](#registered-comparison-tests)
9. [Monte Carlo Analysis](#monte-carlo-analysis)
10. [Launch Acceptability Region & WEZ](#launch-acceptability-region--wez)
11. [Unity 3D Export](#unity-3d-export)

---

## Overview

SRAAM6 models the 6-DoF flight of a short-range, tail-controlled air-to-air missile engaging a maneuvering aerial target. The simulation integrates:

- Full rigid-body equations of motion (Newton–Euler) in NED (North-East-Down) coordinates
- Tabulated aerodynamic and propulsion decks (`.asc` lookup tables)
- A second-order fin actuator model
- An INS (Inertial Navigation System) error model with configurable IMU grade
- A kinematic IR seeker model with acquisition, lock, and blind-range phases
- Proportional Navigation and compensated Pro-Nav guidance laws
- Four inner-loop autopilot architectures (see below)
- 14 target engagement scenarios ranging from straight flight to 9-g last-ditch breaks

Both the Python integrator (1 kHz fixed-step) and the Simulink model share the same physical parameters, enabling direct cross-validation.

---

## Repository Structure

```
SRAAM_Simulation/
├── simulate.py          # Entry point — CLI, run(), compare(), monte_carlo()
├── config.py            # All tunable constants (single source of truth)
├── target.py            # 14 target engagement scenarios
├── tests.py             # Named comparison test registry
│
├── missile/             # Missile subsystem modules
│   ├── missile.py       # Top-level Missile class (assembles all subsystems)
│   ├── aerodynamics.py  # Aero coefficient lookup + angle-of-attack computation
│   ├── propulsion.py    # Thrust and mass depletion
│   ├── forces.py        # Body-frame force and moment assembly
│   ├── kinematics.py    # DCM update, Euler angles, velocity frame
│   ├── newton.py        # Translational equations of motion
│   ├── euler.py         # Rotational equations of motion
│   ├── actuator.py      # Second-order fin actuator dynamics
│   ├── control.py       # Autopilot (Rate / Accel / NDI / INDI / NDI-CoP)
│   ├── guidance.py      # Pro-Nav / compensated Pro-Nav guidance
│   ├── ins.py           # INS error model (gyro & accelerometer biases)
│   ├── sensor.py        # Kinematic IR seeker model
│   ├── environment.py   # ISA atmosphere, gravity
│   └── data/            # Tabulated aero & propulsion decks
│
├── utils/
│   ├── plotting.py      # SimPlotter, ComparisonPlotter, MCPlotter
│   └── plot_log.py      # Standalone log-file plot utility
│
├── unity/
│   └── unity_export.py  # JSON serialiser for Unity 3D playback
│
├── tools/
│   └── export_aero_to_mat.py  # Export aero tables to MATLAB .mat format
│
├── simulink/            # MATLAB/Simulink implementation
│   └── src/
│       ├── sraam.slx            # Main Simulink model
│       ├── sraam2023b.slx       # MATLAB R2023b compatible variant
│       ├── SRAAM6_params.m      # All simulation parameters
│       ├── run.m                # One-click run script
│       ├── plot_results.m       # Results plotting
│       ├── aerodynamics.m       # Aero coefficient S-function
│       ├── control_ndi.m        # NDI autopilot S-function
│       ├── control_ndi_cop.m    # NDI-CoP autopilot S-function
│       ├── control_roll.m       # Roll autopilot S-function
│       ├── guidance_pronav.m    # Pro-Nav guidance S-function
│       ├── ins.m                # INS error model S-function
│       ├── six_dof_deriv.m      # 6-DoF equations of motion
│       └── aero_tables.mat      # Pre-compiled aerodynamic lookup tables
│
└── logs/                # Generated CSV logs and Monte Carlo results
└── plots/               # Generated PNG figures
```

---

## Physics & Modeling

### Equations of Motion

The missile is modeled as a rigid body with 6 degrees of freedom. The state vector integrates at 1 kHz (Python) or equivalent Simulink solver step:

| Category | States |
|---|---|
| Position | NED position `(N, E, D)` in meters |
| Velocity | Body-frame velocity; derived airspeed, Mach, dynamic pressure |
| Attitude | Euler angles `(ψ, θ, φ)` via direction cosine matrix (DCM) |
| Angular rates | Body rates `(p, q, r)` in rad/s |
| Aerodynamics | Angle of attack `α`, sideslip `β`, total AoA `αp` |
| Propulsion | Thrust, mass, CG position |
| Actuator | Four independent fin angles with second-order dynamics |

### Aerodynamics

Aerodynamic coefficients `(CA, CN, CY, Cm, Cn, Cl)` are interpolated from tabulated decks as functions of Mach number, angle of attack, and control surface deflection. Dimensional stability and control derivatives `(dna, dma, dmq, dmd, dyb, dlnb, dlnr, dlnd, …)` are derived from these tables and used by the NDI/INDI controllers.

### Actuator

Each of the four fins is driven by an independent second-order transfer function:

- Natural frequency: **251 rad/s**
- Damping ratio: **0.7**
- Deflection limit: **±28°**
- Rate limit: **±600 °/s**

### INS Model

When `mins=1`, the INS module injects sensor errors from a strapdown IMU:

- Gyroscope bias and random walk noise (configurable IMU grade)
- Accelerometer bias
- Resulting attitude tilt errors `(RECE)`, velocity errors `(EVBE)`, and position errors `(ESTTC)` are propagated and exposed to the controller

The INS model is seeded with a configurable integer for deterministic Monte Carlo runs.

### Seeker

A kinematic IR seeker model (`mseek=2`) tracks the line-of-sight to the target:

- Acquisition at `RACQ` range after a dwell time `DTIMAC`
- Lock phase → switches guidance to terminal compensated Pro-Nav (`mguid=36`)
- Blind range `DBLIND` — seeker disabled in the very terminal phase

---

## Autopilot Designs

Four inner-loop autopilot architectures are implemented and compared:

| Mode | `maut` | Description |
|---|---|---|
| Rate Controller | `2` | Simple pitch/yaw rate feedback |
| Accel Controller | `3` | Pole-placement acceleration controller (classical CADAC-style) |
| NDI | `5` | Nonlinear Dynamic Inversion — inverts the full aerodynamic model |
| INDI | `6` | Incremental NDI — model-independent, uses measured body rates; inherently robust to aero model uncertainty |
| NDI-CoP | `7` | NDI reformulated about the Center of Percussion — eliminates the non-minimum-phase zero of a tail-controlled airframe |

All autopilots share the same outer guidance loop. The NDI and INDI designs use a three-loop cascade:

1. **Inner loop** — body moment / angular rate (bandwidth `WN_NDI_Q = 50 rad/s`)
2. **Middle loop** — angle-of-attack / sideslip (`K_ALPHA = 10 /s`)
3. **Outer loop** — normal/lateral acceleration command from Pro-Nav guidance

---

## Python Simulation

### Requirements

```
Python ≥ 3.9
numpy
scipy
matplotlib
```

Install dependencies:

```bash
pip install numpy scipy matplotlib
```

### Command-Line Usage

**Single scenario run** (saves CSV + 2 PNGs, opens plot window):

```bash
python3 simulate.py          # scenario 1 (default)
python3 simulate.py 3        # scenario 3
```

**Run all scenarios** (summary table):

```bash
python3 simulate.py all
```

**Comparison test** (two or three autopilot variants side by side):

```bash
python3 simulate.py test ndi_vs_indi          # scenario 1
python3 simulate.py test ndi_vs_indi 3        # scenario 3
python3 simulate.py test all_autopilots all   # all scenarios, table only
```

**Monte Carlo analysis**:

```bash
python3 simulate.py mc                        # S1, default variants, 200 runs
python3 simulate.py mc 3                      # S3
python3 simulate.py mc ndi_vs_indi 3 50       # S3, NDI vs INDI, 50 runs
python3 simulate.py mc all                    # all scenarios, CEP table
```

**Unity 3D export**:

```bash
python3 simulate.py unity 3                   # scenario 3
python3 simulate.py unity 3 5                 # scenario 3, every 5th frame
```

### Programmatic Usage

```python
import simulate

# Single run
log, result, cpa = simulate.run(scenario=3)
# cpa = {'min_dist': 0.19, 't_cpa': 5.236, 'pos': (x, y, h)}

# Run with config overrides
log, result, cpa = simulate.run(
    scenario=3,
    cfg={'maut': 6, 'mseek': 2, 'mins': 1, 'ins_seed': 42},
)

# Registered comparison test
simulate.test('ndi_vs_indi', scenario=3)

# Ad-hoc comparison
simulate.compare(
    scenario=5,
    test_name='my_sweep',
    variants={'low-BW': {'maut': 5}, 'high-BW': {'maut': 6}},
)

# Run all scenarios
simulate.run_all(scenarios=[1, 3, 5], save_csvs=True, show_plots=False)

# Monte Carlo
results = simulate.monte_carlo(scenario=1)
# results = {
#   'Accel': {'miss_dists': [...], 'mean': 1.4, 'cep': 0.9, 'p90': 3.1, ...},
#   'NDI':   {...},
#   'INDI':  {...},
# }

# Monte Carlo across all scenarios
simulate.monte_carlo_all_scenarios(n_runs=50, show=False)
```

### Configuration

All parameters are centralised in [config.py](config.py). Key knobs:

| Parameter | Default | Description |
|---|---|---|
| `DT` | `0.001 s` | Integration time step |
| `T_END` | `25.0 s` | Maximum simulation time |
| `MAUT` | `5` (NDI) | Autopilot mode |
| `MINS` | `0` | INS mode — `0` ideal, `1` real IMU errors |
| `MSEEK` | `0` | Seeker — `0` off, `2` kinematic IR |
| `GNAV` | `4.0` | Proportional navigation gain |
| `HIT_R` | `0.0 m` | Intercept radius; `0` → pure CPA mode |
| `MASS` | `91.95 kg` | Initial missile mass |
| `AI33` | `59.80 kg·m²` | Pitch/yaw moment of inertia |
| `WN_NDI_Q` | `50 rad/s` | NDI inner-loop bandwidth |
| `K_ALPHA` | `10 /s` | NDI alpha-loop gain |

A subset of these can be overridden per-run at runtime without editing the file:

```python
cfg = {'maut': 6, 'mins': 1, 'ins_seed': 42, 'mseek': 2, 'gnav': 5.0}
log, result, cpa = simulate.run(scenario=3, cfg=cfg)
```

### Outputs

| Type | Path | Content |
|---|---|---|
| Simulation log | `logs/sim_log_<tag>.csv` | Per time-step state, guidance, control, INS errors |
| Comparison plot | `plots/s<N>_<test>_overview.png` | Trajectory geometry + acceleration response |
| Control plot | `plots/s<N>_<test>_control.png` | Body rates, Euler angles, fin deflections |
| INS error plot | `plots/s<N>_<test>_ins.png` | Position / velocity / attitude INS errors |
| MC distribution | `plots/s<N>_mc_dist.png` | Histogram + KDE, empirical CDF, violin |
| MC statistics | `plots/s<N>_mc_stats.png` | Formatted statistics table figure |
| MC per-run data | `logs/mc_runs_s<N>.csv` | variant, run, miss_dist, result, ins_seed |
| MC aggregate stats | `logs/mc_stats_s<N>.csv` | mean, std, CEP, P90, P95, Pk per variant |
| Unity JSON | `logs/unity/s<N>.json` | Frame-by-frame playback data for Unity |

---

## MATLAB / Simulink Simulation

The Simulink model in `simulink/src/` implements the same 6-DoF equations as a block diagram. It was developed for cross-validation against the Python reference and as an independent design tool during autopilot development.

### Running the Model

Open MATLAB, navigate to `simulink/src/`, and run:

```matlab
run('run.m')
```

`run.m` performs three steps:
1. Loads all parameters from `SRAAM6_params.m` into the workspace
2. Executes `sim('sraam')` to run the Simulink model
3. Calls `plot_results(simOut, SCENARIO, false)` to display results

### Parameters

All settings are controlled from a single file: `SRAAM6_params.m`.

**Scenario selection** (`SCENARIO = 1` to `14`):

| No | Description |
|----|-------------|
| 1 | Straight flight, 6 km lateral offset |
| 2 | 5-g right break turn (t > 1.5 s) |
| 3 | Sinusoidal jink, 3-g lateral + 2-g vertical |
| 4 | Sharp L-maneuver — 8-g break + 6-g pull-up |
| 5 | Beaming / notch, 90° crossing |
| 6 | Head-on approach + 7-g right break at t = 3 s |
| 7 | Look-down engagement, target at 7500 m |
| 8 | Last-ditch 9-g combined break |
| 9 | Afterburner energy-bleed climbing turn |
| 10 | Sustained 4-g coordinated turn |
| 11 | Long range (12 km), straight |
| 12 | Very long range (15 km), sinusoidal jink |
| 13 | Tail-chase escape |
| 14 | Long range beam + hard break |

**INS mode**:

```matlab
MINS = 0;   % 0 = ideal (ground truth)
MINS = 1;   % 1 = real IMU sensor errors
```

When `MINS = 1`, the IMU quality is set with `IMU_GRADE`:

| Grade | Gyro Bias | Accel Bias | Effect |
|-------|-----------|------------|--------|
| 1 (Tactical) | 0.66 deg/hr | 363 μg | Negligible |
| 2 (Industrial) | 206 deg/hr | 51 mg | ~50–200 m position drift |
| 3 (Low-cost) | 2000 deg/hr | 200 mg | Significant miss-distance growth |

**Other key parameters**:

```matlab
GNAV   = 4.0;    % Pro-Nav gain (typical range: 3–5)
HIT_R  = 0;      % 0 = run until CPA;  >0 = stop on intercept
T_END  = 25.0;   % maximum simulation time [s]
```

### NDI-CoP Variant

The Simulink model includes both the standard NDI and the NDI-CoP (Center of Percussion) autopilot. NDI-CoP reformulates the outer acceleration loop about the Center of Percussion, which eliminates the non-minimum-phase zero inherent to tail-controlled missile airframes. To switch:

1. Open the **Control Subsystem** block in `sraam.slx`
2. Right-click the **NDI block** → **Comment Out**
3. Right-click the **NDI-CoP block** → **Uncomment**
4. Run `run.m`

### Simulink Outputs

When `plot_results` is called with the third argument set to `true`, three figures are generated:

- **Figure 1** — Trajectory (3D view, top view, side view, range, G-force)
- **Figure 2** — State variables (airspeed, attitude angles, acceleration, fin deflections)
- **Figure 3** — Actuator detail (4-fin command vs. actual vs. error)

Numerical results are printed to the MATLAB console:

```
Miss distance : X.XX m
t_CPA         : X.XX s
```

---

## Target Scenarios

The Python `Target` class defines 14 engagement scenarios. All use the NED frame; altitude is `hbe = -pos[2]`.

| Scenario | Description |
|----------|-------------|
| 1 | Constant-speed straight flight, lateral offset |
| 2 | Accelerating escape — 5-g right break turn at t > 1.5 s |
| 3 | Sinusoidal jink — 3-g lateral + 2-g vertical weave |
| 4 | Sharp L-maneuver — t > 2 s: 8-g right break + 6-g pull-up |
| 5 | Beaming / notch — 90° crossing, high LOS rate |
| 6 | Head-on + break — head-on approach, t = 3 s 7-g right break |
| 7 | Look-down engagement — target at 7500 m, dive intercept |
| 8 | Last-ditch break — t = 4 s ~9-g combined break (endgame) |
| 9 | Energy-bleed climb — afterburner ~3-g climbing turn |
| 10 | Coordinated turn — sustained 4-g target turn |
| 11 | Long-range straight — 12 km offset |
| 12 | Very long range — 15 km, sinusoidal jink |
| 13 | Tail-chase escape — target fleeing |
| 14 | Long-range beam — 10 km, 90° notch |

Scenarios 2, 4, and 6 use velocity-perpendicular (centripetal) acceleration to conserve airspeed through the turn, which is physically representative of a coordinated aircraft maneuver.

---

## Registered Comparison Tests

Named tests are defined in [tests.py](tests.py) and run via `simulate.test()` or the CLI.

| Test Name | Variants | Purpose |
|-----------|----------|---------|
| `accel_vs_ndi` | Accel vs NDI | Classical vs nonlinear |
| `ndi_vs_indi` | NDI vs INDI | Full model-inversion vs incremental |
| `accel_vs_indi` | Accel vs INDI | Classical vs incremental |
| `ndi_vs_ndi_cop` | NDI vs NDI-CoP | Standard vs Center-of-Percussion NDI |
| `all_autopilots` | Accel + NDI + INDI | 3-way sweep |
| `ins_on_off_ndi` | NDI INS-off vs NDI INS-on | INS error sensitivity (NDI) |
| `ins_on_off_indi` | INDI INS-off vs INDI INS-on | INS error sensitivity (INDI) |
| `seeker_on_off_indi` | INDI seeker-off vs seeker-on | Terminal Pro-Nav effect |

To add a custom test, add an entry to the `TESTS` dict in `tests.py` — no changes to `simulate.py` are required.

---

## Monte Carlo Analysis

The Monte Carlo runner independently samples two sources of uncertainty per run:

- **INS sensor biases** — gyro and accelerometer biases drawn from the configured IMU noise model
- **Aerodynamic model uncertainty** — each dimensional derivative (e.g. `dna`, `dma`, `dmq`, `dmd`, …) is multiplied by a factor `~ N(1, σ²)`

The physical aerodynamic forces (`CA`, `CN`, `CY`) are **not** perturbed — only the controller's internal model is. This means:
- **NDI** (which inverts the full model) is degraded by model uncertainty
- **INDI** (which is incremental and model-independent) is inherently robust

Default configuration (`config.py`):

| Parameter | Default | Description |
|---|---|---|
| `MC_N_RUNS` | `200` | Runs per variant per scenario |
| `MC_AERO_SIGMA` | `0.15` | Aero derivative std dev (15%) |
| `MC_INS_NOISE` | `True` | Inject IMU errors each run |
| `MC_BASE_SEED` | `0` | RNG seed for reproducibility |
| `MC_VARIANTS` | `{Accel, NDI, INDI, NDI-CoP}` | Default autopilot sweep |

Reported statistics per variant: **Mean**, **Std**, **Min**, **Max**, **CEP** (50th percentile), **P90**, **P95**, **Pk<5m**, **Pk<10m**, **Pk<20m**.

> **Runtime note:** each run takes approximately 9 s on a single core (Python 1 kHz integrator). A full sweep of 200 runs × 3 variants ≈ 90 minutes. Use `n_runs=50` for quick sanity checks.

---

## Launch Acceptability Region & WEZ

The `lar.py` module computes and visualises the missile's engagement envelope by running large batches of simulations in parallel across a swept target-position grid.

### Launch Acceptability Region (LAR)

The LAR answers: *"From which positions can the missile successfully intercept the target?"* It sweeps all 360° of azimuth and a configurable range interval, classifying each grid point as in-LAR (miss ≤ threshold) or out-of-LAR.

```bash
# 360° polar LAR, head-on target, coarse grid (~60 runs, ~2 min)
python3 lar.py --aspect 180 --speed 200 --dr 4 --da 30

# Medium resolution
python3 lar.py --aspect 180 --speed 200 --dr 2 --da 15

# INDI autopilot, fine grid
python3 lar.py --maut 6 --dr 1 --da 10 --rmax 25
```

Output: `plots/lar_asp<aspect>_v<speed>.png` — left panel: binary hit/miss; right panel: miss-distance heatmap with LAR boundary contour.

### Weapon Employment Zone (WEZ)

The WEZ extends the LAR with a second simulation pass using a maneuvering target (configurable g-load break), producing the classic fighter pilot's engagement envelope display:

```bash
# Forward hemisphere WEZ, 5-g NEZ
python3 lar.py wez --speed 200 --dr 2 --da 15 --rmax 25
```

Output: `plots/wez.png`

### Programmatic Usage

```python
from lar import compute_lar, plot_lar, compute_wez, plot_wez
import numpy as np

# LAR
data = compute_lar(
    ranges_km        = np.arange(2, 26, 2),
    azimuths_deg     = np.arange(0, 360, 15),
    tgt_speed_ms     = 200,
    tgt_aspect_deg   = 180,
    miss_threshold_m = 20,
    cfg = {'maut': 6, 'mins': 0, 'mseek': 0},
)
plot_lar(data)

# WEZ
wez = compute_wez(
    azimuths_deg     = np.arange(-90, 91, 10),
    tgt_speed_ms     = 200,
    miss_threshold_m = 20,
    nez_g            = 5.0,
    cfg = {'maut': 5, 'mins': 0},
)
plot_wez(wez)
```

> **Performance note:** Each simulation run takes ~3–8 s (Python 1 kHz integrator). A 288-run LAR sweep with 11 parallel workers completes in approximately 3–5 minutes. Use `--dr 4 --da 30` for a quick 60-run estimate.

---

## Unity 3D Export

The simulation can export a frame-by-frame JSON file for 3D playback in Unity:

```bash
python3 simulate.py unity 3        # scenario 3, every frame
python3 simulate.py unity 3 5      # scenario 3, every 5th frame (smaller file)
```

Output: `logs/unity/s<N>.json`

The JSON includes missile and target position, attitude, fin angles, and engagement metadata at each exported timestep. A stride value > 1 reduces file size while preserving visual fidelity at typical playback frame rates.

---

## References

Zipfel, P. H. (2007). *Modeling and Simulation of Aerospace Vehicle Dynamics* (2nd ed.). AIAA Education Series. American Institute of Aeronautics and Astronautics. ISBN 978-1-56347-621-3.

The equations of motion, aerodynamic coefficient conventions, INS error model, and guidance law implementations in this project follow the CADAC (Computer-Aided Design of Aerospace Concepts) framework described in this book.
