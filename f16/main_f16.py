"""
F-16  6-DoF  Scenario Simulation
==================================

Usage:
    python3 main_f16.py              # Scenario 1
    python3 main_f16.py 4            # Scenario 4
    python3 main_f16.py all          # All scenarios (no plot window)

Scenarios:
    1  Constant speed  — Straight and level  Mach 0.60  3000 m
    2  Accelerating    — t=3 s hard 90° right turn
    3  Sinusoidal      — left-right zig-zag jink
    4  Sharp-L         — 90° right + 500 m climb
    5  Beaming         — 90° lateral pass  (high LOS rate)
    6  Head-on break   — t=5 s sudden right break turn
    7  Look-down       — descent from 7500 m to 2500 m
    8  Last-ditch      — t=4 s combined bank + climb
    9  Energy climb    — Afterburner  3000→5500 m
   10  Waypoint nav    — 8×8 km square pattern
"""

import sys
import math
import os

from f16 import F16
from f16.scenarios import SCENARIOS, NAMES
from f16.f16_plotter import F16Plotter

os.makedirs('plots', exist_ok=True)

# ── simulation constants ───────────────────────────────────────────────────────
DT     = 0.005    # integration step - s  (200 Hz)
LOG_DT = 0.05     # logging interval  - s  (20 Hz)

# ── helpers ────────────────────────────────────────────────────────────────────

def _set(f16, path, value):
    """Set a value by following a dotted attribute path, e.g. 'propulsion.vmachcom'."""
    parts = path.split('.')
    obj = f16
    for p in parts[:-1]:
        obj = getattr(obj, p)
    setattr(obj, parts[-1], value)


def _bearing(sbel, wp):
    """Bearing from current NED position to waypoint (degrees from North)."""
    return math.degrees(math.atan2(wp[1] - sbel[1], wp[0] - sbel[0]))


def _hdist(sbel, wp):
    """Horizontal distance to waypoint (m)."""
    return math.hypot(wp[0] - sbel[0], wp[1] - sbel[1])


# ── main runner ────────────────────────────────────────────────────────────────

def run(scenario_no, verbose=True, show_plot=True):
    """Run one scenario. Returns (log, F16 object)."""
    spec  = SCENARIOS[scenario_no]
    name  = spec['name']
    t_end = spec['t_end']

    # ── build F-16 ────────────────────────────────────────────────────────────
    f16 = F16()

    # Apply initial conditions from scenario spec
    for attr, val in spec['init'].items():
        setattr(f16, attr, val)

    # Re-initialise with the new initial conditions
    f16.reset()

    # Apply autopilot settings
    for path, val in spec['autopilot'].items():
        _set(f16, path, val)

    # Waypoint management (scenario 10 only)
    waypoints = spec.get('_waypoints', None)
    wp_idx    = 0
    WP_RADIUS = 600.0

    if waypoints:
        f16.control.psivlcomx = _bearing(f16.SBEL, waypoints[0])

    # ── logging setup ──────────────────────────────────────────────────────────
    log      = []
    log_skip = max(1, round(LOG_DT / DT))
    step     = 0
    t        = 0.0

    events = sorted(spec['events'], key=lambda e: e[0])
    ev_ptr = 0   # index of the next pending event

    if verbose:
        print(f"\n{'='*72}")
        print(f"  F-16  Scenario {scenario_no}  —  {name.split('|')[0].strip()}")
        print(f"  T_END={t_end} s   DT={DT*1000:.0f} ms")
        print(f"{'='*72}")
        print(f"  {'t[s]':>7}  {'alt[m]':>8}  {'TAS':>7}  {'Mach':>6}  "
              f"{'α°':>6}  {'ψ°':>7}  {'φ°':>7}  {'nz[g]':>7}  {'thrust':>8}")
        print(f"  {'-'*76}")

    # ── simulation loop ────────────────────────────────────────────────────────
    while t < t_end:

        # Fire time-based events
        while ev_ptr < len(events) and t >= events[ev_ptr][0]:
            _, path, val = events[ev_ptr]
            _set(f16, path, val)
            if verbose:
                print(f"\n  [Event t={events[ev_ptr][0]:.1f} s]  {path} = {val}")
            ev_ptr += 1

        # Waypoint steering (scenario 10)
        if waypoints and wp_idx < len(waypoints):
            wp = waypoints[wp_idx]
            f16.control.psivlcomx = _bearing(f16.SBEL, wp)
            if _hdist(f16.SBEL, wp) < WP_RADIUS:
                if verbose:
                    print(f"\n  [WP {wp_idx} reached  t={t:.1f} s]  → WP {wp_idx+1}")
                wp_idx += 1
                if wp_idx < len(waypoints):
                    f16.control.psivlcomx = _bearing(f16.SBEL, waypoints[wp_idx])

        # Integrate one step
        f16.step(DT)
        t    += DT
        step += 1

        # Console progress line every 15 s
        if verbose and abs(round(t / 15.0) * 15.0 - t) < DT * 0.5:
            print(f"  {t:7.1f}  {f16.hbe:8.1f}  {f16.dvbe:7.2f}  {f16.mach:6.4f}  "
                  f"{f16.alphax:6.2f}  {f16.psivlx:7.1f}  {f16.phiblx:7.1f}  "
                  f"{f16.anx:7.3f}g  {f16.thrust/1e3:7.1f} kN")

        # Log sample
        if step % log_skip == 0:
            log.append({
                't'        : t,
                'hbe'      : f16.hbe,
                'dvbe'     : f16.dvbe,
                'mach'     : f16.mach,
                'pdynmc'   : f16.pdynmc / 1e3,
                'alphax'   : f16.alphax,
                'betax'    : f16.betax,
                'alppx'    : f16.alppx,
                'psiblx'   : f16.psiblx,
                'thtblx'   : f16.thtblx,
                'phiblx'   : f16.phiblx,
                'psivlx'   : f16.psivlx,
                'thtvlx'   : f16.thtvlx,
                'ppx'      : f16.ppx,
                'qqx'      : f16.qqx,
                'rrx'      : f16.rrx,
                'delax'    : f16.delax,
                'delex'    : f16.delex,
                'delrx'    : f16.delrx,
                'delacx'   : f16.delacx,
                'delecx'   : f16.delecx,
                'delrcx'   : f16.delrcx,
                'thrust'   : f16.thrust,
                'throttle' : f16.propulsion.throttle * 100.0,
                'power'    : f16.propulsion.power,
                'anx'      : f16.anx,
                'ayx'      : f16.ayx,
                'sbel_n'   : f16.SBEL[0],
                'sbel_e'   : f16.SBEL[1],
                'psivlcomx': f16.control.psivlcomx,
            })

    if verbose:
        print(f"\n{'='*72}")
        print(f"  Completed: t={t:.1f} s   alt={f16.hbe:.1f} m   "
              f"Mach={f16.mach:.4f}   alpha={f16.alphax:.2f}°   nz={f16.anx:.3f} g")
        print(f"{'='*72}\n")

    # ── render plots ───────────────────────────────────────────────────────────
    if log:
        F16Plotter(log, scenario_no, name).render(save=True, show=show_plot)

    return log, f16


def run_all():
    """Run all scenarios in sequence without opening any plot window."""
    print(f"\n{'#'*72}")
    print(f"  F-16  ALL SCENARIOS  ({len(SCENARIOS)} total)")
    print(f"{'#'*72}")
    for s in sorted(SCENARIOS):
        print(f"\n{'─'*50}  S{s}: {NAMES[s].split('|')[0].strip()}")
        run(s, verbose=False, show_plot=False)
        print(f"  S{s} done  →  plots/f16_s{s}_*.png")
    print(f"\n{'#'*72}")
    print("  All scenarios completed.")
    print(f"{'#'*72}\n")


# ── CLI entry point ────────────────────────────────────────────────────────────

if __name__ == '__main__':
    argv = sys.argv[1:]

    if not argv:
        run(1)
    elif argv[0] == 'all':
        run_all()
    else:
        try:
            s = int(argv[0])
        except ValueError:
            print(f"Error: '{argv[0]}' is not a valid scenario number.")
            print(__doc__)
            sys.exit(1)
        if s not in SCENARIOS:
            print(f"Error: Scenario {s} is not defined. Valid: {sorted(SCENARIOS)}")
            sys.exit(1)
        run(s)
