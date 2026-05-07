"""
F-16 Evasion Scenario Definitions  (missile-target engagement geometry)
=========================================================================

Starting positions / velocities match target.py exactly (NED, m, m/s) so
the F-16 can serve as a 6-DoF replacement for the kinematic target.

Evasion strategy — why NOT mautp=3 direct-G commands
------------------------------------------------------
Commanding a sudden large ancomx (e.g. 1g → 7g in one step) causes the
pitch-accel controller's feed-forward integrator to wind up and then
discharge a nose-DOWN pulse that departs the aircraft.

Instead we use the natural bank-induced G mechanism:
  maut = 45  (heading hold + altitude hold)  throughout
  philimx = 85°  →  during a hard turn the bank reaches 85°
  altitude hold demands  1/cos(85°) ≈ 11.5 g  to maintain altitude
  anlimpx = 9.0g  clips the command → sustained 9 g  without instability

A simultaneous large altcom step adds a climb G component on top,
maximising the total load factor challenge for the missile.

The max_altrate clamp in _altitude() (50 m/s) keeps the climb G bounded
(≈ 2-3 g additional) while allowing aggressive altitude changes.
"""

import math

# ── engagement-geometry base (matches target.py) ──────────────────────────────
_ENG_INIT = {
    'SBEL_INIT': [6000.0, 1500.0, -10000.0],  # NED  (alt = 10 000 m)
    'dvbe'     : 200.0,                         # ~Mach 0.67 at 10 km
    'alpha0x'  : 4.0,                           # trim AoA at 10 km / 200 m/s
    'beta0x'   : 0.0,
    'psiblx'   : 180.0,                         # heading South (toward missile)
    'thtblx'   : 4.0,
    'phiblx'   : 0.0,
    'ppx': 0.0, 'qqx': 0.0, 'rrx': 0.0,
}

# ── cruise autopilot defaults ──────────────────────────────────────────────────
_CRUISE_AP = {
    'propulsion.mprop'   : 2,
    'propulsion.vmachcom': 0.67,
    'propulsion.gmach'   : 4.0,
    'propulsion.ki_mach' : 0.35,
    'control.maut'       : 45,        # heading hold + altitude hold
    'control.mroll'      : 0,
    'control.psivlcomx'  : 180.0,    # hold South heading
    'control.altcom'     : 10000.0,  # hold 10 000 m
    'control.gainalt'    : 0.07,
    'control.gainaltrate': 0.25,
    'control.wrcl'       : 3.0,
    'control.zrcl'       : 0.7,
    'control.facthead'   : 0.4,
    # G limits are set based on realistic aerodynamic capability at 10 km:
    #   q ≈ 8 280 Pa @ 200 m/s  →  CZ_max ≈ 1.37 (at α=20° stall)
    #   G_max = CZ_max * q * S / (m * g) ≈ 3.4 g
    # philimx=65° → 1/cos(65°) ≈ 2.4 g sustained level turn (safe margin)
    # anlimpx=3.5 g → allows brief 3.5 g peaks during evasive pull-ups
    'control.philimx'    : 65.0,
    'control.anlimpx'    : 3.5,
    'control.anlimnx'    : 2.0,
}


def _ic(overrides=None):
    d = dict(_ENG_INIT)
    if overrides:
        d.update(overrides)
    return d


def _ap(overrides=None):
    d = dict(_CRUISE_AP)
    if overrides:
        d.update(overrides)
    return d


def _hdg(vn, ve):
    """Heading in degrees (from North) for a given NED velocity."""
    return math.degrees(math.atan2(ve, vn))


# ── scenario table ─────────────────────────────────────────────────────────────
SCENARIOS = {

    # 1 — No evasion: straight constant-speed flight  (easiest for missile)
    #     target.py S1: (6000, 1500, -10000)  vel (-200, 0, 0)
    1: {
        'name'  : 'No evasion  |  Straight & level  200 m/s  10 000 m',
        't_end' : 40.0,
        'init'  : _ic(),
        'autopilot': _ap(),
        'events': [],
    },

    # 2 — Accelerating lateral sprint: t=1.5 s → hard right, Mach hold 0.72
    #     target.py S2: t>1.5 s  5 g East sprint
    #     90° heading change → bank → altitude hold drives ~5-6 g
    2: {
        'name'  : 'Accelerating escape  |  t=1.5 s  90° right break  ~5 g',
        't_end' : 35.0,
        'init'  : _ic(),
        'autopilot': _ap({'propulsion.vmachcom': 0.70,
                          'control.philimx'    : 70.0}),  # moderate bank → ~3g
        'events': [
            (1.5, 'control.psivlcomx',  90.0),  # turn right 90° (South → West)
        ],
    },

    # 3 — Sinusoidal jink: alternating 45° heading reversals every 8 s
    #     target.py S3: 3 g lateral + 2 g vertical weave
    3: {
        'name'  : 'Sinusoidal jink  |  ±45° heading reversals  ~3 g',
        't_end' : 50.0,
        'init'  : _ic(),
        'autopilot': _ap({'control.philimx': 60.0}),  # ~2 g in turns
        'events': [
            ( 1.5, 'control.psivlcomx', 135.0),   # first jink right
            ( 9.5, 'control.psivlcomx', 225.0),   # jink left
            (17.5, 'control.psivlcomx', 135.0),
            (25.5, 'control.psivlcomx', 225.0),
            (33.5, 'control.psivlcomx', 180.0),   # recover
        ],
    },

    # 4 — Sharp L-manoeuvre: t=2 s → 8 g right brake, t=4 s → hard climb
    #     target.py S4: t=2..3.5 8g right, t=3.5..4.8 6g up
    #     Large heading step → 85° bank → ~9 g; then altcom step → climb
    4: {
        'name'  : 'Sharp L-manoeuvre  |  t=2 s ~8 g right  t=4 s hard climb',
        't_end' : 30.0,
        'init'  : _ic(),
        'autopilot': _ap(),
        'events': [
            (2.0, 'control.psivlcomx', 90.0),    # hard right — bank → ~9 g
            (4.0, 'control.altcom',   12000.0),  # simultaneous climb command
            (4.0, 'control.psivlcomx', 180.0),  # level the turn, pitch up
        ],
    },

    # 5 — Beaming / notch: 90° crossing (maximum LOS rate for seeker)
    #     target.py S5: (5500,-2000,-10000) vel (0, 300, 0) → heading East
    5: {
        'name'  : 'Beaming / notch  |  90° crossing  high LOS rate  ~Mach 0.90',
        't_end' : 40.0,
        'init'  : _ic({
            'SBEL_INIT': [5500.0, -2000.0, -10000.0],
            'dvbe'     : 270.0,
            'psiblx'   : 90.0,    # heading East — already beaming
            'alpha0x'  : 5.0,
            'thtblx'   : 5.0,
        }),
        'autopilot': _ap({
            'propulsion.vmachcom': 0.90,
            'control.psivlcomx'  : 90.0,
            'control.altcom'     : 10000.0,
            # At 270 m/s: q ≈ 15 000 Pa → G_max ≈ 6 g  (safer headroom)
            'control.philimx'    : 75.0,
            'control.anlimpx'    : 5.0,
        }),
        'events': [
            # Maintain beam; late reversal to deny closing geometry
            (20.0, 'control.psivlcomx', 0.0),    # late turn back
        ],
    },

    # 6 — Head-on + break: t=3 s → 7 g right break from high-speed head-on
    #     target.py S6: (7000, 0, -10000) vel (-300, 0, 0), t=3 s 7g right
    6: {
        'name'  : 'Head-on + break  |  t=3 s  ~7 g right break',
        't_end' : 30.0,
        'init'  : _ic({
            'SBEL_INIT': [7000.0, 0.0, -10000.0],
            'dvbe'     : 270.0,
            'psiblx'   : 180.0,
            'alpha0x'  : 5.0,
            'thtblx'   : 5.0,
        }),
        'autopilot': _ap({
            'propulsion.vmachcom': 0.90,
            'control.psivlcomx'  : 180.0,
            'control.altcom'     : 10000.0,
            # At 270 m/s / 10 km: q ≈ 15 000 Pa → G_max ≈ 6 g
            'control.philimx'    : 75.0,
            'control.anlimpx'    : 5.0,
        }),
        'events': [
            (3.0, 'control.psivlcomx', 90.0),    # 90° right break → steep bank
        ],
    },

    # 7 — Look-down: 7500 m altitude, evasive climbing turn at t=3 s
    #     target.py S7: (5500, 1000, -7500) vel (-220, 0, 0)
    7: {
        'name'  : 'Look-down engage  |  7500 m  evasive climbing right turn',
        't_end' : 35.0,
        'init'  : _ic({
            'SBEL_INIT': [5500.0, 1000.0, -7500.0],
            'dvbe'     : 220.0,
            'psiblx'   : 180.0,
            'alpha0x'  : 4.5,
            'thtblx'   : 4.5,
        }),
        'autopilot': _ap({
            'propulsion.vmachcom': 0.75,
            'control.psivlcomx'  : 180.0,
            'control.altcom'     : 7500.0,
        }),
        'events': [
            (3.0, 'control.psivlcomx', 90.0),    # right break turn
            (3.0, 'control.altcom',    9500.0),  # simultaneous climb to 9500 m
        ],
    },

    # 8 — Last-ditch break: t=4 s → maximum ~9 g combined bank + climb
    #     target.py S8: same S1 geometry, t=4 s ~9 g combined
    #
    #     Mechanism: 120° heading step → bank reaches 85° (philimx)
    #     Altitude hold at 85° bank demands 1/cos(85°)≈11.5g → clipped to 9g.
    #     Simultaneous altcom jump adds a climb component.
    #     Result: sustained ~9 g  without integrator windup / departure.
    8: {
        'name'  : 'Last-ditch break  |  t=4 s  ~9 g bank-induced + climb',
        't_end' : 25.0,
        'init'  : _ic(),
        'autopilot': _ap(),   # uses defaults: philimx=65°, anlimpx=3.5g at 10km
        'events': [
            (4.0, 'control.psivlcomx', 60.0),    # 120° right heading change
            (4.0, 'control.altcom',  12500.0),   # hard climb on top of turn
        ],
    },

    # 9 — Energy-bleed climb: afterburner + aggressive climbing right turn
    #     target.py S9: (6500, 800, -10000) vel (-250, 0, 0), lift+drag climb
    9: {
        'name'  : 'Energy climb  |  afterburner + climbing right jink',
        't_end' : 45.0,
        'init'  : _ic({
            'SBEL_INIT': [6500.0, 800.0, -10000.0],
            'dvbe'     : 250.0,
            'psiblx'   : 180.0,
            'alpha0x'  : 5.0,
            'thtblx'   : 5.0,
        }),
        'autopilot': _ap({
            'propulsion.vmachcom': 0.85,
            'control.psivlcomx'  : 180.0,
            'control.altcom'     : 10000.0,
        }),
        'events': [
            (2.0,  'control.altcom',   13000.0),  # aggressive climb
            (2.0,  'control.psivlcomx', 90.0),   # simultaneous right turn
            (15.0, 'control.psivlcomx', 270.0),  # reverse to left
            (28.0, 'control.psivlcomx', 180.0),  # recover
        ],
    },

    # 10 — Coordinated 4 g sustained turn
    #      target.py S10: (10000,-1000,-12000) vel (-180, 120, 0) → banked turn
    10: {
        'name'  : 'Coordinated turn  |  sustained ~4 g  12 000 m',
        't_end' : 50.0,
        'init'  : _ic({
            'SBEL_INIT': [10000.0, -1000.0, -12000.0],
            'dvbe'     : math.hypot(180.0, 120.0),  # ≈ 216 m/s
            'psiblx'   : _hdg(-180.0, 120.0),       # ≈ 146° SSE
            'alpha0x'  : 4.0,
            'thtblx'   : 4.0,
        }),
        'autopilot': _ap({
            'propulsion.vmachcom': 0.72,
            'control.altcom'     : 12000.0,
            'control.psivlcomx'  : _hdg(-180.0, 120.0),
            'control.philimx'    : 65.0,   # ~2.4 g in sustained turn
        }),
        'events': [
            # Tighten the turn to increase G as missile approaches
            (5.0,  'control.psivlcomx', _hdg(-180.0, 120.0) + 60.0),
            (15.0, 'control.psivlcomx', _hdg(-180.0, 120.0) + 120.0),
            (25.0, 'control.psivlcomx', _hdg(-180.0, 120.0) + 180.0),
        ],
    },
}

NAMES = {s: d['name'] for s, d in SCENARIOS.items()}
