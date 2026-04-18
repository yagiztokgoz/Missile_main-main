"""
================================================================================
 SRAAM6 — Central Configuration
================================================================================

All tunable parameters live in this file. simulate.py, missile.py, and
control.py import from here. To change simulation behaviour, edit THIS file
only — do not scatter constants across other modules.

Runtime override:
    The test runner (simulate.test / simulate.compare) can override a subset
    of these values per-variant via a cfg dict, e.g.
        cfg = {'maut': 6, 'mseek': 2, 'mins': 1, 'ins_seed': 42}
    See simulate.run() for the list of supported override keys.

Style:
    - Each section has a labelled divider (── Section ──)
    - Names are UPPER_CASE constants
    - Units are noted inline (deg, rad/s, kg·m², g's, ...)
================================================================================
"""

import os

# ── Directories ───────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUTDIR   = os.path.join(BASE_DIR, 'logs')     # CSV log outputs
PLOTDIR  = os.path.join(BASE_DIR, 'plots')    # PNG plot outputs
DATA_DIR = os.path.join(BASE_DIR, 'data')

AERO_DECK = os.path.join(DATA_DIR, 'sraam6_aero_deck.asc')
PROP_DECK = os.path.join(DATA_DIR, 'sraam6_prop_deck.asc')


# ── Simulation loop ───────────────────────────────────────────────────────────
DT     = 0.001    # integration step - s
T_END  = 25.0     # max simulation time - s
LOG_DT = 0.01     # logging sample interval - s
HIT_R  = 0.0     # intercept radius - m   (0 → pure CPA mode)

# CPA early-exit thresholds
CPA_GROW_M   = 1.0    # bail out once min_dist grows by this much - m
CPA_MIN_TIME = 0.5    # disable CPA-exit before this time - s


# ── Physical constants ────────────────────────────────────────────────────────
AGRAV = 9.80665   # gravitational acceleration - m/s²


# ── Initial (launch) state ────────────────────────────────────────────────────
SBEL_INIT     = (0.0, 0.0, -10000.0)   # NED position - m (down negative → altitude)
ATTITUDE_INIT = (0.0, 0.0, 0.0)        # (psi, theta, phi) - deg
AERO_INIT     = (0.0, 0.0)             # (alpha0, beta0) - deg
SPEED_INIT    = 250.0                  # initial airspeed - m/s


# ── Aerodynamic airframe ──────────────────────────────────────────────────────
ALP_LIMX = 46.0     # total angle-of-attack limit - deg
REFL     = 0.1524   # reference length for moment derivatives - m
REFA     = 0.01824  # reference area for aero coefficients - m²


# ── Propulsion / Mass / Inertia ───────────────────────────────────────────────
XCG_REF  = 1.2994   # burn-out CG aft of vehicle nose - m
MPROP    = 1        # 0:off  1:on  2:2nd-pulse  3:input-thrust
AEXIT    = 0.0125   # nozzle exit area - m²
MASS     = 91.95    # initial vehicle mass - kg
XCG      = 1.536    # initial CG aft of nose - m
AI11     = 0.308    # roll moment of inertia - kg·m²
AI33     = 59.80    # pitch/yaw moment of inertia - kg·m²


# ── Actuator ──────────────────────────────────────────────────────────────────
MACT    = 2        # 0:no dynamics  2:second-order
DLIMX   = 28.0     # fin-angle limit - deg
DDLIMX  = 600.0    # fin-rate limit - deg/s
WNACT   = 251.0    # actuator natural frequency - rad/s
ZETACT  = 0.7      # actuator damping - ND


# ── Autopilot selection ───────────────────────────────────────────────────────
#   2: rate controller
#   3: accel controller (pole-placement)  ← classical CADAC
#   5: NDI  (Nonlinear Dynamic Inversion)
#   6: INDI (Incremental NDI)
MAUT      = 6

# Common autopilot limits
ALIMIT    = 50.0    # total structural acceleration limit - g's
DQLIMX    = 28.0    # pitch fin command limit - deg
DRLIMX    = 28.0    # yaw fin command limit - deg
DPLIMX    = 28.0    # roll command limit - deg
WBLIMX    = 30.0    # body-rate limit (q, r cmd) - deg/s


# ── Roll controller ───────────────────────────────────────────────────────────
ZRCL      = 0.9     # roll closed-loop pole damping - ND
FACT_WRCL = 0.0     # roll-bandwidth factor - ND


# ── Rate controller (maut=2) ──────────────────────────────────────────────────
ZETLAGR   = 0.6     # closed rate-loop damping - ND


# ── Accel controller (maut=3, pole-placement) ────────────────────────────────
TWCL      = 0.4     # wacl smoother time constant - s
WACL_BIAS = 3.0     # wacl bias - rad/s
FACT_WACL = -0.45   # pitch/yaw closed-loop freq factor
PACL      = 14.0    # closed-loop real pole - ND
ZACL      = 0.7     # accel closed-loop complex pole damping - ND


# ── NDI / INDI (maut=5 / 6) ───────────────────────────────────────────────────
WN_NDI_Q      = 50.0   # inner moment-loop bandwidth (pitch) - rad/s
WN_NDI_R      = 50.0   # inner moment-loop bandwidth (yaw)   - rad/s
K_ALPHA       = 10.0   # middle alpha-loop gain - 1/s
K_BETA        = 10.0   # middle beta-loop gain  - 1/s
WN_INDI_FILT    = 50.0   # INDI matched-LPF bandwidth - rad/s
INDI_TWO_STAGE  = False  # True: 2-stage LPF on fins (matched group delay), False: 1-stage (better empirical performance)


# ── Monte Carlo ────────────────────────────────────────────────────────────────
MC_N_RUNS     = 200      # runs per variant per scenario
MC_AERO_SIGMA = 0.15     # aero derivative model uncertainty — std dev (15%)
MC_AERO_PARAMS = [       # which dimensional derivatives to perturb
    'dna', 'dma', 'dmq', 'dmd',
    'dyb', 'dlnb', 'dlnr', 'dlnd',
]
MC_INS_NOISE  = True     # inject INS sensor errors each run
MC_BASE_SEED  = 0        # RNG seed for reproducibility (None → random each run)
MC_SAVE_CSV   = True     # save per-run CSV + stats summary CSV
MC_VARIANTS   = {        # autopilot variants compared in each MC sweep
    'Accel': {'maut': 3},
    'NDI':   {'maut': 5},
    'INDI':  {'maut': 6},
}


# ── Guidance ──────────────────────────────────────────────────────────────────
# mguid = |mid|term|  two-digit flag
#   mid  = 0:off  3:pronav  2:line-A2G  5:line-A2A
#   term = 0:off  6:compensated-pronav (requires seeker lock)
GNAV      = 4.0    # proportional-navigation gain - ND
MNAV      = 0      # 0:direct target data  3:extrapolate datalink

# Line-guidance parameters
LINE_GAIN     = 0.0
NL_GAIN_FACT  = 0.0
DECREMENT     = 1.0
THTFLX        = 0.0


# ── INS ───────────────────────────────────────────────────────────────────────
MINS       = 0       # 0:ideal passthrough  1:real IMU errors
INS_SEED   = None    # int for deterministic bias realizations (None → random)


# ── Seeker (kinematic IR) ─────────────────────────────────────────────────────
#   mseek = 0: off     → seeker disabled, midcourse guidance only
#           2: enabled → waiting for acquisition
#           (3:acq  4:lock  5:blind — set internally at runtime)
MSEEK     = 0
RACQ      = 5000.0   # acquisition range - m
DTIMAC    = 0.2      # acquisition dwell time - s
DBLIND    = 100.0    # blind range - m


# ── TVC (off by default) ──────────────────────────────────────────────────────
MTVC   = 0           # 0:off  1:on
PARM   = 0.0         # TVC nozzle arm length - m
GTVC   = 0.0         # TVC gain
