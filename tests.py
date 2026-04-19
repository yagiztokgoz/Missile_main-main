"""
================================================================================
 SRAAM6  —  Registered Test Registry
================================================================================

OVERVIEW
    All named comparison tests are defined here as entries in the TESTS dict.
    Each entry specifies a human-readable title and an ordered dict of variants,
    where every variant is a cfg dict understood by simulate.run().

    simulate.py imports TESTS from this module so the CLI and programmatic
    helpers (simulate.test, simulate.test_all_scenarios) stay in sync with
    whatever is defined below.

────────────────────────────────────────────────────────────────────────────────
 HOW TO ADD A NEW TEST
────────────────────────────────────────────────────────────────────────────────

    Add an entry to the TESTS dict following the pattern:

        'my_test_name': {
            'title': 'Human-readable title for plot headings',
            'variants': {
                'Label A': {'maut': 5, 'mins': 0, 'mseek': 0},
                'Label B': {'maut': 6, 'mins': 1, 'ins_seed': 7},
            },
        },

    Rules:
      • Key     — short snake_case identifier used on the CLI
      • title   — shown in plot headings and summary tables
      • variants — ordered dict; up to ~4 variants render cleanly on one figure
      • Each variant cfg is merged with defaults inside simulate.run(); only
        keys that differ from the global config need to be listed.
      • hit_r is forced to 0.0 (pure CPA mode) by compare() and
        test_all_scenarios() so miss distances are always comparable.

────────────────────────────────────────────────────────────────────────────────
 AVAILABLE CFG KEYS  (passed to simulate.run via each variant dict)
────────────────────────────────────────────────────────────────────────────────

    maut      — autopilot mode (2 rate / 3 accel / 5 NDI / 6 INDI)
    mins      — INS mode (0 ideal / 1 real IMU errors)
    ins_seed  — int seed for deterministic INS bias realisation (or None)
    mseek     — seeker mode (0 off / 2 kinematic)
    racq      — seeker acquisition range [m]
    dtimac    — seeker dwell-to-lock time [s]
    dblind    — seeker blind range [m]
    mguid     — guidance flag (auto-set from mseek if omitted)
    gnav      — proportional-navigation gain
    hit_r     — intercept radius [m]; 0 → pure CPA mode

================================================================================
"""


TESTS = {

    # ── autopilot comparisons ──────────────────────────────────────────────────

    'accel_vs_ndi': {
        'title': 'Accel Controller vs NDI',
        'variants': {
            'Accel': {'maut': 3, 'mseek': 0, 'mins': 0},
            'NDI':   {'maut': 5, 'mseek': 0, 'mins': 0},
        },
    },

    'ndi_vs_indi': {
        'title': 'NDI vs INDI',
        'variants': {
            'NDI':  {'maut': 5, 'mseek': 0, 'mins': 0},
            'INDI': {'maut': 6, 'mseek': 0, 'mins': 0},
        },
    },

    'ndi_vs_ndi_cop': {
        'title': 'NDI vs NDI-CoP (Center of Percussion)',
        'variants': {
            'NDI':     {'maut': 5, 'mseek': 0, 'mins': 0},
            'NDI-CoP': {'maut': 7, 'mseek': 0, 'mins': 0},
        },
    },

    'accel_vs_indi': {
        'title': 'Accel Controller vs INDI',
        'variants': {
            'Accel': {'maut': 3, 'mseek': 0, 'mins': 0},
            'INDI':  {'maut': 6, 'mseek': 0, 'mins': 0},
        },
    },

    'all_autopilots': {
        'title': 'Rate vs Accel vs NDI vs INDI',
        'variants': {
            'Rate':  {'maut': 2, 'mseek': 0, 'mins': 0},
            'Accel': {'maut': 3, 'mseek': 0, 'mins': 0},
            'NDI':   {'maut': 5, 'mseek': 0, 'mins': 0},
            'INDI':  {'maut': 6, 'mseek': 0, 'mins': 0},
        },
    },

    # ── INS sensitivity ────────────────────────────────────────────────────────

    'ins_on_off_ndi': {
        'title': 'INS On vs Off (NDI autopilot)',
        'variants': {
            'INS off': {'maut': 5, 'mins': 0,              'mseek': 0},
            'INS on':  {'maut': 5, 'mins': 1, 'ins_seed': 42, 'mseek': 0},
        },
    },

    'ins_on_off_indi': {
        'title': 'INS On vs Off (INDI autopilot)',
        'variants': {
            'INS off': {'maut': 6, 'mins': 0,              'mseek': 0},
            'INS on':  {'maut': 6, 'mins': 1, 'ins_seed': 42, 'mseek': 0},
        },
    },

    # ── seeker effect ──────────────────────────────────────────────────────────

    'seeker_on_off': {
        'title': 'Seeker On vs Off (INDI + ideal INS)',
        'variants': {
            'Seeker off': {'maut': 6, 'mseek': 0, 'mins': 0},
            'Seeker on':  {'maut': 6, 'mseek': 2, 'mins': 0},
        },
    },

}
