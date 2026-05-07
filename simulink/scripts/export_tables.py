"""
Export SRAAM6 lookup tables and config parameters to MATLAB .mat format.

Run from project root:
    python3 simulink/scripts/export_tables.py

Outputs:
    simulink/data/sraam6_tables.mat   — all aero/prop lookup tables
    simulink/data/sraam6_config.mat   — simulation parameters
"""

import os
import sys
import numpy as np
import scipy.io

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))))

import config
from missile.database import SRAAM6_Database

OUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'data')
os.makedirs(OUT_DIR, exist_ok=True)

# ── Load database ──────────────────────────────────────────────────────────────
print("Loading SRAAM6 database...")
db = SRAAM6_Database()
db.load_aero_deck(config.AERO_DECK)
db.load_propulsion_deck(config.PROP_DECK)

# ── Aero tables ────────────────────────────────────────────────────────────────
# Grid axes (shared by all 3-D tables)
mach_grid  = db.mach_grid    # shape (10,)
alpha_grid = db.alpha_grid   # shape (25,)
beta_grid  = db.beta_grid    # shape (25,)

tables = {
    'mach_grid' : mach_grid,
    'alpha_grid': alpha_grid,
    'beta_grid' : beta_grid,
}

# 3-D tables  (Mach × Alpha × Beta)  — .values is already a numpy array
for name, interp in db.aero_db.items():
    key = name.replace('vs_', '').replace('_', '__')
    if hasattr(interp, 'values'):          # RegularGridInterpolator
        tables[f'aero3d__{key}'] = interp.values
    else:                                  # interp1d — 1-D Mach table
        tables[f'aero1d__{key}'] = interp.y

path_aero = os.path.join(OUT_DIR, 'sraam6_tables.mat')
scipy.io.savemat(path_aero, tables)

# ── Propulsion tables ──────────────────────────────────────────────────────────
# Saved as separate struct-like fields with clean MATLAB names:
#   prop_thrust_t / prop_thrust_v   — time [s] / thrust [N]
#   prop_mass_t   / prop_mass_v     — time [s] / mass [kg]
#   prop_cg_t     / prop_cg_v       — time [s] / cg from nose [m]
#   prop_moipitch_t / prop_moipitch_v — time [s] / Iyy [kg*m^2]
prop = {}
for name, interp in db.prop_db.items():
    # 'thrust_vs_time' → 'thrust', 'mass_vs_time' → 'mass', etc.
    key = name.replace('_vs_time', '')
    prop[f'prop_{key}_t'] = interp.x   # time breakpoints
    prop[f'prop_{key}_v'] = interp.y   # values

# Write everything into one .mat file
path_out = os.path.join(OUT_DIR, 'sraam6_tables.mat')
scipy.io.savemat(path_out, {**tables, **prop})

print(f"  Aero tables : {len(db.aero_db)}")
print(f"  Prop tables : {len(db.prop_db)}  "
      f"({', '.join(n.replace('_vs_time','') for n in db.prop_db)})")
print(f"  → {path_out}")

# ── Config parameters ──────────────────────────────────────────────────────────
print("\nExporting config...")
cfg = {
    # geometry
    'refl'        : config.REFL,
    'refa'        : config.REFA,
    'xcg_ref'     : config.XCG_REF,
    # mass / inertia
    'mass'        : config.MASS,
    'xcg'         : config.XCG,
    'ai11'        : config.AI11,
    'ai33'        : config.AI33,
    'aexit'       : config.AEXIT,
    # actuator
    'dlimx'       : config.DLIMX,
    'ddlimx'      : config.DDLIMX,
    'wnact'       : config.WNACT,
    'zetact'      : config.ZETACT,
    # autopilot limits
    'alimit'      : config.ALIMIT,
    'dqlimx'      : config.DQLIMX,
    'drlimx'      : config.DRLIMX,
    'dplimx'      : config.DPLIMX,
    'wblimx'      : config.WBLIMX,
    # NDI / INDI bandwidths
    'wn_ndi_q'    : config.WN_NDI_Q,
    'wn_ndi_r'    : config.WN_NDI_R,
    'k_alpha'     : config.K_ALPHA,
    'k_beta'      : config.K_BETA,
    'wn_indi_filt': config.WN_INDI_FILT,
    # roll controller
    'zrcl'        : config.ZRCL,
    'fact_wrcl'   : config.FACT_WRCL,
    # accel controller
    'twcl'        : config.TWCL,
    'wacl_bias'   : config.WACL_BIAS,
    'fact_wacl'   : config.FACT_WACL,
    'pacl'        : config.PACL,
    'zacl'        : config.ZACL,
    # guidance
    'gnav'        : config.GNAV,
    # initial state
    'sbel_init'   : np.array(config.SBEL_INIT),
    'speed_init'  : config.SPEED_INIT,
    # simulation
    'dt'          : config.DT,
    't_end'       : config.T_END,
    'agrav'       : config.AGRAV,
    'hit_r'       : config.HIT_R,
}

path_cfg = os.path.join(OUT_DIR, 'sraam6_config.mat')
scipy.io.savemat(path_cfg, cfg)
print(f"  {len(cfg)} parameters → {path_cfg}")

print("\n✓ Done.")
print("  Next step: open MATLAB and run simulink/scripts/init_workspace.m")
