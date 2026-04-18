"""
================================================================================
 SRAAM6  —  6-DoF Missile Simulation  ·  Usage Guide
================================================================================

OVERVIEW
    Pro-Nav based missile-intercept simulation. Supports the classical
    accel-autopilot, NDI, and INDI inner loops, with an optional kinematic IR
    seeker and INS error model. Runs in either single-shot or
    variant-comparison (test-runner) mode.

    All tunable constants live in `config.py`. A subset can be overridden
    per-variant at runtime via a cfg dict — see the CFG OVERRIDE section.

────────────────────────────────────────────────────────────────────────────────
 COMMAND-LINE USAGE
────────────────────────────────────────────────────────────────────────────────

 1) SINGLE-SCENARIO RUN (defaults from config.py)
    $ python3 simulate.py                    # scenario 1
    $ python3 simulate.py 3                  # scenario 3
    → saves CSV + 2 PNGs (overview & control), opens the plot window

 2) RUN ALL SCENARIOS (summary table)
    $ python3 simulate.py all

 3) COMPARISON TEST — one scenario, two/three variants
    $ python3 simulate.py test ndi_vs_indi          # default scenario = 1
    $ python3 simulate.py test ndi_vs_indi 3        # scenario 3
    → 3 PNGs: overview (geometry + response), control (rates/Eulers/fins),
      ins (INS-error magnitudes; rendered only if some variant has INS on)

 4) COMPARISON TEST — ACROSS all scenarios (table only)
    $ python3 simulate.py test accel_vs_ndi all
    → prints a scenario × variant miss-distance table, no per-scenario PNG

 5) MONTE CARLO — one scenario
    Tokens after 'mc' can be given in any order:
      <test_name>   name from TESTS registry  (default: MC_VARIANTS in config.py)
      <scenario>    integer 1–10              (default: 1)
      <n_runs>      integer run count         (default: MC_N_RUNS in config.py)

    $ python3 simulate.py mc                        # S1, MC_VARIANTS, MC_N_RUNS
    $ python3 simulate.py mc 3                      # S3, MC_VARIANTS, MC_N_RUNS
    $ python3 simulate.py mc ndi_vs_indi            # S1, NDI vs INDI variants
    $ python3 simulate.py mc ndi_vs_indi 3          # S3, NDI vs INDI variants
    $ python3 simulate.py mc ndi_vs_indi 3 50       # S3, NDI vs INDI, 50 runs
    $ python3 simulate.py mc 4 50                   # S4, MC_VARIANTS, 50 runs

    → saves: logs/mc_runs_s<N>.csv   — per-run: variant, miss_dist, ins_seed
             logs/mc_stats_s<N>.csv  — aggregate: mean/std/CEP/P90/P95/Pk
             plots/s<N>_mc_dist.png  — histogram+KDE, empirical CDF, violin
             plots/s<N>_mc_stats.png — formatted statistics table figure
    Note: runtime ≈ n_runs × n_variants × 9 s  (200 × 3 ≈ 90 min)
          Use 50 runs for a quick sanity check before the full sweep.

 6) MONTE CARLO — across ALL scenarios (CEP table only, no plot windows)
    $ python3 simulate.py mc all                    # MC_VARIANTS, MC_N_RUNS
    $ python3 simulate.py mc all ndi_vs_indi        # NDI vs INDI variants
    $ python3 simulate.py mc all ndi_vs_indi 50     # NDI vs INDI, 50 runs
    → prints scenario × variant CEP table to stdout after all scenarios finish

────────────────────────────────────────────────────────────────────────────────
 REGISTERED TESTS  (defined in tests.py, imported as TESTS)
────────────────────────────────────────────────────────────────────────────────

   Name                  | Variants                    | Purpose
   ──────────────────────┼─────────────────────────────┼──────────────────────
   accel_vs_ndi          | Accel vs NDI                | Classical vs nonlinear
   ndi_vs_indi           | NDI vs INDI                 | Full vs incremental
   accel_vs_indi         | Accel vs INDI               | Classical vs incremental
   all_autopilots        | Accel + NDI + INDI          | 3-way sweep
   ins_on_off_ndi        | NDI INS-off vs NDI INS-on   | INS-error sensitivity
   ins_on_off_indi       | INDI INS-off vs INDI INS-on | INDI's INS robustness
   seeker_on_off         | INDI seeker-off vs on       | Terminal pronav effect

   To add a new test, edit tests.py — no changes to simulate.py required.

────────────────────────────────────────────────────────────────────────────────
 PROGRAMMATIC USAGE  (REPL / notebook)
────────────────────────────────────────────────────────────────────────────────

    import simulate

    # A) Single run with default config
    log, result, cpa = simulate.run(scenario=3)
    # cpa = {'min_dist': 0.19, 't_cpa': 5.236, 'pos': (x, y, h)}

    # B) Single run with cfg override
    log, result, cpa = simulate.run(
        scenario=3,
        cfg={'maut': 6, 'mseek': 2, 'mins': 1, 'ins_seed': 42},
    )

    # C) Run a registered test
    simulate.test('ndi_vs_indi', scenario=3)

    # D) Run a test across every scenario (table + per-scenario miss)
    simulate.test_all_scenarios('all_autopilots')

    # E) Ad-hoc comparison (not in the registry)
    simulate.compare(
        scenario=5,
        test_name='my_sweep',
        variants={
            'low-BW':  {'maut': 6},
            'high-BW': {'maut': 6},
        },
    )

    # F) Batch sweep across scenarios
    simulate.run_all(scenarios=[1, 3, 5], save_csvs=True, show_plots=False)

    # G) Monte Carlo — one scenario, all MC_VARIANTS from config.py
    results = simulate.monte_carlo(scenario=1)
    # results = {
    #   'Accel': {'miss_dists': [...], 'mean': 1.4, 'cep': 0.9, 'p90': 3.1, ...},
    #   'NDI':   {...},
    #   'INDI':  {...},
    # }

    # H) Monte Carlo — ad-hoc variants (override MC_VARIANTS for this call only)
    results = simulate.monte_carlo(
        scenario=4,
        variants={'NDI': {'maut': 5}, 'INDI': {'maut': 6}},
        n_runs=100,            # override MC_N_RUNS
        aero_sigma=0.20,       # override MC_AERO_SIGMA  (20% uncertainty)
        ins_noise=True,        # override MC_INS_NOISE
        base_seed=42,          # override MC_BASE_SEED   (reproducible)
        save=True,             # write CSV + PNG
        show=False,            # suppress plot window
    )

    # I) Monte Carlo — disable aero perturbation (INS noise only)
    results = simulate.monte_carlo(
        scenario=2,
        aero_sigma=0.0,        # no aero perturbation
        ins_noise=True,
    )

    # J) Monte Carlo — sweep all scenarios, print CEP table (no plots)
    simulate.monte_carlo_all_scenarios(n_runs=50, show=False)

────────────────────────────────────────────────────────────────────────────────
 CFG-DICT OVERRIDE
────────────────────────────────────────────────────────────────────────────────

    Keys supported by run() / compare() (parsed inside run()):
      maut      — autopilot (3 accel / 5 NDI / 6 INDI)
      mins      — INS mode (0 ideal / 1 IMU errors)
      ins_seed  — deterministic seed for INS biases (int)
      mseek     — seeker mode (0 off / 2 kinematic on)
      racq, dtimac, dblind — seeker range/timing parameters
      mguid     — guidance flag; if omitted, auto-selected from mseek (30 ↔ 36)
      gnav      — pronav gain
      hit_r     — intercept radius; 0 → pure CPA mode
      aero_pert — {param: factor} multiplicative perturbation on aero derivatives
                  e.g. {'dna': 1.12, 'dma': 0.93}  (set internally by MC runner)

    Parameters NOT in this list (MASS, AI33, WN_NDI_Q, K_ALPHA, …) must be
    edited in config.py BEFORE the Missile is constructed — they are read in
    Missile.__init__() and not refreshed at runtime.

────────────────────────────────────────────────────────────────────────────────
 FUNCTION REFERENCE
────────────────────────────────────────────────────────────────────────────────

    run(scenario, cfg=None, verbose=True)
        Integrate one scenario. Returns (log, result_str, cpa_dict).
        cpa = {'min_dist': float, 't_cpa': float, 'pos': (x, y, h)}

    run_all(scenarios=None, hit_radius=0.0, save_csvs=False, show_plots=False)
        Run the given scenario list (default: all), print summary table.

    compare(scenario, test_name, variants, save=True, show=True)
        Run one scenario with N variants and render an overlay comparison.
        variants = {'Label1': cfg1, 'Label2': cfg2, ...}
        Forces hit_r=0 by default for a fair CPA comparison.

    test(test_name, scenario=1, save=True, show=True)
        Shortcut for running a registered test from TESTS (defined in tests.py).

    test_all_scenarios(test_name, scenarios=None, save=True, show=False)
        Run a registered test across multiple scenarios; prints a
        scenario × variant miss-distance table.

    save_csv(log, tag)
        Write the log to config.OUTDIR/sim_log_<tag>.csv.

    monte_carlo(scenario, variants, n_runs, aero_sigma, aero_params,
                ins_noise, base_seed, save, show)
        Monte Carlo miss-distance analysis. Independently samples INS biases
        and multiplicative aero derivative perturbations each run. Physical
        aerodynamic forces (ca, cn, cy) are unperturbed — only the controller's
        dimensional derivative model (dna, dma, dmq, …) receives the noise,
        so NDI (which inverts the full model) is stressed while INDI
        (incremental, model-independent) is inherently more robust.
        All parameters default to their config.MC_* counterparts.
        Returns {label: {'miss_dists', 'mean', 'std', 'cep', 'p90', 'p95',
                          'pk5', 'pk10', 'pk20', 'results', …}}.

    monte_carlo_all_scenarios(scenarios, variants, n_runs, aero_sigma,
                              aero_params, ins_noise, base_seed, save)
        Run monte_carlo() for every scenario and print a CEP summary table.
        Plots are suppressed (show=False) to avoid N blocking windows.

────────────────────────────────────────────────────────────────────────────────
 MONTE CARLO CONFIG  (config.py  ── MC_* keys)
────────────────────────────────────────────────────────────────────────────────

    MC_N_RUNS      int     200     Runs per variant per scenario
    MC_AERO_SIGMA  float   0.15    Std-dev of each derivative factor N(1,σ²)
    MC_AERO_PARAMS list    8 deriv Derivatives to perturb: dna dma dmq dmd
                                   dyb dlnb dlnr dlnd
    MC_INS_NOISE   bool    True    Inject IMU sensor errors each run
    MC_BASE_SEED   int|None  0     RNG seed (None → different every run)
    MC_SAVE_CSV    bool    True    Write mc_runs_s<N>.csv + mc_stats_s<N>.csv
    MC_VARIANTS    dict    3 auto  Default autopilot variants for the MC sweep
                                   {'Accel': {maut:3}, 'NDI': {maut:5},
                                    'INDI':  {maut:6}}

    Runtime note: each run takes ~9 s (Python 1 kHz integrator).
    200 runs × 3 variants ≈ 90 min.  Use n_runs=50 for quick checks.

────────────────────────────────────────────────────────────────────────────────
 OUTPUTS
────────────────────────────────────────────────────────────────────────────────

    Single-run / comparison
      CSV  → logs/sim_log_<tag>.csv
      PNG  → plots/s<N>_<test>_<fig>.png   (fig ∈ overview | control | ins)

    Monte Carlo
      CSV  → logs/mc_runs_s<N>.csv         per-run: variant, run, miss_dist,
                                            result, ins_seed
      CSV  → logs/mc_stats_s<N>.csv        aggregate: mean, std, cep, p90,
                                            p95, pk5, pk10, pk20 per variant
      PNG  → plots/s<N>_mc_dist.png        histogram + KDE, CDF, violin plot
      PNG  → plots/s<N>_mc_stats.png       formatted statistics table

    Log columns per sample (single-run CSV): t, maut, mins, mseek, mguid,
    position (truth and INS), velocity/Mach/q, attitude (truth and INS),
    body rates (truth and INS), specific force (truth and INS), fin cmd/actual
    (per-fin delcx/delx included), thrust/mass/xcg, accel cmd/achieved plus
    tracking error, range/tgo, INS errors (ESTTC/EVBE/RECE), seeker angles
    and LOS rates.

================================================================================
"""

import sys
import csv
import math
import os
import numpy as np

import config
from missile   import Missile
from target    import Target
from plotting  import SimPlotter, ComparisonPlotter

# ── shorthand handles from config (so tests can import from simulate too) ──────
OUTDIR = config.OUTDIR
G      = config.AGRAV

DT     = config.DT
T_END  = config.T_END
LOG_DT = config.LOG_DT
HIT_R  = config.HIT_R

MINS     = config.MINS
INS_SEED = config.INS_SEED

MSEEK    = config.MSEEK
RACQ     = config.RACQ
DTIMAC   = config.DTIMAC
DBLIND   = config.DBLIND

MAUT     = config.MAUT


def _parse_scenario():
    if len(sys.argv) <= 1:
        return 1
    arg = sys.argv[1]
    if arg in ('all', 'test', 'mc'):
        return None
    return int(arg)


SCENARIO = _parse_scenario()


# ── simulation loop ────────────────────────────────────────────────────────────

def run(scenario=1, cfg=None, verbose=True):
    """Run one scenario. `cfg` optionally overrides any global knob:
        mins, ins_seed, mseek, racq, dtimac, dblind, maut, mguid, gnav, hit_r.
    Returns (log, result).
    """
    cfg = cfg or {}
    name = Target.NAMES[scenario]

    # resolve config (override > module globals)
    mins_      = cfg.get('mins',      MINS)
    ins_seed_  = cfg.get('ins_seed',  INS_SEED)
    mseek_     = cfg.get('mseek',     MSEEK)
    racq_      = cfg.get('racq',      RACQ)
    dtimac_    = cfg.get('dtimac',    DTIMAC)
    dblind_    = cfg.get('dblind',    DBLIND)
    maut_      = cfg.get('maut',      MAUT)
    mguid_     = cfg.get('mguid',     36 if mseek_ != 0 else 30)
    gnav_      = cfg.get('gnav',      4.0)
    hit_r_     = cfg.get('hit_r',     HIT_R)
    aero_pert_ = cfg.get('aero_pert', {})

    if verbose:
        print(f"\n{'='*70}")
        print(f"  Scenario {scenario}: {name}")
        print(f"  cfg: maut={maut_}  mins={mins_}  mseek={mseek_}  mguid={mguid_}")
        print(f"{'='*70}")

    tgt = Target(scenario)
    m   = Missile()

    m.ins_seed  = ins_seed_
    m.mins      = mins_
    m.mguid     = mguid_
    m.gnav      = gnav_
    m.maut      = maut_
    m.mnav      = 0
    m.mseek     = mseek_
    m.racq      = racq_
    m.dtimac    = dtimac_
    m.dblind    = dblind_
    m.aero_pert = aero_pert_

    log_every = max(1, round(LOG_DT / DT))
    log      = []
    step     = 0
    result   = "TIMEOUT"
    min_dtbc = float('inf')
    min_t    = 0.0
    min_pos  = (0.0, 0.0, 0.0)

    if verbose:
        print(f"\n{'t':>7}  {'hbe':>8}  {'dvbe':>8}  {'mach':>6}  "
              f"{'alpha':>7}  {'ancomx':>8}  {'dtbc':>8}  {'tgoc':>6}")
        print("-" * 70)

    while m.time < T_END:

        tpos, tvel = tgt.state(m.time)
        m.STEL = tpos
        m.VTEL = tvel
        m.launch_time = m.time

        m.environment.environment()
        m.kinematics.kinematics(DT)
        m.aerodynamics.aerodynamics()
        m.propulsion.propulsion()
        m.forces.forces()
        m.ins.ins(DT)
        m.sensor.sensor(DT)
        m.guidance.guidance()
        m.control.control(DT)
        m.actuator.actuator(DT)
        m.euler.euler(DT)
        m.newton.newton(DT)

        if step % log_every == 0:
            dtbc = float(np.linalg.norm(m.STEL - m.SBEL))

            # derived metrics
            an_cmd_mag  = math.sqrt(m.ancomx**2 + m.alcomx**2)
            an_ach_mag  = math.sqrt(m.anx**2 + m.ayx**2)
            tr_err_n    = m.ancomx - m.anx      # normal tracking error - g
            tr_err_l    = m.alcomx - m.ayx      # lateral tracking error - g
            tr_err_mag  = math.sqrt(tr_err_n**2 + tr_err_l**2)

            # truth body rates (will equal estimate when mins=0)
            wbeb_pdeg = m.WBEB[0] * 57.2957795
            wbeb_qdeg = m.WBEB[1] * 57.2957795
            wbeb_rdeg = m.WBEB[2] * 57.2957795

            log.append({
                # time + config snapshot (self-describing logs)
                't':       m.time,
                'maut':    maut_,     'mins':    mins_,
                'mseek':   m.mseek,   'mguid':   mguid_,
                # position — truth & INS
                'sbel1':   m.sbel1,    'sbel2':   m.sbel2,    'hbe':      m.hbe,
                'stel1':   m.STEL[0],  'stel2':   m.STEL[1],  'stel_hbe': -m.STEL[2],
                'sbelc1':  m.SBELC[0], 'sbelc2':  m.SBELC[1], 'hbem':     m.hbem,
                # velocity / speed
                'dvbe':    m.dvbe,     'dvbec':   m.dvbec,
                'mach':    m.mach,     'pdynmc':  m.pdynmc / 1e3,
                # flight path / attitude
                'psivlx':  m.psivlx,   'thtvlx':  m.thtvlx,
                'alphax':  m.alphax,   'betax':   m.betax,    'alppx':    m.alppx,
                'alphaxc': getattr(m, 'alphaxc', m.alphax),
                'betaxc':  getattr(m, 'betaxc',  m.betax),
                'psiblx':  m.psiblx,   'thtblx':  m.thtblx,   'phiblx':   m.phiblx,
                'psivlcx': m.psivlcx,  'thtvlcx': m.thtvlcx,  'phiblcx':  m.phiblcx,
                # rates — truth & INS estimate (both in deg/s)
                'ppx':     m.ppx,      'qqx':     m.qqx,      'rrx':      m.rrx,
                'wbeb_p':  wbeb_pdeg,  'wbeb_q':  wbeb_qdeg,  'wbeb_r':   wbeb_rdeg,
                # specific force — truth & INS estimate (m/s²)
                'fspb1':   m.FSPB[0],  'fspb2':   m.FSPB[1],  'fspb3':    m.FSPB[2],
                'fspcb1':  m.FSPCB[0], 'fspcb2':  m.FSPCB[1], 'fspcb3':   m.FSPCB[2],
                # fins — commands and actuals
                'dpcx':    m.dpcx,     'dqcx':    m.dqcx,     'drcx':     m.drcx,
                'dpx':     m.dpx,      'dqx':     m.dqx,      'drx':      m.drx,
                'delcx1':  m.actuator.delcx1, 'delcx2': m.actuator.delcx2,
                'delcx3':  m.actuator.delcx3, 'delcx4': m.actuator.delcx4,
                'delx1':   m.actuator.delx1,  'delx2':  m.actuator.delx2,
                'delx3':   m.actuator.delx3,  'delx4':  m.actuator.delx4,
                # propulsion / mass
                'thrust':  m.thrust,   'mass':    m.mass,     'xcg':      m.xcg,
                # accels — achieved & commanded (g's) + tracking error
                'anx':     m.anx,      'ayx':     m.ayx,
                'ancomx':  m.ancomx,   'alcomx':  m.alcomx,
                'an_cmd_mag': an_cmd_mag, 'an_ach_mag': an_ach_mag,
                'tr_err_n':   tr_err_n,   'tr_err_l':  tr_err_l,  'tr_err_mag': tr_err_mag,
                # range / tgo
                'dtbc':    dtbc,       'tgoc':    m.tgoc,
                # INS error states
                'esttc1':  m.ins.ESTTC[0], 'esttc2': m.ins.ESTTC[1], 'esttc3': m.ins.ESTTC[2],
                'evbe1':   m.ins.EVBE[0],  'evbe2':  m.ins.EVBE[1],  'evbe3':  m.ins.EVBE[2],
                'rece1x':  m.ins.RECE[0]*57.2957795, 'rece2x': m.ins.RECE[1]*57.2957795,
                'rece3x':  m.ins.RECE[2]*57.2957795,
                # seeker
                'psipbx':  m.psipbx,   'thtpbx':  m.thtpbx,
                'sigdpy':  m.sigdpy,   'sigdpz':  m.sigdpz,
            })

            if verbose and step % (log_every * 20) == 0:
                print(f"{m.time:7.3f}  {m.hbe:8.1f}  {m.dvbe:8.2f}  {m.mach:6.3f}  "
                      f"{m.alphax:7.2f}  {m.ancomx:8.3f}  {dtbc:8.1f}  {m.tgoc:6.2f}")

        dtbc = float(np.linalg.norm(m.STEL - m.SBEL))

        if dtbc < min_dtbc:
            min_dtbc = dtbc
            min_t    = m.time
            min_pos  = (m.sbel1, m.sbel2, m.hbe)

        if hit_r_ > 0 and dtbc < hit_r_:
            result = f"INTERCEPT  t={m.time:.3f}s  miss={dtbc:.2f}m"
            if verbose:
                print(f"\n{result}  pos=({m.sbel1:.0f}, {m.sbel2:.0f}, {m.hbe:.0f}m)")
            break

        if m.time > config.CPA_MIN_TIME and dtbc > min_dtbc + config.CPA_GROW_M:
            result = (f"CPA PASSED  t={min_t:.3f}s  min_dist={min_dtbc:.2f}m  "
                      f"pos=({min_pos[0]:.0f}, {min_pos[1]:.0f}, {min_pos[2]:.0f}m)")
            if verbose:
                print(f"\n{result}")
            break

        if m.hbe <= 0:
            result = f"GROUND IMPACT  t={m.time:.3f}s  min_dist={min_dtbc:.2f}m"
            if verbose:
                print(f"\n{result}")
            break
        if m.alppx > m.alplimx:
            result = f"ALPHA LIMIT  t={m.time:.3f}s  alpp={m.alppx:.1f}deg  min_dist={min_dtbc:.2f}m"
            if verbose:
                print(f"\n{result}")
            break

        tgt.step(m.time, DT)

        m.time += DT
        step   += 1

    if result == "TIMEOUT":
        result = (f"TIMEOUT  t={T_END:.1f}s  min_dist={min_dtbc:.2f}m  "
                  f"(@t={min_t:.3f}s)")
        if verbose:
            print(f"\n{result}")

    cpa = {'min_dist': min_dtbc, 't_cpa': min_t, 'pos': min_pos}
    return log, result, cpa


# ── CSV export ─────────────────────────────────────────────────────────────────

def save_csv(log, tag):
    """Save log to logs/sim_log_<tag>.csv."""
    if not log:
        return
    os.makedirs(OUTDIR, exist_ok=True)
    path = f'{OUTDIR}/sim_log_{tag}.csv'
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(log[0].keys()))
        writer.writeheader()
        writer.writerows(log)
    print(f"CSV saved → {path}  ({len(log)} rows)")


# ── batch sweep ────────────────────────────────────────────────────────────────

def run_all(scenarios=None, hit_radius=0.0, save_csvs=False, show_plots=False):
    """Run every scenario and print a miss-distance summary."""
    if scenarios is None:
        scenarios = sorted(Target.NAMES.keys())

    rows = []
    for s in scenarios:
        log, result, cpa = run(s, cfg={'hit_r': hit_radius})
        if not log:
            rows.append((s, float('nan'), float('nan'), 'EMPTY LOG'))
            continue
        rows.append((s, cpa['min_dist'], cpa['t_cpa'], result))

        if save_csvs:
            save_csv(log, f's{s}')
        if show_plots:
            SimPlotter(log, s, Target.NAMES[s], result, hit_radius).render()

    _print_summary_table(rows, hit_radius)
    return rows


def _print_summary_table(rows, hit_radius):
    print("\n" + "=" * 78)
    print(f"  SUMMARY  —  {len(rows)} scenarios  (HIT_R={hit_radius})")
    print("=" * 78)
    print(f"  {'S':>3}  {'min_dist [m]':>12}  {'t_CPA [s]':>10}   result")
    print("  " + "-" * 74)
    for s, d, tc, res in rows:
        name = Target.NAMES[s].split('|')[0].strip()
        print(f"  {s:>3}  {d:>12.2f}  {tc:>10.3f}   {name}")
    print("=" * 78)

    valid = [r for r in rows if not math.isnan(r[1])]
    if valid:
        mean_d = sum(r[1] for r in valid) / len(valid)
        max_d  = max(r[1] for r in valid)
        min_d  = min(r[1] for r in valid)
        print(f"  mean={mean_d:.2f} m   min={min_d:.2f} m   max={max_d:.2f} m")
        print("=" * 78)


# ── variant-comparison testbed ─────────────────────────────────────────────────

def compare(scenario, test_name, variants, save=True, show=True):
    """Run one scenario with multiple config variants and render a side-by-side
    comparison figure.

    `variants` is an ordered dict: {label: cfg, ...}
    """
    print(f"\n{'#'*70}")
    print(f"  TEST: {test_name}   ·   scenario {scenario} ({Target.NAMES[scenario].split('|')[0].strip()})")
    print(f"{'#'*70}")

    logs    = {}
    results = {}
    miss    = {}
    t_cpa   = {}
    for label, cfg in variants.items():
        # force full CPA mode for comparison (no early termination at HIT_R)
        cfg_full = dict(cfg)
        cfg_full.setdefault('hit_r', 0.0)
        print(f"\n--- variant: {label}   cfg={cfg_full}")
        log, result, cpa = run(scenario, cfg=cfg_full, verbose=False)
        logs[label]    = log
        results[label] = result
        miss[label]    = cpa['min_dist'] if log else float('nan')
        t_cpa[label]   = cpa['t_cpa']    if log else float('nan')

    # summary
    print(f"\n{'─'*70}")
    print(f"  {test_name}  summary")
    print(f"{'─'*70}")
    for label in variants:
        print(f"    {label:>14}   miss={miss[label]:7.2f} m   t_cpa={t_cpa[label]:6.3f}s")
    print(f"{'─'*70}\n")

    # save individual CSVs for each variant
    if save:
        for label, log in logs.items():
            safe = label.replace(' ', '_').replace('/', '_')
            save_csv(log, f's{scenario}_{test_name}_{safe}')

    # comparison plot
    plotter = ComparisonPlotter(
        logs, scenario, Target.NAMES[scenario], test_name, results,
    )
    plotter.render(save=save, show=show)

    return logs, results


# ── registered tests (defined in tests.py, imported here) ─────────────────────

from tests import TESTS


def test(test_name, scenario=1, save=True, show=True):
    """Run a registered test."""
    if test_name not in TESTS:
        raise ValueError(f"unknown test: '{test_name}'. available: {list(TESTS)}")
    spec = TESTS[test_name]
    return compare(scenario, spec['title'], spec['variants'], save=save, show=show)


def test_all_scenarios(test_name, scenarios=None, save=True, show=False):
    """Run one test across multiple scenarios. Returns miss-distance table."""
    if scenarios is None:
        scenarios = sorted(Target.NAMES.keys())
    spec = TESTS[test_name]
    variants = spec['variants']

    rows = []
    for s in scenarios:
        print(f"\n────── {test_name} · scenario {s} ──────")
        miss = {}
        for label, cfg in variants.items():
            cfg_full = dict(cfg)
            cfg_full.setdefault('hit_r', 0.0)
            log, _, cpa = run(s, cfg=cfg_full, verbose=False)
            miss[label] = cpa['min_dist'] if log else float('nan')
        rows.append((s, miss))

    # pretty-print compare table
    labels = list(variants)
    print(f"\n{'='*78}")
    print(f"  TEST:  {spec['title']}  —  all scenarios")
    print(f"{'='*78}")
    header = f"  {'S':>3}  " + "  ".join(f"{lbl:>10}" for lbl in labels) + "   best"
    print(header)
    print("  " + "-" * (len(header) - 2))
    for s, miss in rows:
        best = min(miss, key=lambda k: miss[k])
        cells = "  ".join(f"{miss[lbl]:>10.2f}" for lbl in labels)
        print(f"  {s:>3}  {cells}   {best}")
    print("=" * 78)
    for lbl in labels:
        vals = [m[lbl] for _, m in rows if not math.isnan(m[lbl])]
        if vals:
            print(f"  {lbl:>14}:  mean={sum(vals)/len(vals):6.2f} m   "
                  f"min={min(vals):5.2f}   max={max(vals):5.2f}")
    print("=" * 78)
    return rows


# ── Monte Carlo ────────────────────────────────────────────────────────────────

def monte_carlo(
    scenario=1,
    variants=None,
    n_runs=None,
    aero_sigma=None,
    aero_params=None,
    ins_noise=None,
    base_seed=None,
    save=True,
    show=True,
):
    """Monte Carlo miss-distance analysis with aero model uncertainty + INS noise.

    Each run independently samples:
      - INS sensor biases  (if ins_noise=True)
      - Multiplicative perturbation on each aero derivative in aero_params
        factor ~ 1 + aero_sigma * N(0,1)

    The actual aerodynamic forces (ca, cy, cn, …) are truth — only the
    controller's dimensional derivative model is perturbed, so NDI (which
    inverts the full model) is stressed while INDI (incremental) is not.

    Args:
        scenario   : target scenario 1–10
        variants   : {label: cfg} — defaults to config.MC_VARIANTS
        n_runs     : runs per variant — defaults to config.MC_N_RUNS
        aero_sigma : std dev of multiplicative perturbation — config.MC_AERO_SIGMA
        aero_params: list of derivative names to perturb — config.MC_AERO_PARAMS
        ins_noise  : inject INS errors — config.MC_INS_NOISE
        base_seed  : RNG seed for reproducibility — config.MC_BASE_SEED
        save       : write CSVs + PNGs to disk
        show       : open plot window

    Returns:
        dict {label: {'miss_dists', 'results', 'mean', 'std', 'cep', …}}
    """
    from plotting import MCPlotter

    if variants   is None: variants   = config.MC_VARIANTS
    if n_runs     is None: n_runs     = config.MC_N_RUNS
    if aero_sigma is None: aero_sigma = config.MC_AERO_SIGMA
    if aero_params is None: aero_params = config.MC_AERO_PARAMS
    if ins_noise  is None: ins_noise  = config.MC_INS_NOISE
    if base_seed  is None: base_seed  = config.MC_BASE_SEED

    rng  = np.random.default_rng(base_seed)
    name = Target.NAMES[scenario]

    print(f"\n{'#'*70}")
    print(f"  MONTE CARLO  ·  S{scenario}: {name.split('|')[0].strip()}")
    print(f"  n={n_runs}  σ_aero={aero_sigma:.0%}  INS={'on' if ins_noise else 'off'}")
    print(f"  variants: {list(variants)}")
    print(f"{'#'*70}\n")

    # pre-sample seeds and factors — same across variants for fair comparison
    ins_seeds = rng.integers(0, 2**31, size=n_runs).tolist()
    if aero_sigma > 0 and aero_params:
        aero_draws = {p: rng.standard_normal(n_runs) for p in aero_params}
    else:
        aero_draws = {}

    mc_results = {}

    for label, variant_cfg in variants.items():
        print(f"  {label:<10} ", end='', flush=True)
        miss_dists, result_strs = [], []

        for i in range(n_runs):
            run_cfg = dict(variant_cfg)
            run_cfg['hit_r'] = 0.0

            if ins_noise:
                run_cfg['mins']     = 1
                run_cfg['ins_seed'] = ins_seeds[i]
            else:
                run_cfg['mins'] = 0

            if aero_draws:
                run_cfg['aero_pert'] = {
                    p: 1.0 + aero_sigma * aero_draws[p][i]
                    for p in aero_params
                }

            _, res_str, cpa = run(scenario, cfg=run_cfg, verbose=False)
            miss_dists.append(cpa['min_dist'])
            result_strs.append(res_str)

            if (i + 1) % (n_runs // 10 or 1) == 0:
                print('.', end='', flush=True)

        print()

        arr = np.array(miss_dists)
        mc_results[label] = {
            'miss_dists':  miss_dists,
            'results':     result_strs,
            'ins_seeds':   ins_seeds,
            'aero_draws':  aero_draws,
            # statistics
            'mean': float(np.mean(arr)),
            'std':  float(np.std(arr)),
            'min':  float(np.min(arr)),
            'max':  float(np.max(arr)),
            'cep':  float(np.percentile(arr, 50)),
            'p90':  float(np.percentile(arr, 90)),
            'p95':  float(np.percentile(arr, 95)),
            'pk5':  float(np.mean(arr <  5.0) * 100),
            'pk10': float(np.mean(arr < 10.0) * 100),
            'pk20': float(np.mean(arr < 20.0) * 100),
        }

    _print_mc_table(mc_results, scenario, n_runs, aero_sigma)

    if save and config.MC_SAVE_CSV:
        _save_mc_csv(mc_results, scenario, n_runs, aero_sigma)

    mc_cfg = {
        'n_runs': n_runs, 'aero_sigma': aero_sigma,
        'aero_params': aero_params, 'ins_noise': ins_noise,
    }
    MCPlotter(mc_results, scenario, name, mc_cfg).render(save=save, show=show)

    return mc_results


def monte_carlo_all_scenarios(
    scenarios=None,
    variants=None,
    n_runs=None,
    aero_sigma=None,
    aero_params=None,
    ins_noise=None,
    base_seed=None,
    save=True,
):
    """Run monte_carlo() across multiple scenarios and print a CEP summary table.

    Returns list of (scenario, {label: mc_result}) tuples.
    """
    if scenarios is None:
        scenarios = sorted(Target.NAMES.keys())
    if variants  is None:
        variants  = config.MC_VARIANTS

    all_results = []
    for s in scenarios:
        res = monte_carlo(
            scenario=s, variants=variants, n_runs=n_runs,
            aero_sigma=aero_sigma, aero_params=aero_params,
            ins_noise=ins_noise, base_seed=base_seed,
            save=save, show=False,
        )
        all_results.append((s, res))

    # summary CEP table
    labels = list(variants)
    print(f"\n{'='*78}")
    print(f"  MC ALL-SCENARIOS  —  CEP [m]  (n={n_runs or config.MC_N_RUNS}  "
          f"σ={aero_sigma or config.MC_AERO_SIGMA:.0%})")
    print(f"{'='*78}")
    header = f"  {'S':>3}  " + "  ".join(f"{lbl:>10}" for lbl in labels) + "   best"
    print(header)
    print("  " + "-" * (len(header) - 2))
    for s, res in all_results:
        cells = "  ".join(f"{res[lbl]['cep']:>10.2f}" for lbl in labels)
        best  = min(labels, key=lambda l: res[l]['cep'])
        print(f"  {s:>3}  {cells}   {best}")
    print("=" * 78)
    for lbl in labels:
        vals = [res[lbl]['cep'] for _, res in all_results]
        print(f"  {lbl:>10}:  mean CEP={sum(vals)/len(vals):6.2f} m   "
              f"min={min(vals):5.2f}   max={max(vals):5.2f}")
    print("=" * 78)

    return all_results


def _print_mc_table(mc_results, scenario, n_runs, aero_sigma):
    """Print formatted Monte Carlo statistics table to stdout."""
    print(f"\n{'='*82}")
    print(f"  MC RESULTS  S{scenario}  n={n_runs}  σ_aero={aero_sigma:.0%}")
    print(f"{'='*82}")
    print(f"  {'Variant':<10} {'Mean':>7} {'Std':>6} {'Min':>6} {'Max':>7} "
          f"{'CEP':>7} {'P90':>7} {'P95':>7} {'Pk<5m':>7} {'Pk<10m':>8}")
    print("  " + "-" * 78)
    for lbl, d in mc_results.items():
        print(f"  {lbl:<10} {d['mean']:>7.2f} {d['std']:>6.2f} {d['min']:>6.2f} "
              f"{d['max']:>7.2f} {d['cep']:>7.2f} {d['p90']:>7.2f} {d['p95']:>7.2f} "
              f"{d['pk5']:>6.1f}% {d['pk10']:>7.1f}%")
    print(f"{'='*82}")


def _save_mc_csv(mc_results, scenario, n_runs, aero_sigma):
    """Save per-run results and aggregate stats to CSV files."""
    os.makedirs(OUTDIR, exist_ok=True)

    # per-run CSV
    path_runs = f"{OUTDIR}/mc_runs_s{scenario}.csv"
    with open(path_runs, 'w', newline='') as f:
        fields = ['variant', 'run', 'miss_dist', 'result', 'ins_seed']
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for lbl, d in mc_results.items():
            for i, (md, res) in enumerate(zip(d['miss_dists'], d['results'])):
                writer.writerow({
                    'variant': lbl, 'run': i,
                    'miss_dist': f"{md:.4f}", 'result': res,
                    'ins_seed': d['ins_seeds'][i],
                })
    print(f"MC runs  → {path_runs}  ({n_runs} rows/variant)")

    # stats CSV
    path_stats = f"{OUTDIR}/mc_stats_s{scenario}.csv"
    with open(path_stats, 'w', newline='') as f:
        stat_keys = ['mean', 'std', 'min', 'max', 'cep', 'p90', 'p95', 'pk5', 'pk10', 'pk20']
        fields    = ['variant', 'n_runs', 'aero_sigma'] + stat_keys
        writer    = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for lbl, d in mc_results.items():
            row = {'variant': lbl, 'n_runs': n_runs, 'aero_sigma': aero_sigma}
            row.update({k: f"{d[k]:.4f}" for k in stat_keys})
            writer.writerow(row)
    print(f"MC stats → {path_stats}")


# ── entry point ────────────────────────────────────────────────────────────────

def _cli_help():
    print(__doc__)
    print("Registered tests:")
    for k, v in TESTS.items():
        print(f"  {k:<20}  — {v['title']}")


if __name__ == '__main__':
    argv = sys.argv[1:]

    if argv and argv[0] == 'test':
        if len(argv) < 2:
            _cli_help()
            sys.exit(0)
        test_name = argv[1]
        if len(argv) > 2 and argv[2] == 'all':
            test_all_scenarios(test_name)
        else:
            scenario = int(argv[2]) if len(argv) > 2 else 1
            test(test_name, scenario)

    elif argv and argv[0] == 'all':
        run_all()

    elif argv and argv[0] == 'mc':
        # Accepted forms (tokens after 'mc', in any order):
        #   all              → sweep all scenarios
        #   <test_name>      → use that test's variants (must be in TESTS)
        #   <scenario int>   → scenario number (default 1)
        #   <n_runs int>     → number of MC runs (default MC_N_RUNS)
        rest      = argv[1:]
        all_scen  = 'all' in rest
        test_name = next((t for t in rest if t in TESTS), None)
        ints      = [int(t) for t in rest if t.lstrip('-').isdigit()]
        scenario  = ints[0] if ints and not all_scen else 1
        n_runs    = ints[1] if len(ints) > 1 else (ints[0] if ints and all_scen else None)
        variants  = TESTS[test_name]['variants'] if test_name else None

        if all_scen:
            monte_carlo_all_scenarios(variants=variants, n_runs=n_runs)
        else:
            monte_carlo(scenario=scenario, variants=variants, n_runs=n_runs)

    else:
        log, result, _ = run(SCENARIO)
        if log:
            save_csv(log, f's{SCENARIO}')
            SimPlotter(log, SCENARIO, Target.NAMES[SCENARIO], result, HIT_R).render()
