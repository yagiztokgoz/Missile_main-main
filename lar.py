"""
================================================================================
 SRAAM6 — Launch Acceptability Region (LAR)
================================================================================

OVERVIEW
    Computes the LAR by sweeping target position (range × azimuth) at a fixed
    altitude, running one simulation per grid point, and classifying each point
    as "in LAR" (miss ≤ threshold) or "out of LAR".

USAGE
    # Quick estimate (coarse grid, ~60 runs)
    python3 lar.py

    # Medium resolution, head-on targets
    python3 lar.py --aspect 180 --speed 200 --dr 2 --da 15

    # Full fine grid, save only (no window)
    python3 lar.py --dr 1 --da 10 --no-show

PROGRAMMATIC
    from lar import compute_lar, plot_lar

    data = compute_lar(
        ranges_km    = np.arange(2, 26, 2),   # 2–25 km
        azimuths_deg = np.arange(0, 360, 15),  # every 15°
        tgt_speed_ms = 200,
        tgt_aspect_deg = 180,                  # head-on
        miss_threshold_m = 20,
        cfg = {'maut': 6, 'mseek': 0, 'mins': 0},
    )
    plot_lar(data)

ASPECT ANGLE CONVENTION
    tgt_aspect_deg is the angle at the target between the missile-to-target
    LOS and the target velocity vector:
        0°   → tail aspect   (target flies away)
        90°  → beam aspect   (target crosses)
        180° → head-on       (target flies toward missile)
================================================================================
"""

import math
import os
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
from concurrent.futures import ProcessPoolExecutor, as_completed

import config
from target import CustomTarget


# ── worker (must be top-level for pickling) ────────────────────────────────────

def _worker(args):
    """Single LAR grid point simulation. Runs in a child process."""
    north, east, vn, ve, az_deg, miss_thr, sim_cfg = args
    from simulate import run as sim_run

    alt_m = -config.SBEL_INIT[2]
    tgt = CustomTarget(
        pos  = (north, east, -alt_m),
        vel  = (vn,    ve,    0.0),
        name = f'LAR({north:.0f},{east:.0f})',
    )
    run_cfg = dict(sim_cfg)
    run_cfg['psi_init_deg'] = az_deg   # aim missile at target azimuth
    try:
        _log, _result, cpa = sim_run(
            scenario=1, cfg=run_cfg, verbose=False, tgt_override=tgt
        )
        miss = cpa['min_dist']
    except Exception:
        miss = float('inf')

    return miss, miss <= miss_thr


# ── main API ───────────────────────────────────────────────────────────────────

def compute_lar(
    ranges_km      = None,
    azimuths_deg   = None,
    tgt_speed_ms   = 200.0,
    tgt_aspect_deg = 180.0,
    miss_threshold_m = 20.0,
    cfg            = None,
    n_workers      = None,
    verbose        = True,
):
    """Sweep target positions and return miss-distance grid.

    Parameters
    ----------
    ranges_km        : 1-D array of target ranges [km]
    azimuths_deg     : 1-D array of target bearings from missile [deg, 0=North CW]
    tgt_speed_ms     : target airspeed [m/s]
    tgt_aspect_deg   : angle between missile→target LOS and target velocity [deg]
    miss_threshold_m : miss distance threshold for "in LAR" [m]
    cfg              : autopilot/guidance overrides passed to simulate.run()
    n_workers        : parallel workers (default: CPU count - 1)
    verbose          : print progress

    Returns
    -------
    dict with keys: ranges_km, azimuths_deg, miss_grid, hit_grid,
                    tgt_speed_ms, tgt_aspect_deg, miss_threshold_m
    """
    if ranges_km is None:
        ranges_km = np.arange(2.0, 26.0, 2.0)
    if azimuths_deg is None:
        azimuths_deg = np.arange(0, 360, 15)

    ranges_km    = np.asarray(ranges_km,    dtype=float)
    azimuths_deg = np.asarray(azimuths_deg, dtype=float)

    asp_r    = math.radians(tgt_aspect_deg)
    sim_cfg  = dict(cfg or {})
    sim_cfg.setdefault('hit_r',    0.0)
    sim_cfg.setdefault('mseek',    0)
    sim_cfg.setdefault('skip_log', True)   # skip log building — 3-4× faster
    sim_cfg.setdefault('t_end',    30.0)   # max sim time for LAR runs

    # Build task list
    tasks = []
    for r_km in ranges_km:
        r_m = r_km * 1000.0
        for az_deg in azimuths_deg:
            az_r  = math.radians(az_deg)
            north = r_m * math.cos(az_r)
            east  = r_m * math.sin(az_r)

            # LOS unit vector (missile → target)
            los_n, los_e = north / r_m, east / r_m

            # Target velocity: rotate LOS by aspect angle (clockwise in NE plane)
            vn = tgt_speed_ms * ( los_n * math.cos(asp_r) + los_e * math.sin(asp_r))
            ve = tgt_speed_ms * (-los_n * math.sin(asp_r) + los_e * math.cos(asp_r))

            tasks.append((north, east, vn, ve, az_deg, miss_threshold_m, sim_cfg))

    n_total = len(tasks)
    if n_workers is None:
        import multiprocessing
        n_workers = max(1, multiprocessing.cpu_count() - 1)

    if verbose:
        print(f"\nLAR computation: {len(ranges_km)} ranges × {len(azimuths_deg)} azimuths "
              f"= {n_total} runs  ({n_workers} workers)")

    # Parallel execution
    miss_flat = [float('inf')] * n_total
    hit_flat  = [False]        * n_total

    with ProcessPoolExecutor(max_workers=n_workers) as pool:
        future_to_idx = {pool.submit(_worker, t): i for i, t in enumerate(tasks)}
        done = 0
        for future in as_completed(future_to_idx):
            i = future_to_idx[future]
            try:
                miss, hit = future.result()
            except Exception as exc:
                miss, hit = float('inf'), False
                if verbose:
                    print(f"  [warn] task {i} failed: {exc}")
            miss_flat[i] = miss
            hit_flat[i]  = hit
            done += 1
            if verbose and done % max(1, n_total // 10) == 0:
                print(f"  {done}/{n_total} ({100*done//n_total}%)")

    # Reshape to (n_ranges, n_azimuths)
    nr, na = len(ranges_km), len(azimuths_deg)
    miss_grid = np.array(miss_flat).reshape(nr, na)
    hit_grid  = np.array(hit_flat ).reshape(nr, na)

    if verbose:
        n_in  = hit_grid.sum()
        print(f"  Done. In-LAR: {n_in}/{n_total} "
              f"({100*n_in//n_total}%)  miss_thr={miss_threshold_m}m")

    return {
        'ranges_km'       : ranges_km,
        'azimuths_deg'    : azimuths_deg,
        'miss_grid'       : miss_grid,
        'hit_grid'        : hit_grid,
        'tgt_speed_ms'    : tgt_speed_ms,
        'tgt_aspect_deg'  : tgt_aspect_deg,
        'miss_threshold_m': miss_threshold_m,
    }


# ── plotting ───────────────────────────────────────────────────────────────────

# Match plotting.py colour palette and theme
_BLUE   = '#428BCA'
_GREEN  = '#5CB85C'
_RED    = '#D9534F'
_ORANGE = '#F0AD4E'

_THEME = {
    'figure.facecolor':  'white',
    'axes.facecolor':    '#F7F7F7',
    'savefig.facecolor': 'white',
    'axes.edgecolor':    '#CCCCCC',
    'axes.labelcolor':   '#333333',
    'text.color':        '#111111',
    'xtick.color':       '#555555',
    'ytick.color':       '#555555',
    'grid.color':        '#DDDDDD',
    'grid.alpha':        0.9,
    'legend.facecolor':  'white',
    'legend.edgecolor':  '#BBBBBB',
}


def plot_lar(lar_data, save=True, show=True):
    """Render LAR as two side-by-side polar plots (plotting.py style).

    Left  : binary hit/miss with glowing LAR boundary
    Right : miss-distance heatmap with contour lines
    """
    ranges_km    = lar_data['ranges_km']
    azimuths_deg = lar_data['azimuths_deg']
    miss_grid    = lar_data['miss_grid']
    hit_grid     = lar_data['hit_grid']
    miss_thr     = lar_data['miss_threshold_m']
    spd          = lar_data['tgt_speed_ms']
    asp          = lar_data['tgt_aspect_deg']
    n_in         = int(hit_grid.sum())
    n_tot        = hit_grid.size

    azims_r = np.deg2rad(azimuths_deg)
    # close the polar loop so there is no gap at 360°
    azims_closed = np.append(azims_r, azims_r[0] + 2 * math.pi)
    miss_closed  = np.hstack([miss_grid,  miss_grid[:, :1]])
    hit_closed   = np.hstack([hit_grid,   hit_grid[:, :1]])
    R, A = np.meshgrid(ranges_km, azims_closed, indexing='ij')

    miss_plot    = np.where(np.isinf(miss_closed), 500.0, miss_closed)
    miss_clipped = np.clip(miss_plot, 0, 200)

    with plt.rc_context(_THEME):
        fig, (ax1, ax2) = plt.subplots(
            1, 2, figsize=(16, 7),
            subplot_kw={'projection': 'polar'},
        )
        fig.patch.set_facecolor('white')
        fig.suptitle(
            f'SRAAM6  ·  Launch Acceptability Region\n'
            f'V_tgt = {spd:.0f} m/s   aspect = {asp:.0f}°   '
            f'miss threshold = {miss_thr:.0f} m   '
            f'in-LAR: {n_in}/{n_tot} ({100*n_in//n_tot}%)',
            fontsize=13, fontweight='bold', color='#111111', y=1.02,
        )

        def _setup(ax, title):
            ax.set_theta_zero_location('N')
            ax.set_theta_direction(-1)           # clockwise = compass
            ax.set_rlabel_position(40)
            ax.tick_params(labelsize=9, colors='#555555')
            ax.grid(True, linestyle='-',  linewidth=0.7, alpha=0.5, color='#CCCCCC')
            ax.set_title(title, fontsize=12, fontweight='bold',
                         color='#333333', pad=14)
            # range labels in km
            rticks = ax.get_yticks()
            ax.set_yticklabels([f'{r:.0f}' for r in rticks], fontsize=8)
            ax.set_xlabel('Range [km]', labelpad=14, fontsize=10, color='#333333')

        # ── left: hit / miss ──────────────────────────────────────────────────
        _setup(ax1, 'Hit / Miss')

        cmap_bin = mcolors.LinearSegmentedColormap.from_list(
            'lar_bin', ['#c62828', '#2e7d32'], N=2)
        ax1.pcolormesh(A, R, hit_closed.astype(float),
                       cmap=cmap_bin, vmin=0, vmax=1, shading='auto')

        # LAR boundary — solid white line with glow
        if hit_grid.any() and not hit_grid.all():
            for lw, alpha in [(8, 0.18), (3, 0.7)]:
                try:
                    ax1.contour(A, R, hit_closed.astype(float),
                                levels=[0.5], colors='white',
                                linewidths=lw, alpha=alpha)
                except Exception:
                    pass

        # centre dot = missile position
        ax1.scatter([0], [0], s=80, color=_BLUE, zorder=10,
                    label='Missile', transform=ax1.transData)
        ax1.legend(loc='lower right', fontsize=9, framealpha=0.85)

        # ── right: miss-distance heatmap ─────────────────────────────────────
        _setup(ax2, 'Miss Distance')

        pcm = ax2.pcolormesh(A, R, miss_clipped,
                              cmap='RdYlGn_r', shading='auto', vmin=0, vmax=200)
        cbar = plt.colorbar(pcm, ax=ax2, pad=0.13, fraction=0.045, shrink=0.85)
        cbar.set_label('Miss distance [m]', fontsize=10, color='#333333')
        cbar.ax.tick_params(labelsize=9, colors='#555555')

        # miss-threshold contour levels with glow
        levels = sorted({miss_thr, miss_thr * 2, miss_thr * 5})
        colors = [_GREEN, _ORANGE, _RED]
        for lvl, col in zip(levels, colors):
            for lw, al in [(7, 0.15), (2, 0.85)]:
                try:
                    cs = ax2.contour(A, R, miss_plot, levels=[lvl],
                                     colors=[col], linewidths=lw, alpha=al)
                    if al > 0.5 and lw < 4:
                        ax2.clabel(cs, fmt=f'{lvl:.0f} m', fontsize=8,
                                   colors=[col], inline=True)
                except Exception:
                    pass

        # range-ring annotations
        for r_km in ranges_km[::2]:
            ax2.text(math.radians(42), r_km, f'{r_km:.0f} km',
                     fontsize=7, color='#888888', ha='left', va='center')

        plt.tight_layout(pad=2.0)

    if save:
        os.makedirs(config.PLOTDIR, exist_ok=True)
        fname = f'lar_asp{int(asp)}_v{int(spd)}.png'
        path  = os.path.join(config.PLOTDIR, fname)
        fig.savefig(path, dpi=200, bbox_inches='tight', facecolor='white')
        print(f"Saved → {path}")

    if show:
        plt.show()

    return fig


# ── WEZ helpers ───────────────────────────────────────────────────────────────

def _extract_boundary(ranges_km, hit_col):
    """Return (r_max, r_min) in km for one azimuth column of hit flags.

    r_max: outermost hit range  |  r_min: innermost hit range
    Returns (nan, nan) if no hits in this column.
    """
    hit_idx = np.where(hit_col)[0]
    if len(hit_idx) == 0:
        return float('nan'), float('nan')
    return ranges_km[hit_idx[-1]], ranges_km[hit_idx[0]]


def _accel_break(g_load, onset_t=1.5):
    """Return an accel_fn that starts a max-g horizontal break at onset_t seconds."""
    G = 9.80665
    def fn(t, vel):
        if t < onset_t:
            return np.zeros(3)
        vn, ve = float(vel[0]), float(vel[1])
        mag = math.hypot(vn, ve)
        if mag < 1.0:
            return np.zeros(3)
        # 90° clockwise (right break)
        return np.array([ve / mag, -vn / mag, 0.0]) * g_load * G
    return fn


def compute_wez(
    azimuths_deg     = None,
    tgt_speed_ms     = 200.0,
    miss_threshold_m = 20.0,
    r_min_km         = 0.5,
    r_max_km         = 25.0,
    dr_km            = 1.0,
    nez_g            = 5.0,
    cfg              = None,
    n_workers        = None,
    verbose          = True,
):
    """Compute WEZ envelope curves for forward hemisphere.

    Runs two LAR sweeps:
      1. Non-maneuvering target  → R_MAX (outer boundary)
      2. nez_g-g maneuvering target → R_MAX2 / R_MIN2 (NEZ boundaries)

    Returns dict with azimuth arrays and boundary curves in km.
    """
    if azimuths_deg is None:
        azimuths_deg = np.arange(-90, 91, 10)

    azimuths_deg = np.asarray(azimuths_deg, dtype=float)
    ranges_km    = np.arange(r_min_km, r_max_km + dr_km / 2, dr_km)

    # Compass azimuths (0=N, CW): forward hemisphere is 270°→360° + 0°→90°
    # We receive signed azimuths (-90° to +90°) relative to shooter heading
    compass_az = azimuths_deg % 360.0

    base_cfg = dict(cfg or {})
    base_cfg.setdefault('mseek',    0)
    base_cfg.setdefault('mins',     0)
    base_cfg.setdefault('skip_log', True)
    base_cfg.setdefault('t_end',    30.0)

    def _run_lar(g_load):
        """Run LAR sweep; g_load=0 → non-maneuvering, g_load>0 → break maneuver."""
        tasks = []
        for r_km in ranges_km:
            r_m = r_km * 1000.0
            for az_deg in compass_az:
                az_r  = math.radians(az_deg)
                north = r_m * math.cos(az_r)
                east  = r_m * math.sin(az_r)
                los_n, los_e = north / r_m, east / r_m
                vn = -tgt_speed_ms * los_n
                ve = -tgt_speed_ms * los_e
                run_cfg  = dict(base_cfg)
                run_cfg['psi_init_deg'] = az_deg
                tasks.append((north, east, vn, ve, g_load, az_deg,
                               miss_threshold_m, run_cfg))

        import multiprocessing
        nw = n_workers or max(1, multiprocessing.cpu_count() - 1)
        if verbose:
            print(f"  {len(tasks)} runs ({nw} workers)...")

        results = []
        with ProcessPoolExecutor(max_workers=nw) as pool:
            future_to_idx = {pool.submit(_worker_wez, t): i
                             for i, t in enumerate(tasks)}
            done_list = [None] * len(tasks)
            for fut in as_completed(future_to_idx):
                i = future_to_idx[fut]
                try:
                    done_list[i] = fut.result()
                except Exception:
                    done_list[i] = (float('inf'), False)

        nr, na = len(ranges_km), len(compass_az)
        miss_grid = np.array([d[0] for d in done_list]).reshape(nr, na)
        hit_grid  = np.array([d[1] for d in done_list]).reshape(nr, na)
        return miss_grid, hit_grid

    if verbose:
        print("\nWEZ pass 1/2: non-maneuvering target")
    _, hit1 = _run_lar(0.0)

    if verbose:
        print(f"\nWEZ pass 2/2: {nez_g:.0f}-g maneuvering target (NEZ)")
    _, hit2 = _run_lar(nez_g)

    # Extract boundary curves per azimuth
    r_max  = np.full(len(compass_az), np.nan)
    r_min  = np.full(len(compass_az), np.nan)
    r_max2 = np.full(len(compass_az), np.nan)
    r_min2 = np.full(len(compass_az), np.nan)

    for j in range(len(compass_az)):
        r_max[j],  r_min[j]  = _extract_boundary(ranges_km, hit1[:, j])
        r_max2[j], r_min2[j] = _extract_boundary(ranges_km, hit2[:, j])

    return {
        'azimuths_deg'    : azimuths_deg,   # signed (-90..+90)
        'r_max'           : r_max,
        'r_min'           : np.maximum(r_min,  r_min_km),
        'r_max2'          : r_max2,
        'r_min2'          : np.maximum(r_min2, r_min_km),
        'r_min_km'        : r_min_km,
        'tgt_speed_ms'    : tgt_speed_ms,
        'miss_threshold_m': miss_threshold_m,
        'nez_g'           : nez_g,
    }


def _worker_wez(args):
    """WEZ worker — nez_g=0 → straight flight, nez_g>0 → break maneuver."""
    north, east, vn, ve, nez_g, az_deg, miss_thr, sim_cfg = args
    from simulate import run as sim_run
    alt_m = -config.SBEL_INIT[2]
    accel_fn = _accel_break(nez_g) if nez_g > 0 else None
    tgt = CustomTarget(
        pos      = (north, east, -alt_m),
        vel      = (vn,    ve,    0.0),
        accel_fn = accel_fn,
        name     = 'WEZ',
    )
    try:
        _, _, cpa = sim_run(scenario=1, cfg=sim_cfg,
                            verbose=False, tgt_override=tgt)
        miss = cpa['min_dist']
    except Exception:
        miss = float('inf')
    return miss, miss <= miss_thr


def plot_wez(wez_data, save=True, show=True):
    """Classic WEZ fan diagram — shooter at bottom, target ahead."""
    az   = wez_data['azimuths_deg']          # signed degrees
    rmax  = wez_data['r_max']
    rmin  = wez_data['r_min']
    rmax2 = wez_data['r_max2']
    rmin2 = wez_data['r_min2']
    spd   = wez_data['tgt_speed_ms']
    thr   = wez_data['miss_threshold_m']
    nez_g = wez_data['nez_g']

    az_r = np.deg2rad(az)

    def _to_xy(r_km, az_r):
        """Convert (range, signed_azimuth) → (x=East, y=North) km."""
        x = r_km * np.sin(az_r)
        y = r_km * np.cos(az_r)
        return x, y

    def _curve(r_arr):
        mask = ~np.isnan(r_arr)
        if not mask.any():
            return np.array([]), np.array([])
        return _to_xy(r_arr[mask], az_r[mask])

    with plt.rc_context(_THEME):
        fig, ax = plt.subplots(figsize=(10, 12))
        fig.patch.set_facecolor('white')

        ax.set_facecolor('#F7F7F7')
        ax.set_aspect('equal')
        ax.grid(True, linestyle='-', linewidth=0.6, alpha=0.5, color='#CCCCCC')
        ax.set_xlabel('Crossrange [km]', fontsize=11, color='#333333')
        ax.set_ylabel('Downrange [km]',  fontsize=11, color='#333333')
        ax.tick_params(labelsize=9, colors='#555555')
        ax.set_title(
            f'SRAAM6  ·  Weapon Employment Zone\n'
            f'V_tgt = {spd:.0f} m/s   miss_thr = {thr:.0f} m   '
            f'NEZ: {nez_g:.0f}-g break',
            fontsize=13, fontweight='bold', color='#111111', pad=14,
        )

        def _fill_between_curves(r_outer, r_inner, color, alpha, label=None):
            """Fill area between two range boundary curves."""
            mask = ~(np.isnan(r_outer) | np.isnan(r_inner))
            if not mask.any():
                return
            r_o = np.where(mask, r_outer, np.nan)
            r_i = np.where(mask, r_inner, np.nan)
            xo, yo = _to_xy(r_o[mask], az_r[mask])
            xi, yi = _to_xy(r_i[mask], az_r[mask])
            # build closed polygon: outer curve fwd, inner curve reversed
            px = np.concatenate([xo, xi[::-1]])
            py = np.concatenate([yo, yi[::-1]])
            ax.fill(px, py, color=color, alpha=alpha,
                    label=label, zorder=2)

        def _draw_boundary(r_arr, color, lw, label, linestyle='-'):
            x, y = _curve(r_arr)
            if len(x) == 0:
                return
            for lw_, al in [(lw * 3, 0.15), (lw, 0.9)]:
                ax.plot(x, y, color=color, lw=lw_, alpha=al,
                        linestyle=linestyle, zorder=4)
            ax.plot([], [], color=color, lw=lw, linestyle=linestyle, label=label)

        # ── filled regions (back to front) ───────────────────────────────────
        # outer launch zone  (R_MAX2 → R_MAX)
        _fill_between_curves(rmax, rmax2, color='#AED6F1', alpha=0.55,
                             label='Outer launch zone')
        # no escape zone  (R_MIN2 → R_MAX2)
        _fill_between_curves(rmax2, rmin2, color='#E74C3C', alpha=0.30,
                             label='No Escape Zone (NEZ)')
        # inner dead zone  (R_MIN → R_MIN2)
        _fill_between_curves(rmin2, rmin, color='#F39C12', alpha=0.30,
                             label='Inner zone')

        # ── boundary lines ────────────────────────────────────────────────────
        _draw_boundary(rmax,  _BLUE,   2.5, f'R_MAX  (non-maneuvering)')
        _draw_boundary(rmax2, _RED,    2.0, f'R_MAX2 (NEZ outer)')
        _draw_boundary(rmin2, _ORANGE, 2.0, f'R_MIN2 (NEZ inner)', linestyle='--')
        _draw_boundary(rmin,  _BLUE,   1.5, f'R_MIN',              linestyle='--')

        # ── side lines (fan edges at ±max azimuth) ────────────────────────────
        for sign in (-1, 1):
            j = 0 if sign == -1 else -1
            if not np.isnan(rmax[j]):
                xf, yf = _to_xy(rmax[j], az_r[j])
                ax.plot([0, xf], [0, yf], color='#555555',
                        lw=1.2, alpha=0.5, linestyle=':', zorder=3)

        # ── range rings ───────────────────────────────────────────────────────
        for r_km in np.arange(5, np.nanmax(rmax) + 5, 5):
            theta = np.linspace(np.deg2rad(az.min()), np.deg2rad(az.max()), 120)
            ax.plot(r_km * np.sin(theta), r_km * np.cos(theta),
                    color='#BBBBBB', lw=0.8, alpha=0.6, zorder=1)
            ax.text(0, r_km, f'  {r_km:.0f} km', fontsize=7,
                    color='#888888', va='center')

        # ── annotations ───────────────────────────────────────────────────────
        # NEZ label in centre of NEZ region
        nez_vals  = (rmax2 + rmin2) / 2
        nez_valid = nez_vals[~np.isnan(nez_vals)]
        nez_r_mid = float(np.median(nez_valid)) if len(nez_valid) > 0 else float('nan')
        if not np.isnan(nez_r_mid):
            ax.text(0, nez_r_mid, 'NO ESCAPE\nZONE',
                    ha='center', va='center', fontsize=10,
                    fontweight='bold', color='#C0392B', alpha=0.8, zorder=5)

        # Shooter icon (triangle at origin)
        ax.scatter([0], [0], s=200, marker='^', color=_ORANGE,
                   zorder=10, label='Shooter A/C')
        ax.text(0, -0.8, 'Shooter A/C', ha='center', va='top',
                fontsize=9, color='#333333', fontweight='bold')

        # Target icon at top
        y_top = np.nanmax(rmax) * 1.05 if not np.all(np.isnan(rmax)) else 25
        ax.scatter([0], [y_top], s=180, marker='v', color=_BLUE,
                   zorder=10, label='Target A/C')
        ax.text(0, y_top + 0.5, 'Target A/C', ha='center', va='bottom',
                fontsize=9, color='#333333', fontweight='bold')

        ax.legend(loc='lower right', fontsize=8, framealpha=0.9,
                  edgecolor='#BBBBBB')
        plt.tight_layout()

    if save:
        os.makedirs(config.PLOTDIR, exist_ok=True)
        path = os.path.join(config.PLOTDIR, 'wez.png')
        fig.savefig(path, dpi=200, bbox_inches='tight', facecolor='white')
        print(f"Saved → {path}")
    if show:
        plt.show()
    return fig


# ── CLI ────────────────────────────────────────────────────────────────────────

def _parse_args():
    p = argparse.ArgumentParser(description='SRAAM6 LAR / WEZ computation')
    p.add_argument('mode',      nargs='?',  default='lar', choices=['lar', 'wez'],
                   help='lar: full 360° polar  |  wez: forward-hemisphere fan diagram')
    p.add_argument('--speed',   type=float, default=200,  help='Target speed [m/s]')
    p.add_argument('--aspect',  type=float, default=180,  help='Aspect angle [deg] (LAR only)')
    p.add_argument('--miss',    type=float, default=20,   help='Miss threshold [m]')
    p.add_argument('--dr',      type=float, default=2,    help='Range step [km]')
    p.add_argument('--rmax',    type=float, default=25,   help='Max range [km]')
    p.add_argument('--rmin',    type=float, default=1,    help='Min range [km]')
    p.add_argument('--da',      type=float, default=15,   help='Azimuth step [deg]')
    p.add_argument('--maut',    type=int,   default=5,    help='Autopilot (5=NDI, 6=INDI)')
    p.add_argument('--workers', type=int,   default=None, help='Parallel workers')
    p.add_argument('--no-show', action='store_true',      help='Do not open plot window')
    return p.parse_args()


if __name__ == '__main__':
    args = _parse_args()

    if args.mode == 'wez':
        wez = compute_wez(
            azimuths_deg     = np.arange(-90, 91, args.da),
            tgt_speed_ms     = args.speed,
            miss_threshold_m = args.miss,
            r_min_km         = args.rmin,
            r_max_km         = args.rmax,
            dr_km            = args.dr,
            cfg              = {'maut': args.maut, 'mins': 0, 'mseek': 0},
            n_workers        = args.workers,
        )
        plot_wez(wez, save=True, show=not args.no_show)
    else:
        data = compute_lar(
            ranges_km        = np.arange(args.rmin, args.rmax + args.dr/2, args.dr),
            azimuths_deg     = np.arange(0, 360, args.da),
            tgt_speed_ms     = args.speed,
            tgt_aspect_deg   = args.aspect,
            miss_threshold_m = args.miss,
            cfg              = {'maut': args.maut, 'mins': 0, 'mseek': 0},
            n_workers        = args.workers,
        )
        plot_lar(data, save=True, show=not args.no_show)
