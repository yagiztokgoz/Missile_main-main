"""
SRAAM6 plotting — packaged SimPlotter class.

Produces three figures (trajectory, state variables, actuator) with a
consistent theme and glow effect, saves them as PNG under plots/.
"""

import os
import sys
import numpy as np

# Ensure venv matplotlib/Pillow is preferred over system one.
_venv = [p for p in sys.path if 'venv' in p and 'site-packages' in p]
_rest = [p for p in sys.path if p not in _venv]
sys.path = _venv + _rest

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401


# ── palette ────────────────────────────────────────────────────────────────────
BLUE   = '#428BCA'
RED    = '#D9534F'
GREEN  = '#5CB85C'
ORANGE = '#F0AD4E'
PURPLE = '#9467BD'
TEAL   = '#17A2B8'
C4     = [BLUE, ORANGE, GREEN, RED]
# palette used by ComparisonPlotter to distinguish variants
CVAR   = [BLUE, RED, GREEN, ORANGE, PURPLE, TEAL]

FIN_LIM = 28.0  # m.dlimx mirror; matches control-fin physical limit

THEME = {
    'figure.facecolor':   'white',
    'axes.facecolor':     '#F7F7F7',
    'savefig.facecolor':  'white',
    'axes.edgecolor':     '#CCCCCC',
    'axes.labelcolor':    '#333333',
    'text.color':         '#111111',
    'xtick.color':        '#555555',
    'ytick.color':        '#555555',
    'grid.color':         '#DDDDDD',
    'grid.alpha':         0.9,
    'legend.facecolor':   'white',
    'legend.edgecolor':   '#BBBBBB',
    'legend.fontsize':    9,
}


class SimPlotter:
    """Render and save the three standard SRAAM6 diagnostic figures.

    Usage:
        SimPlotter(log, scenario, name, result, hit_r).render()
    """

    def __init__(self, log, scenario, name, result,
                 hit_r=30.0, outdir='plots'):
        self.log      = log
        self.scenario = scenario
        self.name     = name
        self.result   = result
        self.hit_r    = hit_r
        self.outdir   = outdir

        # Extract columns once.
        self._extract()

    # ── helpers ────────────────────────────────────────────────────────────────

    def _c(self, key):
        return np.array([r[key] for r in self.log])

    def _extract(self):
        c = self._c
        self.t       = c('t')
        self.sbel1   = c('sbel1');   self.sbel2   = c('sbel2');   self.hbe     = c('hbe')
        self.stel1   = c('stel1');   self.stel2   = c('stel2');   self.stel_hbe= c('stel_hbe')
        self.dvbe    = c('dvbe');    self.mach    = c('mach');    self.pdynmc  = c('pdynmc')
        # INS diagnostics (optional — absent on pre-INS logs)
        if 'esttc1' in self.log[0]:
            self.esttc1 = c('esttc1'); self.esttc2 = c('esttc2'); self.esttc3 = c('esttc3')
            self.evbe1  = c('evbe1');  self.evbe2  = c('evbe2');  self.evbe3  = c('evbe3')
            self.rece1x = c('rece1x'); self.rece2x = c('rece2x'); self.rece3x = c('rece3x')
            self.has_ins = True
        else:
            self.has_ins = False
        self.psivlx  = c('psivlx');  self.thtvlx  = c('thtvlx')
        self.alphax  = c('alphax');  self.betax   = c('betax');   self.alppx   = c('alppx')
        self.psiblx  = c('psiblx');  self.thtblx  = c('thtblx');  self.phiblx  = c('phiblx')
        self.ppx     = c('ppx');     self.qqx     = c('qqx');     self.rrx     = c('rrx')
        self.dpcx    = c('dpcx');    self.dqcx    = c('dqcx');    self.drcx    = c('drcx')
        self.dpx     = c('dpx');     self.dqx     = c('dqx');     self.drx     = c('drx')
        self.delcx1  = c('delcx1');  self.delcx2  = c('delcx2')
        self.delcx3  = c('delcx3');  self.delcx4  = c('delcx4')
        self.delx1   = c('delx1');   self.delx2   = c('delx2')
        self.delx3   = c('delx3');   self.delx4   = c('delx4')
        self.thrust  = c('thrust');  self.mass    = c('mass');    self.xcg     = c('xcg')
        self.anx     = c('anx');     self.ayx     = c('ayx')
        self.ancomx  = c('ancomx');  self.alcomx  = c('alcomx')
        self.dtbc    = c('dtbc');    self.tgoc    = c('tgoc')

    @staticmethod
    def _grid(ax):
        ax.grid(True, which='major', linestyle='-', linewidth=0.8, alpha=0.6)
        ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.4)
        ax.minorticks_on()

    # ── figure 1: trajectory ──────────────────────────────────────────────────

    def _fig_trajectory(self):
        s, name, result = self.scenario, self.name, self.result
        fig_title = (f"SRAAM6  ·  Scenario {s}  —  "
                     f"{name.split('|')[0].strip()}\n{result}")

        fig = plt.figure(f'S{s} Trajectory', figsize=(16, 9))
        fig.suptitle(fig_title, fontsize=15, fontweight='bold',
                     y=0.96, color='#111111')

        all_x = np.concatenate([self.sbel1, self.stel1])
        all_y = np.concatenate([self.sbel2, self.stel2])
        all_z = np.concatenate([self.hbe,   self.stel_hbe])
        mid_x = (all_x.max() + all_x.min()) * 0.5
        mid_y = (all_y.max() + all_y.min()) * 0.5
        mid_z = (all_z.max() + all_z.min()) * 0.5
        max_range = max(
            all_x.max() - all_x.min(),
            all_y.max() - all_y.min(),
            all_z.max() - all_z.min(),
        ) * 0.5

        # ── 3D ──
        ax3d = fig.add_subplot(221, projection='3d')
        ax3d.set_facecolor('white')
        for axis in (ax3d.xaxis, ax3d.yaxis, ax3d.zaxis):
            axis.set_pane_color((0.97, 0.97, 0.97, 1.0))
            axis.line.set_color('#AAAAAA')

        ax3d.plot(self.sbel1, self.sbel2, self.hbe,      color=BLUE, lw=2.5, label='Missile Path')
        ax3d.plot(self.sbel1, self.sbel2, self.hbe,      color=BLUE, lw=7.0, alpha=0.15)
        ax3d.plot(self.stel1, self.stel2, self.stel_hbe, color=RED,  lw=2.0, ls='--', label='Target Path')
        ax3d.plot(self.stel1, self.stel2, self.stel_hbe, color=RED,  lw=6.0, alpha=0.15)

        z_min = mid_z - max_range
        ax3d.plot(self.sbel1, self.sbel2, np.full_like(self.hbe, z_min),
                  color=BLUE, lw=1.5, alpha=0.4, ls=':')
        ax3d.plot(self.stel1, self.stel2, np.full_like(self.stel_hbe, z_min),
                  color=RED,  lw=1.5, alpha=0.4, ls=':')

        ax3d.scatter(self.sbel1[0],  self.sbel2[0],  self.hbe[0],      c=BLUE, s=60,  label='Launch', zorder=5)
        ax3d.scatter(self.sbel1[-1], self.sbel2[-1], self.hbe[-1],     c=BLUE, s=150, marker='*', zorder=6)
        ax3d.scatter(self.stel1[-1], self.stel2[-1], self.stel_hbe[-1], c=RED,  s=150, marker='X', zorder=6, label='Intercept/End')
        ax3d.set_xlabel('North [m]', labelpad=10)
        ax3d.set_ylabel('East [m]',  labelpad=10)
        ax3d.set_zlabel('Altitude [m]', labelpad=10)
        ax3d.set_xlim(mid_x - max_range, mid_x + max_range)
        ax3d.set_ylim(mid_y - max_range, mid_y + max_range)
        ax3d.set_zlim(mid_z - max_range, mid_z + max_range)
        ax3d.set_title('3D Engagement Geometry', fontsize=12, fontweight='bold', pad=15, color='#333333')
        ax3d.legend(loc='best', framealpha=0.9)
        ax3d.view_init(elev=25, azim=-55)

        # ── top view ──
        ax_top = fig.add_subplot(222)
        ax_top.plot(self.sbel1, self.sbel2, color=BLUE, lw=2.5, label='Missile')
        ax_top.plot(self.sbel1, self.sbel2, color=BLUE, lw=7.0, alpha=0.15)
        ax_top.plot(self.stel1, self.stel2, color=RED,  lw=2.0, ls='--', label='Target')
        ax_top.plot(self.stel1, self.stel2, color=RED,  lw=6.0, alpha=0.15)
        ax_top.scatter(self.sbel1[0],  self.sbel2[0],  c=BLUE, s=60)
        ax_top.scatter(self.sbel1[-1], self.sbel2[-1], c=BLUE, s=150, marker='*', zorder=5, label='End')
        ax_top.scatter(self.stel1[-1], self.stel2[-1], c=RED,  s=150, marker='X', zorder=5)
        ax_top.set_xlabel('North [m]'); ax_top.set_ylabel('East [m]')
        ax_top.set_title('Top View (N-E Plane)', fontsize=12, fontweight='bold', color='#333333')
        self._grid(ax_top)
        ax_top.set_aspect('equal', adjustable='datalim')

        # ── side view ──
        ax_side = fig.add_subplot(223)
        ax_side.plot(self.sbel1, self.hbe,      color=BLUE, lw=2.5, label='Missile')
        ax_side.plot(self.sbel1, self.hbe,      color=BLUE, lw=7.0, alpha=0.15)
        ax_side.fill_between(self.sbel1, 0, self.hbe, color=BLUE, alpha=0.05)
        ax_side.plot(self.stel1, self.stel_hbe, color=RED,  lw=2.0, ls='--', label='Target')
        ax_side.plot(self.stel1, self.stel_hbe, color=RED,  lw=6.0, alpha=0.15)
        ax_side.fill_between(self.stel1, 0, self.stel_hbe, color=RED, alpha=0.05)
        ax_side.scatter(self.sbel1[0],  self.hbe[0],      c=BLUE, s=60)
        ax_side.scatter(self.sbel1[-1], self.hbe[-1],     c=BLUE, s=150, marker='*', zorder=5)
        ax_side.scatter(self.stel1[-1], self.stel_hbe[-1], c=RED,  s=150, marker='X', zorder=5)
        ax_side.set_xlabel('North [m]'); ax_side.set_ylabel('Altitude [m]')
        ax_side.set_title('Side View (N-Alt Plane)', fontsize=12, fontweight='bold', color='#333333')
        self._grid(ax_side)

        # Prevent tiny altitude changes from looking like a dive.
        if (all_z.max() - all_z.min()) < 3000.0:
            ax_side.set_ylim(mid_z - 1500.0, mid_z + 1500.0)

        # ── range ──
        ax_rng = fig.add_subplot(224)
        ax_rng.plot(self.t, self.dtbc, color=GREEN, lw=2.5, label='Slant Range')
        ax_rng.plot(self.t, self.dtbc, color=GREEN, lw=7.0, alpha=0.15)
        ax_rng.fill_between(self.t, 0, self.dtbc, color=GREEN, alpha=0.05)
        ax_rng.axhline(self.hit_r, color=RED, ls='--', lw=1.5,
                       label=f'Hit Threshold ({self.hit_r} m)')
        ax_rng.set_xlabel('Time [s]'); ax_rng.set_ylabel('Range [m]')
        ax_rng.set_title('Distance to Target', fontsize=12, fontweight='bold', color='#333333')
        ax_rng.legend(loc='best', framealpha=0.9)
        self._grid(ax_rng)

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        return fig

    # ── figure 2: state variables ─────────────────────────────────────────────

    def _fig_states(self):
        s = self.scenario
        fig, axes = plt.subplots(4, 4, figsize=(22, 14),
                                 num=f'S{s} State Variables')
        fig.suptitle(f'SRAAM6  ·  Scenario {s}  —  State Variables',
                     fontsize=15, fontweight='bold', y=0.97, color='#111111')

        t = self.t

        def panel(r, c, ys, labels, title, ylabel, colors=C4):
            ax = axes[r][c]
            for y, lbl, col in zip(ys, labels, colors):
                ax.plot(t, y, color=col, lw=2.0, label=lbl)
                ax.plot(t, y, color=col, lw=6.0, alpha=0.15)
                ax.fill_between(t, 0, y, color=col, alpha=0.05)
            ax.set_title(title, fontsize=12, fontweight='bold', color='#333333')
            ax.set_xlabel('Time [s]', fontsize=10)
            ax.set_ylabel(ylabel, fontsize=10)
            ax.legend(fontsize=8, loc='best', framealpha=0.9)
            self._grid(ax)
            ax.tick_params(labelsize=9)

        panel(0, 0, [self.dvbe],                  ['Speed'],              'Speed',            'm/s')
        panel(0, 1, [self.mach],                  ['Mach'],               'Mach Number',      '–')
        panel(0, 2, [self.pdynmc],                ['q'],                  'Dynamic Pressure', 'kPa')
        panel(0, 3, [self.hbe],                   ['Altitude'],           'Altitude',         'm')

        panel(1, 0, [self.alphax, self.betax],    ['α AoA', 'β sideslip'], 'Incidence Angles', 'deg')
        panel(1, 1, [self.alppx],                 ['αₜ total AoA'],        'Total AoA',        'deg')
        panel(1, 2, [self.psiblx, self.thtblx, self.phiblx],
                    ['ψ yaw', 'θ pitch', 'φ roll'], 'Euler Angles',        'deg')
        panel(1, 3, [self.psivlx, self.thtvlx],
                    ['ψᵥ heading', 'γ FPA'],       'Flight Path Angles',  'deg')

        panel(2, 0, [self.ppx, self.qqx, self.rrx],
                    ['p roll', 'q pitch', 'r yaw'], 'Angular Rates',      'deg/s')
        panel(2, 1, [self.dpcx, self.dqcx, self.drcx],
                    ['δp cmd', 'δq cmd', 'δr cmd'], 'Fin Commands',       'deg')
        panel(2, 2, [self.dpx, self.dqx, self.drx],
                    ['δp act', 'δq act', 'δr act'], 'Fin Deflections',    'deg')
        panel(2, 3, [self.anx, self.ayx],
                    ['aₙ normal', 'aᵧ lateral'],    'Achieved Accel',     'g')

        panel(3, 0, [self.ancomx, self.alcomx],
                    ['aₙ cmd', 'aₗ cmd'],           'Guidance Commands',  'g')
        panel(3, 1, [self.dtbc, self.tgoc],
                    ['Range (m)', 'TGO (s)'],       'Range & TGO',        'm / s')
        panel(3, 2, [self.thrust], ['Thrust'],       'Thrust',             'N', colors=[RED])

        # Mass + CG twin-axis panel (custom — panel() can't do twins)
        ax_m = axes[3][3]
        ax_m.plot(t, self.mass, color=BLUE, lw=2.0, label='Mass [kg]')
        ax_m.plot(t, self.mass, color=BLUE, lw=6.0, alpha=0.15)
        ax_m.set_title('Mass & CG', fontsize=12, fontweight='bold', color='#333333')
        ax_m.set_xlabel('Time [s]', fontsize=10)
        ax_m.set_ylabel('Mass [kg]', fontsize=10)
        self._grid(ax_m)
        ax_m.tick_params(labelsize=9)
        ax_cg = ax_m.twinx()
        ax_cg.plot(t, self.xcg, color=ORANGE, lw=2.0, ls='--', label='CG [m]')
        ax_cg.plot(t, self.xcg, color=ORANGE, lw=6.0, alpha=0.15)
        ax_cg.set_ylabel('CG from nose [m]', fontsize=10)
        ax_cg.tick_params(labelsize=9)
        h1, l1 = ax_m.get_legend_handles_labels()
        h2, l2 = ax_cg.get_legend_handles_labels()
        ax_m.legend(h1 + h2, l1 + l2, fontsize=8, loc='best', framealpha=0.9)

        fig.tight_layout(rect=[0, 0, 1, 0.97])
        return fig

    # ── figure 3: actuator ────────────────────────────────────────────────────

    def _fig_actuator(self):
        s = self.scenario
        fig, axf = plt.subplots(4, 2, figsize=(14, 12),
                                num=f'S{s} Actuator', sharex=True)
        fig.suptitle(f'SRAAM6  ·  Scenario {s}  —  Actuator: Commands vs Actual',
                     fontsize=15, fontweight='bold', y=0.96, color='#111111')

        t = self.t
        cmds = [self.delcx1, self.delcx2, self.delcx3, self.delcx4]
        acts = [self.delx1,  self.delx2,  self.delx3,  self.delx4]

        for i, (cmd, act) in enumerate(zip(cmds, acts)):
            # left: command vs actual overlay
            axl = axf[i][0]
            axl.plot(t, cmd, color=ORANGE, lw=1.8, ls='--', label=f'δ{i+1} Cmd')
            axl.plot(t, act, color=BLUE,   lw=2.0,            label=f'δ{i+1} Act')
            axl.plot(t, act, color=BLUE,   lw=6.0, alpha=0.15)
            axl.fill_between(t, 0, act, color=BLUE, alpha=0.05)
            axl.axhline( FIN_LIM, color=RED, ls=':', lw=1.5, alpha=0.7)
            axl.axhline(-FIN_LIM, color=RED, ls=':', lw=1.5, alpha=0.7)
            axl.set_title(f'Fin {i+1}  —  Command vs Actual', fontsize=12, fontweight='bold', color='#333333')
            axl.set_ylabel('Deflection [deg]', fontsize=10)
            axl.legend(fontsize=9, loc='best', framealpha=0.9)
            self._grid(axl)
            axl.tick_params(labelsize=9)

            # right: tracking error
            axr = axf[i][1]
            err = cmd - act
            axr.plot(t, err, color=RED, lw=2.0, label=f'δ{i+1} Error')
            axr.plot(t, err, color=RED, lw=6.0, alpha=0.15)
            axr.fill_between(t, 0, err, color=RED, alpha=0.05)
            axr.axhline(0, color='#555555', lw=1.0, alpha=0.5)
            axr.set_title(f'Fin {i+1}  —  Tracking Error', fontsize=12, fontweight='bold', color='#333333')
            axr.set_ylabel('Error [deg]', fontsize=10)
            axr.legend(fontsize=9, loc='best', framealpha=0.9)
            self._grid(axr)
            axr.tick_params(labelsize=9)

        axf[-1][0].set_xlabel('Time [s]', fontsize=10)
        axf[-1][1].set_xlabel('Time [s]', fontsize=10)
        fig.tight_layout(rect=[0, 0, 1, 0.97])
        return fig

    # ── figure 4: INS errors (optional) ───────────────────────────────────────

    def _fig_ins(self):
        s = self.scenario
        fig, axes = plt.subplots(3, 1, figsize=(14, 10),
                                 num=f'S{s} INS Errors', sharex=True)
        fig.suptitle(f'SRAAM6  ·  Scenario {s}  —  INS Errors (truth − estimate)',
                     fontsize=15, fontweight='bold', y=0.97, color='#111111')

        t = self.t

        def panel(ax, ys, labels, title, ylabel, colors=C4):
            for y, lbl, col in zip(ys, labels, colors):
                ax.plot(t, y, color=col, lw=2.0, label=lbl)
                ax.plot(t, y, color=col, lw=6.0, alpha=0.15)
            ax.axhline(0, color='#555555', lw=1.0, alpha=0.5)
            ax.set_title(title, fontsize=12, fontweight='bold', color='#333333')
            ax.set_ylabel(ylabel, fontsize=10)
            ax.legend(fontsize=9, loc='best', framealpha=0.9)
            self._grid(ax)
            ax.tick_params(labelsize=9)

        panel(axes[0], [self.esttc1, self.esttc2, self.esttc3],
              ['N', 'E', 'D'], 'Position Error  (ESTTC)', 'm')
        panel(axes[1], [self.evbe1,  self.evbe2,  self.evbe3],
              ['N', 'E', 'D'], 'Velocity Error  (EVBE)',  'm/s')
        panel(axes[2], [self.rece1x, self.rece2x, self.rece3x],
              ['roll', 'pitch', 'yaw'], 'Tilt Error  (RECE)', 'deg')
        axes[-1].set_xlabel('Time [s]', fontsize=10)

        fig.tight_layout(rect=[0, 0, 1, 0.96])
        return fig

    # ── public API ────────────────────────────────────────────────────────────

    def render(self, save=True, show=True):
        plt.rcParams.update(THEME)

        fig1 = self._fig_trajectory()
        fig2 = self._fig_states()
        fig3 = self._fig_actuator()
        fig4 = self._fig_ins() if self.has_ins else None

        if save:
            os.makedirs(self.outdir, exist_ok=True)
            s = self.scenario
            fig1.savefig(f'{self.outdir}/s{s}_trajectory.png',
                         dpi=300, bbox_inches='tight', facecolor='white')
            fig2.savefig(f'{self.outdir}/s{s}_states.png',
                         dpi=300, bbox_inches='tight', facecolor='white')
            fig3.savefig(f'{self.outdir}/s{s}_actuator.png',
                         dpi=300, bbox_inches='tight', facecolor='white')
            if fig4 is not None:
                fig4.savefig(f'{self.outdir}/s{s}_ins.png',
                             dpi=300, bbox_inches='tight', facecolor='white')
            print(f"\n[✓] Tüm grafikler '{self.outdir}' klasörüne PNG formatında kaydedildi.")

        if show:
            plt.show()

        return fig1, fig2, fig3, fig4


# ── ComparisonPlotter ────────────────────────────────────────────────────────

class ComparisonPlotter:
    """Overlay multiple simulation runs on the same axes for variant comparison.

    Usage:
        ComparisonPlotter(logs_dict, scenario, name, test_name, results_dict).render()

    logs_dict    : {label: log}      each log is a list[dict]
    results_dict : {label: result}   terminal message per variant (for legend)
    """

    def __init__(self, logs, scenario, name, test_name, results,
                 outdir='plots'):
        self.logs      = logs            # dict label → list[row-dict]
        self.scenario  = scenario
        self.name      = name
        self.test_name = test_name
        self.results   = results
        self.outdir    = outdir

        self.labels = list(logs.keys())
        self.colors = {lbl: CVAR[i % len(CVAR)] for i, lbl in enumerate(self.labels)}

    # ── helpers ──────────────────────────────────────────────────────────────

    def _col(self, log, key):
        return np.array([r[key] for r in log])

    @staticmethod
    def _grid(ax):
        ax.grid(True, which='major', linestyle='-', linewidth=0.8, alpha=0.6)
        ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.4)
        ax.minorticks_on()

    def _overlay(self, ax, ykey, title, ylabel, xkey='t',
                 zero_line=False, abs_val=False):
        """Plot `ykey` vs `xkey` for each variant on `ax`."""
        for lbl, log in self.logs.items():
            if not log or ykey not in log[0]:
                continue
            x = self._col(log, xkey)
            y = self._col(log, ykey)
            if abs_val:
                y = np.abs(y)
            col = self.colors[lbl]
            ax.plot(x, y, color=col, lw=2.0, label=lbl)
            ax.plot(x, y, color=col, lw=6.0, alpha=0.12)
        if zero_line:
            ax.axhline(0, color='#555555', lw=0.8, alpha=0.5)
        ax.set_title(title, fontsize=11, fontweight='bold', color='#333333')
        ax.set_xlabel(xkey if xkey != 't' else 'Time [s]', fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax)
        ax.tick_params(labelsize=8)

    def _overlay_pair(self, ax, key_cmd, key_ach, title, ylabel):
        """Plot commanded vs achieved pair for each variant (cmd dashed, ach solid)."""
        for lbl, log in self.logs.items():
            if not log:
                continue
            t   = self._col(log, 't')
            cmd = self._col(log, key_cmd)
            ach = self._col(log, key_ach)
            col = self.colors[lbl]
            ax.plot(t, cmd, color=col, lw=1.6, ls='--', alpha=0.8, label=f'{lbl} cmd')
            ax.plot(t, ach, color=col, lw=2.0,              label=f'{lbl} ach')
        ax.axhline(0, color='#555555', lw=0.8, alpha=0.5)
        ax.set_title(title, fontsize=11, fontweight='bold', color='#333333')
        ax.set_xlabel('Time [s]', fontsize=9)
        ax.set_ylabel(ylabel, fontsize=9)
        ax.legend(fontsize=7, loc='best', framealpha=0.9, ncol=2)
        self._grid(ax)
        ax.tick_params(labelsize=8)

    # ── figures ──────────────────────────────────────────────────────────────

    def _fig_overview(self):
        """3×3 overview: trajectory + performance + control."""
        s = self.scenario
        fig, ax = plt.subplots(3, 3, figsize=(20, 14),
                               num=f'S{s} · {self.test_name}')
        title = (f"SRAAM6  ·  Scenario {s}  —  {self.name.split('|')[0].strip()}\n"
                 f"TEST: {self.test_name}")
        fig.suptitle(title, fontsize=15, fontweight='bold', y=0.98, color='#111111')

        # ── row 1: geometry ───────────────────────────────────────────────────
        # (0,0) top view
        ax_top = ax[0][0]
        for lbl, log in self.logs.items():
            if not log:
                continue
            col = self.colors[lbl]
            x = self._col(log, 'sbel1'); y = self._col(log, 'sbel2')
            tx = self._col(log, 'stel1'); ty = self._col(log, 'stel2')
            ax_top.plot(x, y, color=col, lw=2.0, label=f'{lbl} missile')
            ax_top.plot(x, y, color=col, lw=6.0, alpha=0.12)
            ax_top.plot(tx, ty, color=col, lw=1.4, ls=':', alpha=0.8)
            ax_top.scatter(x[-1], y[-1], c=col, s=80, marker='*', zorder=5)
            ax_top.scatter(tx[-1], ty[-1], c=col, s=80, marker='X', zorder=5)
        ax_top.set_title('Top View (N-E)', fontsize=11, fontweight='bold', color='#333333')
        ax_top.set_xlabel('North [m]', fontsize=9); ax_top.set_ylabel('East [m]', fontsize=9)
        ax_top.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax_top); ax_top.tick_params(labelsize=8)
        ax_top.set_aspect('equal', adjustable='datalim')

        # (0,1) side view
        ax_side = ax[0][1]
        for lbl, log in self.logs.items():
            if not log:
                continue
            col = self.colors[lbl]
            x  = self._col(log, 'sbel1'); h  = self._col(log, 'hbe')
            tx = self._col(log, 'stel1'); th = self._col(log, 'stel_hbe')
            ax_side.plot(x, h, color=col, lw=2.0, label=f'{lbl}')
            ax_side.plot(x, h, color=col, lw=6.0, alpha=0.12)
            ax_side.plot(tx, th, color=col, lw=1.4, ls=':', alpha=0.8)
        ax_side.set_title('Side View (N-Alt)', fontsize=11, fontweight='bold', color='#333333')
        ax_side.set_xlabel('North [m]', fontsize=9); ax_side.set_ylabel('Altitude [m]', fontsize=9)
        ax_side.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax_side); ax_side.tick_params(labelsize=8)

        # (0,2) range to target — with CPA markers
        ax_rng = ax[0][2]
        for lbl, log in self.logs.items():
            if not log:
                continue
            col = self.colors[lbl]
            t  = self._col(log, 't'); d = self._col(log, 'dtbc')
            idx = int(np.argmin(d))
            ax_rng.plot(t, d, color=col, lw=2.0, label=f'{lbl}  (miss={d[idx]:.2f} m)')
            ax_rng.plot(t, d, color=col, lw=6.0, alpha=0.12)
            ax_rng.scatter(t[idx], d[idx], c=col, s=80, marker='v', zorder=5)
        ax_rng.set_title('Distance to Target (CPA marked)', fontsize=11, fontweight='bold', color='#333333')
        ax_rng.set_xlabel('Time [s]', fontsize=9); ax_rng.set_ylabel('Range [m]', fontsize=9)
        ax_rng.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax_rng); ax_rng.tick_params(labelsize=8)

        # ── row 2: flight mechanics response ──────────────────────────────────
        self._overlay(ax[1][0], 'alphax', 'Angle of Attack α', 'deg', zero_line=True)
        self._overlay(ax[1][1], 'qqx',    'Pitch Rate q',       'deg/s', zero_line=True)
        self._overlay(ax[1][2], 'dqcx',   'Pitch Fin Command δq', 'deg', zero_line=True)

        # ── row 3: accel tracking ─────────────────────────────────────────────
        self._overlay_pair(ax[2][0], 'ancomx', 'anx', 'Normal Accel (cmd vs ach)', 'g')
        self._overlay_pair(ax[2][1], 'alcomx', 'ayx', 'Lateral Accel (cmd vs ach)', 'g')
        self._overlay(ax[2][2], 'tr_err_mag', 'Accel Tracking Error |cmd−ach|', 'g', zero_line=True)

        fig.tight_layout(rect=[0, 0, 1, 0.96])
        return fig

    def _fig_control(self):
        """3×3 control / attitude / INS detail."""
        s = self.scenario
        fig, ax = plt.subplots(3, 3, figsize=(20, 14),
                               num=f'S{s} · {self.test_name} · Control')
        title = (f"SRAAM6  ·  Scenario {s}  —  CONTROL DETAIL\n"
                 f"TEST: {self.test_name}")
        fig.suptitle(title, fontsize=15, fontweight='bold', y=0.98, color='#111111')

        # row 1: body rates
        self._overlay(ax[0][0], 'ppx', 'Roll Rate p',  'deg/s', zero_line=True)
        self._overlay(ax[0][1], 'qqx', 'Pitch Rate q', 'deg/s', zero_line=True)
        self._overlay(ax[0][2], 'rrx', 'Yaw Rate r',   'deg/s', zero_line=True)

        # row 2: Euler angles
        self._overlay(ax[1][0], 'phiblx', 'Bank Angle φ',  'deg', zero_line=True)
        self._overlay(ax[1][1], 'thtblx', 'Pitch Angle θ', 'deg', zero_line=True)
        self._overlay(ax[1][2], 'psiblx', 'Yaw Angle ψ',   'deg', zero_line=True)

        # row 3: fin commands vs actual (pitch + yaw), and speed
        self._overlay_pair(ax[2][0], 'dqcx', 'dqx', 'δq fin (cmd vs actual)', 'deg')
        self._overlay_pair(ax[2][1], 'drcx', 'drx', 'δr fin (cmd vs actual)', 'deg')
        self._overlay(ax[2][2], 'dvbe', 'Speed', 'm/s')

        fig.tight_layout(rect=[0, 0, 1, 0.96])
        return fig

    def _fig_ins(self):
        """3×1 INS error comparison (only meaningful if any variant has mins=1)."""
        # check if any variant has non-zero INS error state
        has_ins_err = any(
            log and any(abs(r.get('evbe1', 0)) + abs(r.get('esttc1', 0)) > 1e-9 for r in log)
            for log in self.logs.values()
        )
        if not has_ins_err:
            return None

        s = self.scenario
        fig, ax = plt.subplots(3, 1, figsize=(14, 10),
                               num=f'S{s} · {self.test_name} · INS',
                               sharex=True)
        fig.suptitle(f"SRAAM6  ·  Scenario {s}  —  INS Error Overlay\n"
                     f"TEST: {self.test_name}",
                     fontsize=14, fontweight='bold', y=0.98, color='#111111')

        # Position, Velocity, Tilt error magnitudes
        for i, (keys, title, ylabel) in enumerate([
            (('esttc1', 'esttc2', 'esttc3'), 'Position Error magnitude', 'm'),
            (('evbe1',  'evbe2',  'evbe3'),  'Velocity Error magnitude', 'm/s'),
            (('rece1x', 'rece2x', 'rece3x'), 'Tilt Error magnitude',     'deg'),
        ]):
            for lbl, log in self.logs.items():
                if not log:
                    continue
                t = self._col(log, 't')
                a = self._col(log, keys[0])
                b = self._col(log, keys[1])
                c = self._col(log, keys[2])
                mag = np.sqrt(a*a + b*b + c*c)
                col = self.colors[lbl]
                ax[i].plot(t, mag, color=col, lw=2.0, label=lbl)
                ax[i].plot(t, mag, color=col, lw=6.0, alpha=0.12)
            ax[i].set_title(title, fontsize=11, fontweight='bold', color='#333333')
            ax[i].set_ylabel(ylabel, fontsize=9)
            ax[i].legend(fontsize=8, loc='best', framealpha=0.9)
            self._grid(ax[i]); ax[i].tick_params(labelsize=8)

        ax[-1].set_xlabel('Time [s]', fontsize=9)
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        return fig

    # ── public API ───────────────────────────────────────────────────────────

    def render(self, save=True, show=True):
        plt.rcParams.update(THEME)

        fig1 = self._fig_overview()
        fig2 = self._fig_control()
        fig3 = self._fig_ins()

        if save:
            os.makedirs(self.outdir, exist_ok=True)
            s   = self.scenario
            tag = self.test_name.replace(' ', '_').replace('/', '_')
            fig1.savefig(f'{self.outdir}/s{s}_cmp_{tag}_overview.png',
                         dpi=200, bbox_inches='tight', facecolor='white')
            fig2.savefig(f'{self.outdir}/s{s}_cmp_{tag}_control.png',
                         dpi=200, bbox_inches='tight', facecolor='white')
            if fig3 is not None:
                fig3.savefig(f'{self.outdir}/s{s}_cmp_{tag}_ins.png',
                             dpi=200, bbox_inches='tight', facecolor='white')
            print(f"\n[✓] Comparison plots saved to '{self.outdir}/'")

        if show:
            plt.show()

        return fig1, fig2, fig3


# ── MCPlotter ──────────────────────────────────────────────────────────────────

class MCPlotter:
    """Monte Carlo miss-distance distribution plotter.

    Usage:
        MCPlotter(mc_results, scenario, name, mc_cfg).render(save=True, show=True)

    mc_results : {label: {'miss_dists': list, 'cep': float, 'p90': float, …}}
    mc_cfg     : {'n_runs': int, 'aero_sigma': float, 'ins_noise': bool, …}
    """

    def __init__(self, mc_results, scenario, name, mc_cfg, outdir='plots'):
        self.mc_results = mc_results
        self.scenario   = scenario
        self.name       = name
        self.mc_cfg     = mc_cfg
        self.outdir     = outdir
        self.labels     = list(mc_results)
        self.colors     = {lbl: CVAR[i % len(CVAR)] for i, lbl in enumerate(self.labels)}

    @staticmethod
    def _grid(ax):
        ax.grid(True, which='major', linestyle='-', linewidth=0.8, alpha=0.6)
        ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.4)
        ax.minorticks_on()

    def render(self, save=True, show=True):
        with plt.rc_context(THEME):
            fig1 = self._fig_distributions()
            fig2 = self._fig_stats_table()
        os.makedirs(self.outdir, exist_ok=True)
        if save:
            s = self.scenario
            fig1.savefig(f'{self.outdir}/s{s}_mc_dist.png',
                         dpi=150, bbox_inches='tight', facecolor='white')
            fig2.savefig(f'{self.outdir}/s{s}_mc_stats.png',
                         dpi=150, bbox_inches='tight', facecolor='white')
            print(f"MC plots → {self.outdir}/s{s}_mc_dist.png  +  s{s}_mc_stats.png")
        if show:
            plt.show()
        return fig1, fig2

    # ── figure 1: distribution (histogram + CDF + violin) ────────────────────

    def _fig_distributions(self):
        from scipy.stats import gaussian_kde

        n       = self.mc_cfg['n_runs']
        sigma   = self.mc_cfg['aero_sigma']
        ins_lbl = 'on' if self.mc_cfg['ins_noise'] else 'off'
        s_name  = self.name.split('|')[0].strip()

        fig, axes = plt.subplots(1, 3, figsize=(18, 6))
        fig.suptitle(
            f"Monte Carlo  ·  S{self.scenario}: {s_name}\n"
            f"n={n}  σ_aero={sigma:.0%}  INS={ins_lbl}",
            fontsize=14, fontweight='bold', color='#111111',
        )
        ax_hist, ax_cdf, ax_vln = axes

        all_miss = np.concatenate([d['miss_dists'] for d in self.mc_results.values()])
        x_max    = min(float(np.percentile(all_miss, 99)) * 1.1 + 1.0, 50.0)
        bins     = np.linspace(0.0, x_max, 35)

        # ── histogram + KDE ──────────────────────────────────────────────────
        for lbl, d in self.mc_results.items():
            arr = np.array(d['miss_dists'])
            col = self.colors[lbl]
            ax_hist.hist(arr, bins=bins, alpha=0.35, color=col,
                         density=True, label=lbl)
            x_kde = np.linspace(0.0, x_max, 400)
            try:
                kde = gaussian_kde(arr, bw_method='scott')
                ax_hist.plot(x_kde, kde(x_kde), color=col, lw=2.5)
            except Exception:
                pass
            ax_hist.axvline(d['cep'], color=col, lw=1.8, ls='--', alpha=0.9,
                            label=f"CEP={d['cep']:.1f} m")

        ax_hist.set_xlim(0, x_max)
        ax_hist.set_xlabel('Miss Distance [m]', fontsize=10)
        ax_hist.set_ylabel('Probability Density', fontsize=10)
        ax_hist.set_title('Histogram + KDE', fontsize=11, fontweight='bold')
        ax_hist.legend(fontsize=8, loc='upper right')
        self._grid(ax_hist)

        # ── empirical CDF ─────────────────────────────────────────────────────
        for lbl, d in self.mc_results.items():
            arr = np.sort(d['miss_dists'])
            col = self.colors[lbl]
            cdf = np.arange(1, len(arr) + 1) / len(arr) * 100.0
            ax_cdf.plot(arr, cdf, color=col, lw=2.5, label=lbl)
            ax_cdf.fill_between(arr, cdf, alpha=0.07, color=col)
            for pct, key in ((50, 'cep'), (90, 'p90'), (95, 'p95')):
                ax_cdf.plot(d[key], pct, marker='o', ms=5,
                            color=col, alpha=0.85, zorder=5)

        for pct in (50, 90, 95):
            ax_cdf.axhline(pct, color='#AAAAAA', lw=0.9, ls=':')
            ax_cdf.text(x_max * 0.02, pct + 1.5, f'P{pct}', fontsize=7, color='#777')

        ax_cdf.set_xlim(0, x_max)
        ax_cdf.set_ylim(0, 102)
        ax_cdf.set_xlabel('Miss Distance [m]', fontsize=10)
        ax_cdf.set_ylabel('Cumulative Probability [%]', fontsize=10)
        ax_cdf.set_title('Empirical CDF', fontsize=11, fontweight='bold')
        ax_cdf.legend(fontsize=9, loc='lower right')
        self._grid(ax_cdf)

        # ── violin plot ───────────────────────────────────────────────────────
        data_vln  = [d['miss_dists'] for d in self.mc_results.values()]
        positions = list(range(len(self.labels)))
        vp = ax_vln.violinplot(data_vln, positions=positions,
                               showmedians=True, showextrema=True)
        for i, body in enumerate(vp['bodies']):
            col = self.colors[self.labels[i]]
            body.set_facecolor(col)
            body.set_edgecolor(col)
            body.set_alpha(0.45)
        vp['cmedians'].set_color('#222222')
        vp['cmedians'].set_linewidth(2.0)
        for part in ('cmaxes', 'cmins', 'cbars'):
            vp[part].set_color('#888888')
            vp[part].set_linewidth(1.2)

        for i, (lbl, d) in enumerate(self.mc_results.items()):
            ax_vln.plot(i, d['cep'], marker='D', ms=7, color=self.colors[lbl],
                        zorder=6, label=f"{lbl} CEP={d['cep']:.1f} m")

        ax_vln.set_xticks(positions)
        ax_vln.set_xticklabels(self.labels, fontsize=10)
        ax_vln.set_ylabel('Miss Distance [m]', fontsize=10)
        ax_vln.set_title('Spread (Violin + CEP ◆)', fontsize=11, fontweight='bold')
        ax_vln.legend(fontsize=8, loc='upper right')
        self._grid(ax_vln)

        fig.tight_layout(rect=[0, 0, 1, 0.93])
        return fig

    # ── figure 2: statistics table ────────────────────────────────────────────

    def _fig_stats_table(self):
        n       = self.mc_cfg['n_runs']
        sigma   = self.mc_cfg['aero_sigma']
        ins_lbl = 'on' if self.mc_cfg['ins_noise'] else 'off'

        stat_keys  = ['mean', 'std',  'min',  'max',  'cep',  'p90',  'p95',  'pk5',    'pk10'   ]
        stat_names = ['Mean', 'Std',  'Min',  'Max',  'CEP',  'P90',  'P95',  'Pk<5 m', 'Pk<10 m']
        stat_units = ['[m]',  '[m]',  '[m]',  '[m]',  '[m]',  '[m]',  '[m]',  '[%]',    '[%]'    ]

        col_labels = ['Metric', 'Unit'] + self.labels
        rows = []
        for key, name, unit in zip(stat_keys, stat_names, stat_units):
            row = [name, unit]
            for lbl in self.labels:
                v = self.mc_results[lbl][key]
                row.append(f"{v:.1f}")
            rows.append(row)

        fig, ax = plt.subplots(figsize=(4 + 2.5 * len(self.labels), 5))
        ax.axis('off')
        fig.suptitle(
            f"MC Statistics  ·  S{self.scenario}  n={n}  σ={sigma:.0%}  INS={ins_lbl}",
            fontsize=13, fontweight='bold', color='#111111',
        )

        tbl = ax.table(
            cellText=rows,
            colLabels=col_labels,
            cellLoc='center', loc='center',
        )
        tbl.auto_set_font_size(False)
        tbl.set_fontsize(11)
        tbl.scale(1.1, 2.2)

        n_cols = len(col_labels)
        for j in range(n_cols):
            cell = tbl[0, j]
            cell.set_facecolor('#2C3E50')
            cell.set_text_props(color='white', fontweight='bold')

        for j, lbl in enumerate(self.labels):
            cell = tbl[0, j + 2]
            cell.set_facecolor(self.colors[lbl])
            cell.set_text_props(color='white', fontweight='bold')

        for i in range(1, len(rows) + 1):
            bg = '#F4F6F7' if i % 2 == 0 else 'white'
            for j in range(n_cols):
                tbl[i, j].set_facecolor(bg)

        fig.tight_layout(rect=[0, 0, 1, 0.92])
        return fig
