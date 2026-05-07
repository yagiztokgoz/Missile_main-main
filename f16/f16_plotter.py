"""
F-16 Plotter  —  three diagnostic figures in the utils/plotting.py style.

F16Plotter(log, scenario, name).render(save=True, show=True)

log : list[dict] — one entry per logged time step.
      Required keys:
        t, hbe, dvbe, mach, pdynmc,
        alphax, betax, alppx,
        psiblx, thtblx, phiblx, psivlx, thtvlx,
        ppx, qqx, rrx,
        delax, delex, delrx, delacx, delecx, delrcx,
        thrust, throttle, power,
        anx, ayx,
        sbel_n, sbel_e, psivlcomx

Figures produced:
    f16_s{N}_trajectory.png  — 3D track, top/side views, altitude, speed, G-force
    f16_s{N}_states.png      — 4×4 grid of all state variables incl. load factors
    f16_s{N}_actuator.png    — control surface command vs actual + tracking error
"""

import os
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401

# ── colour palette (mirrors utils/plotting.py) ─────────────────────────────────
BLUE   = '#428BCA'
RED    = '#D9534F'
GREEN  = '#5CB85C'
ORANGE = '#F0AD4E'
PURPLE = '#9467BD'
TEAL   = '#17A2B8'
C4     = [BLUE, ORANGE, GREEN, RED]

THEME = {
    'figure.facecolor' : 'white',
    'axes.facecolor'   : '#F7F7F7',
    'savefig.facecolor': 'white',
    'axes.edgecolor'   : '#CCCCCC',
    'axes.labelcolor'  : '#333333',
    'text.color'       : '#111111',
    'xtick.color'      : '#555555',
    'ytick.color'      : '#555555',
    'grid.color'       : '#DDDDDD',
    'grid.alpha'       : 0.9,
    'legend.facecolor' : 'white',
    'legend.edgecolor' : '#BBBBBB',
    'legend.fontsize'  : 9,
}

# F-16 structural limits for G-force reference lines
G_POS_LIM =  5.0    # +ve structural limit - g
G_NEG_LIM = -1.5    # -ve structural limit - g
SURF_LIM  = 25.0    # control surface position limit - deg


class F16Plotter:

    def __init__(self, log, scenario, name, outdir='plots'):
        self.log      = log
        self.scenario = scenario
        self.name     = name
        self.outdir   = outdir
        self._extract()

    # ── data extraction ────────────────────────────────────────────────────────

    def _c(self, key, default=0.0):
        if not self.log or key not in self.log[0]:
            return np.full(len(self.log), default)
        return np.array([r.get(key, default) for r in self.log])

    def _extract(self):
        c = self._c
        self.t      = c('t')
        self.hbe    = c('hbe');    self.dvbe    = c('dvbe')
        self.mach   = c('mach');   self.pdynmc  = c('pdynmc')
        self.alphax = c('alphax'); self.betax   = c('betax');  self.alppx  = c('alppx')
        self.psiblx = c('psiblx'); self.thtblx  = c('thtblx'); self.phiblx = c('phiblx')
        self.psivlx = c('psivlx'); self.thtvlx  = c('thtvlx')
        self.ppx    = c('ppx');    self.qqx     = c('qqx');    self.rrx    = c('rrx')
        self.delax  = c('delax');  self.delex   = c('delex');  self.delrx  = c('delrx')
        self.delacx = c('delacx'); self.delecx  = c('delecx'); self.delrcx = c('delrcx')
        self.thrust    = c('thrust')
        self.throttle  = c('throttle')
        self.power     = c('power')
        self.anx       = c('anx');   self.ayx       = c('ayx')
        self.sbel_n    = c('sbel_n'); self.sbel_e    = c('sbel_e')
        self.psivlcomx = c('psivlcomx')
        # total load factor magnitude (in lateral-normal plane)
        self.G_total = np.sqrt(self.anx**2 + self.ayx**2)

    # ── drawing helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _grid(ax):
        ax.grid(True, which='major', linestyle='-',  linewidth=0.8, alpha=0.6)
        ax.grid(True, which='minor', linestyle=':', linewidth=0.5, alpha=0.4)
        ax.minorticks_on()

    def _glow(self, ax, x, y, color, lw=2.0, **kw):
        """Plot a sharp line with a wide transparent halo (glow effect)."""
        ax.plot(x, y, color=color, lw=lw, **kw)
        ax.plot(x, y, color=color, lw=lw * 4, alpha=0.15)

    def _panel(self, ax, ys_labels, title, ylabel,
               colors=None, zero=False, fill=False):
        colors = colors or C4
        for (y, lbl), col in zip(ys_labels, colors):
            self._glow(ax, self.t, y, col, label=lbl)
            if fill:
                ax.fill_between(self.t, 0, y, color=col, alpha=0.05)
        if zero:
            ax.axhline(0, color='#777', lw=0.8, alpha=0.5)
        ax.set_title(title,   fontsize=12, fontweight='bold', color='#333333')
        ax.set_xlabel('Time [s]', fontsize=10)
        ax.set_ylabel(ylabel,  fontsize=10)
        ax.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax)
        ax.tick_params(labelsize=9)

    def _suptitle(self, fig, extra=''):
        s     = self.scenario
        title = (f"F-16  6-DoF  ·  Scenario {s}  —  "
                 f"{self.name.split('|')[0].strip()}")
        if extra:
            title += f'\n{extra}'
        fig.suptitle(title, fontsize=14, fontweight='bold',
                     y=0.97, color='#111111')

    # ── Figure 1: Trajectory ───────────────────────────────────────────────────

    def _fig_trajectory(self):
        s    = self.scenario
        n, e = self.sbel_n, self.sbel_e
        h    = self.hbe

        fig = plt.figure(f'F16_S{s}_Trajectory', figsize=(20, 9))
        self._suptitle(fig)

        # ── 3D track ──────────────────────────────────────────────────────────
        ax3 = fig.add_subplot(231, projection='3d')
        ax3.set_facecolor('white')
        for axis in (ax3.xaxis, ax3.yaxis, ax3.zaxis):
            axis.set_pane_color((0.97, 0.97, 0.97, 1.0))
            axis.line.set_color('#AAAAAA')

        ax3.plot(n, e, h, color=BLUE, lw=2.5, label='F-16 Track')
        ax3.plot(n, e, h, color=BLUE, lw=7.0, alpha=0.15)
        h0 = h.min() - 200
        ax3.plot(n, e, np.full_like(h, h0), color=BLUE, lw=1.5, alpha=0.4, ls=':')
        ax3.scatter(n[0],  e[0],  h[0],  c=BLUE, s=60,  zorder=5, label='Start')
        ax3.scatter(n[-1], e[-1], h[-1], c=RED,  s=150, marker='*', zorder=6, label='End')
        ax3.set_xlabel('North [km]', labelpad=10)
        ax3.set_ylabel('East [km]',  labelpad=10)
        ax3.set_zlabel('Alt [m]',    labelpad=10)
        ax3.set_title('3D Track', fontsize=12, fontweight='bold', color='#333333')
        ax3.legend(loc='best', framealpha=0.9)
        ax3.view_init(elev=25, azim=-55)

        # Equal-scale axes: matplotlib 3D does not support set_aspect('equal')
        # so we compute a common half-range and apply it to all three axes.
        mid_n = (n.max() + n.min()) * 0.5
        mid_e = (e.max() + e.min()) * 0.5
        mid_h = (h.max() + h.min()) * 0.5
        half  = max(n.max()-n.min(), e.max()-e.min(), h.max()-h.min()) * 0.5
        half  = max(half, 500.0)   # minimum ±500 m view extent
        ax3.set_xlim(mid_n - half, mid_n + half)
        ax3.set_ylim(mid_e - half, mid_e + half)
        ax3.set_zlim(mid_h - half, mid_h + half)
        ax3.xaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/1e3:.1f}'))
        ax3.yaxis.set_major_formatter(plt.FuncFormatter(lambda x, _: f'{x/1e3:.1f}'))

        # ── top view ──────────────────────────────────────────────────────────
        ax_top = fig.add_subplot(232)
        self._glow(ax_top, n/1e3, e/1e3, BLUE, label='Track')
        ax_top.scatter(n[0]/1e3,  e[0]/1e3,  c=BLUE, s=60,  zorder=5, label='Start')
        ax_top.scatter(n[-1]/1e3, e[-1]/1e3, c=RED,  s=150, marker='*', zorder=6, label='End')
        ax_top.set_xlabel('North [km]'); ax_top.set_ylabel('East [km]')
        ax_top.set_title('Top View (N-E Plane)',
                         fontsize=12, fontweight='bold', color='#333333')
        ax_top.set_aspect('equal', adjustable='datalim')
        ax_top.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax_top)

        # ── side view ─────────────────────────────────────────────────────────
        ax_side = fig.add_subplot(233)
        self._glow(ax_side, n/1e3, h, BLUE, label='F-16')
        ax_side.fill_between(n/1e3, h.min()-50, h, color=BLUE, alpha=0.05)
        ax_side.scatter(n[0]/1e3,  h[0],  c=BLUE, s=60,  zorder=5, label='Start')
        ax_side.scatter(n[-1]/1e3, h[-1], c=RED,  s=150, marker='*', zorder=6, label='End')
        ax_side.set_xlabel('North [km]'); ax_side.set_ylabel('Altitude [m]')
        ax_side.set_title('Side View (N-Alt Plane)',
                          fontsize=12, fontweight='bold', color='#333333')
        ax_side.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax_side)

        # ── altitude vs time ──────────────────────────────────────────────────
        ax_h = fig.add_subplot(234)
        self._glow(ax_h, self.t, self.hbe, BLUE, lw=2.0, label='Altitude')
        ax_h.set_xlabel('Time [s]'); ax_h.set_ylabel('Altitude [m]')
        ax_h.set_title('Altitude vs Time', fontsize=12, fontweight='bold', color='#333333')
        ax_h.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax_h)

        # ── speed and Mach vs time ─────────────────────────────────────────────
        ax_v = fig.add_subplot(235)
        ax_v2 = ax_v.twinx()
        self._glow(ax_v,  self.t, self.dvbe, BLUE,   lw=2.0, label='TAS [m/s]')
        self._glow(ax_v2, self.t, self.mach, ORANGE, lw=1.8, label='Mach')
        ax_v.set_xlabel('Time [s]')
        ax_v.set_ylabel('TAS [m/s]',  color=BLUE)
        ax_v2.set_ylabel('Mach',      color=ORANGE)
        ax_v.set_title('Speed / Mach vs Time',
                        fontsize=12, fontweight='bold', color='#333333')
        h1, l1 = ax_v.get_legend_handles_labels()
        h2, l2 = ax_v2.get_legend_handles_labels()
        ax_v.legend(h1+h2, l1+l2, fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax_v)

        # ── G-force vs time ───────────────────────────────────────────────────
        ax_g = fig.add_subplot(236)
        self._glow(ax_g, self.t, self.anx,     BLUE,   lw=2.0, label='nz  normal')
        self._glow(ax_g, self.t, self.ayx,     GREEN,  lw=1.6, label='ny  lateral')
        self._glow(ax_g, self.t, self.G_total, ORANGE, lw=1.8, label='|G| total')
        ax_g.axhline(G_POS_LIM, color=RED, ls='--', lw=1.4, alpha=0.7,
                     label=f'+{G_POS_LIM:.0f} g  struct. limit')
        ax_g.axhline(G_NEG_LIM, color=RED, ls='--', lw=1.4, alpha=0.7,
                     label=f'{G_NEG_LIM:.1f} g  struct. limit')
        ax_g.axhline(1.0, color='#888', ls=':', lw=1.0, alpha=0.5)
        ax_g.axhline(0.0, color='#888', lw=0.8, alpha=0.4)
        ax_g.set_xlabel('Time [s]'); ax_g.set_ylabel('Load Factor [g]')
        ax_g.set_title('G-Force vs Time',
                        fontsize=12, fontweight='bold', color='#333333')
        ax_g.legend(fontsize=7, loc='best', framealpha=0.9, ncol=2)
        self._grid(ax_g)

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        return fig

    # ── Figure 2: State variables ──────────────────────────────────────────────

    def _fig_states(self):
        s = self.scenario
        fig, axes = plt.subplots(4, 4, figsize=(24, 14),
                                 num=f'F16_S{s}_States')
        self._suptitle(fig, extra='State Variables')

        def P(r, c, ys_lbls, title, ylabel, colors=None, zero=False, fill=True):
            self._panel(axes[r][c], ys_lbls, title, ylabel, colors, zero, fill)

        # Row 0: flight kinematics
        P(0, 0, [(self.dvbe,   'TAS [m/s]')],          'Speed',            'm/s',  [BLUE],   fill=True)
        P(0, 1, [(self.mach,   'Mach')],                'Mach Number',      '–',    [ORANGE], fill=True)
        P(0, 2, [(self.pdynmc, 'q [kPa]')],             'Dynamic Pressure', 'kPa',  [GREEN],  fill=True)
        P(0, 3, [(self.hbe,    'Alt [m]')],             'Altitude',         'm',    [BLUE],   fill=True)

        # Row 1: incidence and attitude
        P(1, 0, [(self.alphax, 'α AoA'), (self.betax, 'β sideslip')],
              'Incidence Angles', 'deg', [BLUE, RED], zero=True)
        P(1, 1, [(self.alppx,  'αₜ total AoA')],       'Total AoA',        'deg',  [PURPLE], zero=True)
        P(1, 2, [(self.psiblx, 'ψ yaw'),
                 (self.thtblx, 'θ pitch'),
                 (self.phiblx, 'φ roll')],
              'Euler Angles', 'deg', C4[:3], zero=True)
        P(1, 3, [(self.psivlx, 'ψᵥ heading'),
                 (self.thtvlx, 'γ FPA')],
              'Flight Path Angles', 'deg', [BLUE, GREEN], zero=True)

        # Row 2: angular rates and control surfaces
        P(2, 0, [(self.ppx, 'p roll'),
                 (self.qqx, 'q pitch'),
                 (self.rrx, 'r yaw')],
              'Body Rates', 'deg/s', C4[:3], zero=True)
        P(2, 1, [(self.delacx, 'δa cmd'),
                 (self.delecx, 'δe cmd'),
                 (self.delrcx, 'δr cmd')],
              'Surface Commands', 'deg', C4[:3], zero=True)
        P(2, 2, [(self.delax, 'δa act'),
                 (self.delex, 'δe act'),
                 (self.delrx, 'δr act')],
              'Surface Deflections', 'deg', C4[:3], zero=True)

        # G-force panel with structural limit lines
        ax_g = axes[2][3]
        self._glow(ax_g, self.t, self.anx,     BLUE,   lw=2.0, label='nz  normal')
        self._glow(ax_g, self.t, self.ayx,     GREEN,  lw=1.6, label='ny  lateral')
        self._glow(ax_g, self.t, self.G_total, ORANGE, lw=1.8, label='|G| total')
        ax_g.axhline(G_POS_LIM, color=RED, ls='--', lw=1.5, alpha=0.7,
                     label=f'+{G_POS_LIM:.0f} g limit')
        ax_g.axhline(G_NEG_LIM, color=RED, ls='--', lw=1.5, alpha=0.7,
                     label=f'{G_NEG_LIM:.1f} g limit')
        ax_g.axhline(1.0, color='#888', ls=':', lw=1.0, alpha=0.5)
        ax_g.axhline(0.0, color='#888', lw=0.8, alpha=0.4)
        ax_g.set_title('Load Factors (G-Force)',
                        fontsize=12, fontweight='bold', color='#333333')
        ax_g.set_xlabel('Time [s]', fontsize=10)
        ax_g.set_ylabel('Load Factor [g]', fontsize=10)
        ax_g.legend(fontsize=7, loc='best', framealpha=0.9, ncol=2)
        self._grid(ax_g)
        ax_g.tick_params(labelsize=9)

        # Row 3: propulsion
        P(3, 0, [(self.thrust/1e3, 'Thrust [kN]')], 'Engine Thrust', 'kN',  [RED],    fill=True)
        P(3, 1, [(self.throttle,   'Throttle [%]')], 'Throttle',     '%',   [ORANGE], fill=True)
        P(3, 2, [(self.power,      'Power [%]')],    'Power Setting', '%',   [GREEN],  fill=True)

        # Heading: actual vs commanded
        ax = axes[3][3]
        self._glow(ax, self.t, self.psivlx,    BLUE, lw=2.0, label='ψ actual')
        self._glow(ax, self.t, self.psivlcomx, RED,  lw=1.6, label='ψ command')
        ax.plot(self.t, self.psivlcomx, '--', color=RED, lw=1.2, alpha=0.5)
        ax.axhline(0, color='#777', lw=0.8, alpha=0.4)
        ax.set_title('Heading: Actual vs Command',
                      fontsize=12, fontweight='bold', color='#333333')
        ax.set_xlabel('Time [s]', fontsize=10)
        ax.set_ylabel('Heading [deg]', fontsize=10)
        ax.legend(fontsize=8, loc='best', framealpha=0.9)
        self._grid(ax)
        ax.tick_params(labelsize=9)

        fig.tight_layout(rect=[0, 0, 1, 0.96])
        return fig

    # ── Figure 3: Control surfaces ─────────────────────────────────────────────

    def _fig_actuator(self):
        s = self.scenario
        fig, axf = plt.subplots(3, 2, figsize=(14, 12),
                                num=f'F16_S{s}_Actuator', sharex=True)
        self._suptitle(fig, extra='Control Surfaces  —  Command vs Actual')

        t     = self.t
        surfs = [
            ('Aileron',  self.delacx, self.delax,  BLUE),
            ('Elevator', self.delecx, self.delex,  ORANGE),
            ('Rudder',   self.delrcx, self.delrx,  GREEN),
        ]

        for i, (surf_name, cmd, act, col) in enumerate(surfs):
            # Left column: command vs actual overlay
            axl = axf[i][0]
            axl.plot(t, cmd, color=ORANGE, lw=1.8, ls='--', label=f'{surf_name} Command')
            axl.plot(t, act, color=col,    lw=2.0,           label=f'{surf_name} Actual')
            axl.plot(t, act, color=col,    lw=6.0, alpha=0.15)
            axl.fill_between(t, 0, act, color=col, alpha=0.05)
            axl.axhline( SURF_LIM, color=RED, ls=':', lw=1.5, alpha=0.6)
            axl.axhline(-SURF_LIM, color=RED, ls=':', lw=1.5, alpha=0.6)
            axl.set_title(f'{surf_name}  —  Command vs Actual',
                          fontsize=12, fontweight='bold', color='#333333')
            axl.set_ylabel('Deflection [deg]', fontsize=10)
            axl.legend(fontsize=9, loc='best', framealpha=0.9)
            self._grid(axl)
            axl.tick_params(labelsize=9)

            # Right column: tracking error
            axr = axf[i][1]
            err = cmd - act
            axr.plot(t, err, color=RED, lw=2.0, label='Tracking Error')
            axr.plot(t, err, color=RED, lw=6.0, alpha=0.15)
            axr.fill_between(t, 0, err, color=RED, alpha=0.05)
            axr.axhline(0, color='#555', lw=1.0, alpha=0.5)
            axr.set_title(f'{surf_name}  —  Tracking Error',
                          fontsize=12, fontweight='bold', color='#333333')
            axr.set_ylabel('Error [deg]', fontsize=10)
            axr.legend(fontsize=9, loc='best', framealpha=0.9)
            self._grid(axr)
            axr.tick_params(labelsize=9)

        axf[-1][0].set_xlabel('Time [s]', fontsize=10)
        axf[-1][1].set_xlabel('Time [s]', fontsize=10)
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        return fig

    # ── public API ─────────────────────────────────────────────────────────────

    def render(self, save=True, show=True):
        plt.rcParams.update(THEME)

        fig1 = self._fig_trajectory()
        fig2 = self._fig_states()
        fig3 = self._fig_actuator()

        if save:
            os.makedirs(self.outdir, exist_ok=True)
            s = self.scenario
            fig1.savefig(f'{self.outdir}/f16_s{s}_trajectory.png',
                         dpi=200, bbox_inches='tight', facecolor='white')
            fig2.savefig(f'{self.outdir}/f16_s{s}_states.png',
                         dpi=200, bbox_inches='tight', facecolor='white')
            fig3.savefig(f'{self.outdir}/f16_s{s}_actuator.png',
                         dpi=200, bbox_inches='tight', facecolor='white')
            print(f"\n[✓] F-16 S{s} plots saved to '{self.outdir}/'")

        if show:
            plt.show()

        return fig1, fig2, fig3
