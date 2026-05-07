"""
F-16 Environment module.

US 1976 Standard Atmosphere + gravity + wind model + Mach/q computation.

Wind modes (mwind):
    0 — no wind
    1 — constant wind: speed dvae, direction psiwdx (deg from North)
    2 — linear shear wind: dvael at waltl → dvaeh at walth (m)
        outside the band the wind is zero

Wind vector is smoothed by a first-order filter (τ = twind s) before use.

Vehicle airspeed:
    VBAL = VBEL - VAEL   (velocity relative to air, local NED frame)
    dvba = |VBAL|
    mach = dvba / vsound
    pdynmc = 0.5 * rho * dvba²
"""

import math
import numpy as np

from missile.environment import atmosphere76   # reuse the shared US76 model

# Physical constants
REARTH     = 6.3781e6     # Earth mean radius - m
G_CONST    = 6.674e-11    # Gravitational constant - m³/(kg·s²)
EARTH_MASS = 5.9722e24    # Earth mass - kg

RAD = math.pi / 180.0


def _integrate_vec(dydx_new, dydx, y, dt):
    """Trapezoidal integration for numpy 3-vectors."""
    return y + (dydx_new + dydx) * 0.5 * dt


class Environment:

    def __init__(self, f16):
        self.f16 = f16

        # wind mode
        self.mwind  = 0       # 0: off | 1: constant | 2: shear
        self.dvae   = 0.0     # constant wind speed - m/s
        self.dvael  = 0.0     # wind speed at low altitude - m/s
        self.waltl  = 0.0     # low altitude bound - m
        self.dvaeh  = 0.0     # wind speed at high altitude - m/s
        self.walth  = 1000.0  # high altitude bound - m
        self.vaed3  = 0.0     # vertical wind component (positive down) - m/s
        self.psiwdx = 0.0     # wind direction from North - deg
        self.twind  = 0.1     # smoothing time constant - s

        # wind filter states
        self.VAELS  = np.zeros(3)   # smoothed wind velocity - m/s
        self.VAELSD = np.zeros(3)   # derivative - m/s²

        # atmosphere outputs
        self.rho    = 1.225    # air density - kg/m³
        self.press  = 101325.0 # pressure - Pa
        self.tempk  = 288.15   # temperature - K
        self.vsound = 340.3    # speed of sound - m/s
        self.grav   = 9.80665  # gravitational acceleration - m/s²

        # flight condition outputs
        self.mach   = 0.0
        self.pdynmc = 0.0
        self.dvba   = 0.0     # airspeed (wrt air) - m/s
        self.VAEL   = np.zeros(3)   # wind velocity in local coords - m/s
        self.VBAL   = np.zeros(3)   # vehicle velocity wrt air - m/s

        # freeze state
        self._mfreeze_prev = 0
        self._machf        = 0.0
        self._pdynmcf      = 0.0

    # ── main environment step ──────────────────────────────────────────────────

    def environment(self, int_step):
        f   = self.f16
        hbe = f.hbe

        # ── gravity ───────────────────────────────────────────────────────────
        rad  = REARTH + hbe
        grav = G_CONST * EARTH_MASS / (rad * rad)

        # ── atmosphere ────────────────────────────────────────────────────────
        rho, press, tempk = atmosphere76(hbe)
        if not math.isfinite(tempk) or tempk <= 0.0:
            tempk = 216.65   # fallback: lower stratosphere temperature
        vsound = math.sqrt(1.4 * 287.058 * tempk)

        # ── wind ──────────────────────────────────────────────────────────────
        VAEL = np.zeros(3)
        if self.mwind > 0:
            if self.mwind == 1:
                dvw = self.dvae
            else:   # mwind == 2: linear shear
                if hbe < self.waltl or hbe > self.walth:
                    dvw = 0.0
                else:
                    dvw = (self.dvael
                           + (self.dvaeh - self.dvael)
                           * (hbe - self.waltl) / (self.walth - self.waltl))

            psi = self.psiwdx * RAD
            VAEL_raw = np.array([
                -dvw * math.cos(psi),
                -dvw * math.sin(psi),
                self.vaed3,
            ])

            # first-order smoothing filter
            VAELSD_new  = (VAEL_raw - self.VAELS) / self.twind
            self.VAELS  = _integrate_vec(VAELSD_new, self.VAELSD, self.VAELS, int_step)
            self.VAELSD = VAELSD_new
            VAEL = self.VAELS.copy()

        # ── flight conditions ─────────────────────────────────────────────────
        VBAL = f.VBEL - VAEL
        dvba = float(np.linalg.norm(VBAL))

        mach   = dvba / vsound if vsound > 0.0 else 0.0
        pdynmc = 0.5 * rho * dvba * dvba

        # ── freeze for autopilot analysis ─────────────────────────────────────
        mfreeze = getattr(f, 'mfreeze', 0)
        if mfreeze == 0:
            self._mfreeze_prev = 0
        else:
            if mfreeze != self._mfreeze_prev:
                self._mfreeze_prev = mfreeze
                self._machf    = mach
                self._pdynmcf  = pdynmc
            mach   = self._machf
            pdynmc = self._pdynmcf

        # ── write to parent ───────────────────────────────────────────────────
        self.rho    = rho
        self.press  = press
        self.tempk  = tempk
        self.vsound = vsound
        self.grav   = grav
        self.mach   = mach
        self.pdynmc = pdynmc
        self.dvba   = dvba
        self.VAEL   = VAEL
        self.VBAL   = VBAL

        f.rho    = rho
        f.press  = press
        f.tempk  = tempk
        f.vsound = vsound
        f.grav   = grav
        f.mach   = mach
        f.pdynmc = pdynmc
        f.dvba   = dvba
        f.dvbe   = float(np.linalg.norm(f.VBEL))   # earth-relative speed
        f.VBAL   = VBAL
