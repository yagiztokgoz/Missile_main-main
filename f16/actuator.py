"""
F-16 Actuator module — aileron / elevator / rudder dynamics.

State vector (3-element, one per surface):
    DX  [i] = achieved fin position - deg
    DDX [i] = achieved fin rate     - deg/s

Second-order dynamics (mact=2):
    DDX_dot = ωn² · (cmd − DX) − 2·ζ·ωn · DDX
    DX_dot  = DDX

Limiting:
    Position: |DX[i]| ≤ dlimx;  if saturated AND rate drives further in → clamp rate to 0
    Rate    : |DDX[i]| ≤ ddlimx; if saturated AND rate-dot drives further in → clamp rate-dot to 0

mact modes:
    0 — no dynamics, position limit only
    2 — full second-order with position + rate limiting (default)
"""

import math
import numpy as np

EPS = 1e-10


def _sign(x):
    return math.copysign(1.0, x) if x != 0.0 else 1.0


def _integrate(dydx_new, dydx, y, dt):
    return y + (dydx_new + dydx) * 0.5 * dt


class Actuator:

    def __init__(self, f16):
        self.f16 = f16

        self.mact   = 2       # 0: no dynamics | 2: second order
        self.dlimx  = 25.0    # position limit - deg
        self.ddlimx = 60.0    # rate limit     - deg/s
        self.wnact  = 20.2    # natural frequency - rad/s
        self.zetact = 0.7     # damping ratio

        # state (aileron=0, elevator=1, rudder=2)
        self.DX   = np.zeros(3)   # fin position - deg
        self.DXD  = np.zeros(3)   # derivative   - deg/s
        self.DDX  = np.zeros(3)   # fin rate     - deg/s
        self.DDXD = np.zeros(3)   # derivative   - deg/s²

        # outputs
        self.delax = 0.0   # aileron  deflection - deg
        self.delex = 0.0   # elevator deflection - deg
        self.delrx = 0.0   # rudder   deflection - deg

    def actuator(self, int_step):
        f = self.f16
        ACTCX = np.array([f.delacx, f.delecx, f.delrcx])

        if self.mact == 0:
            ACTX = np.clip(ACTCX, -self.dlimx, self.dlimx)
        else:
            ACTX = self._second_order(ACTCX, int_step)

        self.delax = float(ACTX[0])
        self.delex = float(ACTX[1])
        self.delrx = float(ACTX[2])

        f.delax = self.delax
        f.delex = self.delex
        f.delrx = self.delrx

    def _second_order(self, ACTCX, dt):
        dlimx  = self.dlimx
        ddlimx = self.ddlimx
        wn     = self.wnact
        ze     = self.zetact

        for i in range(3):
            # ── position limiting ────────────────────────────────────────────
            if abs(self.DX[i]) > dlimx:
                self.DX[i] = dlimx * _sign(self.DX[i])
                if self.DX[i] * self.DDX[i] > 0.0:
                    self.DDX[i] = 0.0

            # ── rate limiting ────────────────────────────────────────────────
            iflag = 0
            if abs(self.DDX[i]) > ddlimx:
                iflag = 1
                self.DDX[i] = ddlimx * _sign(self.DDX[i])

            # ── position state integration  (DX_dot = DDX) ──────────────────
            DXD_new       = self.DDX[i]
            self.DX[i]    = _integrate(DXD_new, self.DXD[i], self.DX[i], dt)
            self.DXD[i]   = DXD_new

            # ── rate state integration  (DDX_dot = ωn²·e − 2·ζ·ωn·DDX) ────
            edx           = ACTCX[i] - self.DX[i]
            DDXD_new      = wn*wn * edx - 2.0*ze*wn * self.DDX[i]
            self.DDX[i]   = _integrate(DDXD_new, self.DDXD[i], self.DDX[i], dt)
            self.DDXD[i]  = DDXD_new

            # clamp rate-dot when rate is saturated and derivative pushes same way
            if iflag and self.DDX[i] * self.DDXD[i] > 0.0:
                self.DDXD[i] = 0.0

        return self.DX.copy()
