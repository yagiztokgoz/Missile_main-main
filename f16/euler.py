"""
F-16 Euler module — rigid-body rotational equations of motion.

Integrates the angular velocity WBEB (body axes, wrt inertial frame).

Euler equation with engine gyroscopic coupling:
    I · ω̇ = M_body − ω × (I·ω + h_engine)

where
    I          = IBBB  (3×3 inertia matrix, kg·m²)
    ω  = WBEB  (body angular velocity, rad/s)
    M_body     = FMB   (total aerodynamic + other moments, N·m)
    h_engine   = [eng_ang_mom, 0, 0]  (engine spin axis = body x, kg·m²/s)

Inertia (from NASA TP-1538 / Stevens & Lewis):
    Ixx = 12875  kg·m²
    Iyy = 75673  kg·m²
    Izz = 85551  kg·m²
    Ixz = Izx = -1331.4 kg·m²   (Ixy = Iyz = 0)

Engine angular momentum: eng_ang_mom = 70 000 kg·m²/s (F100 rotor)
"""

import math
import numpy as np

RAD = math.pi / 180.0
DEG = 180.0 / math.pi


def _integrate_vec(dydx_new, dydx, y, dt):
    return y + (dydx_new + dydx) * 0.5 * dt


class Euler:

    def __init__(self, f16):
        self.f16 = f16

        # angular velocity state (rad/s)
        self.WBEB  = np.zeros(3)
        self.WBEBD = np.zeros(3)

        # outputs (deg/s)
        self.ppx = 0.0
        self.qqx = 0.0
        self.rrx = 0.0

    def init(self):
        """Initialise WBEB from initial body-rate conditions on f16."""
        f = self.f16
        self.WBEB = np.array([f.ppx * RAD, f.qqx * RAD, f.rrx * RAD])

    def euler(self, int_step):
        f = self.f16

        FMB     = f.FMB             # total moments in body axes - N·m  (3,)
        I       = f.IBBB            # inertia matrix - kg·m²  (3,3)
        h_eng   = f.eng_ang_mom     # engine angular momentum scalar - kg·m²/s

        # engine angular momentum vector along body x-axis
        L_ENGINE = np.array([h_eng, 0.0, 0.0])

        # Euler equation: ω̇ = I⁻¹ · (M − ω × (I·ω + h))
        gyro = np.cross(self.WBEB, I @ self.WBEB + L_ENGINE)
        WACC = np.linalg.solve(I, FMB - gyro)

        # trapezoidal integration
        self.WBEB  = _integrate_vec(WACC, self.WBEBD, self.WBEB, int_step)
        self.WBEBD = WACC

        ppx = self.WBEB[0] * DEG
        qqx = self.WBEB[1] * DEG
        rrx = self.WBEB[2] * DEG

        self.ppx = ppx
        self.qqx = qqx
        self.rrx = rrx

        f.WBEB = self.WBEB.copy()
        f.ppx  = ppx
        f.qqx  = qqx
        f.rrx  = rrx
