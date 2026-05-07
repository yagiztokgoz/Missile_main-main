"""
F-16 Guidance module — waypoint / IP line guidance.

mguid encoding  |mguidl|mguidp|  (two-digit integer):
    mguidl = mguid // 10   lateral guidance
    mguidp = mguid  % 10   pitch guidance

    mguidl = 0  no lateral guidance
             3  lateral line guidance toward waypoint
    mguidp = 0  no pitch guidance
             3  pitch line guidance toward IP

Guidance law (guidance_line):
    Decomposes vehicle velocity into LOS and LOA frames, then
    commands acceleration to fly a straight-line intercept of the
    designated waypoint / IP.

    ALGV[1] → alcomx = ALGV[1] / g   (lateral, g's)
    ALGV[2] → ancomx = −ALGV[2] / g  (normal,  g's)

Outputs:
    f16.alcomx  — lateral acceleration command  (g's)
    f16.ancomx  — normal  acceleration command  (g's)
"""

import math
import numpy as np

RAD = math.pi / 180.0
DEG = 180.0 / math.pi
EPS = 1e-10


def _mat2tr(psi, gamma):
    """Local-level → (heading psi, elevation gamma) frame DCM."""
    cp = math.cos(psi);  sp = math.sin(psi)
    cg = math.cos(gamma); sg = math.sin(gamma)
    return np.array([
        [ cg*cp,  cg*sp, -sg],
        [   -sp,     cp,  0.],
        [ sg*cp,  sg*sp,  cg],
    ])


def _pol_from_cart(v):
    """Cartesian → [range, azimuth, elevation] (CADAC convention, NED frame).

    azimuth  = atan2(East, North)  — bearing from North, positive clockwise
    elevation = asin(−Down/range)  — positive upward
    """
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    r = math.sqrt(x*x + y*y + z*z)
    if r < EPS:
        return np.array([0.0, 0.0, 0.0])
    az = math.atan2(y, x)
    el = math.asin(max(-1.0, min(1.0, -z / r)))
    return np.array([r, az, el])


class Guidance:

    def __init__(self, f16):
        self.f16 = f16

        self.mguid       = 0      # guidance mode flag (|mguidl|mguidp|)
        self.line_gain   = 0.0    # line guidance gain - 1/s
        self.nl_gain_fact= 0.0    # nonlinear gain factor - ND
        self.decrement   = 1.0    # distance decrement - m  (avoid /0)
        self.philimx     = 70.0   # bank angle limit for turn_min calc - deg

        # waypoint / IP in local NED (m)
        self.swel1 = 0.0
        self.swel2 = 0.0
        self.swel3 = 0.0

        # line-of-attack angles
        self.psiflx = 0.0   # heading LOA - deg
        self.thtflx = 0.0   # elevation LOA - deg

        # diagnostics
        self.dwb      = 0.0
        self.dwbh     = 0.0
        self.nl_gain  = 0.0
        self.wp_flag  = 1
        self.turn_min = 0.0
        self.SWBL     = np.zeros(3)
        self.VBEO     = np.zeros(3)
        self.VBEF     = np.zeros(3)

    # ── main guidance step ─────────────────────────────────────────────────────

    def guidance(self):
        f = self.f16

        if self.mguid == 0:
            return

        mguidl = self.mguid // 10
        mguidp = self.mguid  % 10

        SBEL = f.SBEL
        VBEL = f.VBEL
        dvbe = f.dvbe
        grav = f.grav

        # ── waypoint geometry ─────────────────────────────────────────────────
        SWEL = np.array([self.swel1, self.swel2, self.swel3])
        SWBL = SWEL - SBEL
        dwb  = float(np.linalg.norm(SWBL))
        dwbh = math.sqrt(SWBL[0]**2 + SWBL[1]**2)

        # ── waypoint flag (closing vs fleeting) ───────────────────────────────
        wp_flag  = 1
        turn_min = dvbe**2 / max(grav * math.tan(self.philimx * RAD), EPS)

        if dwbh < 2.0 * turn_min:
            closing = SWBL[0]*VBEL[0] + SWBL[1]*VBEL[1]
            wp_flag = 1 if closing >= 0.0 else -1

        # ── line guidance ─────────────────────────────────────────────────────
        ALGV = self._guidance_line(SWBL)

        alcomx = 0.0
        ancomx = 0.0
        if mguidl == 3:
            alcomx =  float(ALGV[1]) / grav
        if mguidp == 3:
            ancomx = -float(ALGV[2]) / grav

        # store diagnostics
        self.dwb      = dwb
        self.dwbh     = dwbh
        self.SWBL     = SWBL
        self.wp_flag  = wp_flag
        self.turn_min = turn_min

        f.alcomx = alcomx
        f.ancomx = ancomx

    # ── line guidance sub-function ─────────────────────────────────────────────

    def _guidance_line(self, SWBL):
        """
        Compute acceleration vector in flight-path frame for line guidance.

        Returns ALGV (3,) where:
            ALGV[1] → lateral accel  (g's via / grav)
            ALGV[2] → normal accel   (g's via −/grav)
        """
        f      = self.f16
        VBEL   = f.VBEL
        grav   = f.grav
        thtvlx = f.thtvlx

        psiflx = self.psiflx
        thtflx = self.thtflx

        # LOA (Line-Of-Attack) transformation: local → LOA frame
        TFL = _mat2tr(psiflx * RAD, thtflx * RAD)

        # LOS (Line-Of-Sight) transformation: local → LOS frame
        polar       = _pol_from_cart(SWBL)
        wp_sltrange = polar[0]
        psiol       = polar[1]
        thtol       = polar[2]
        TOL = _mat2tr(psiol, thtol)

        # velocity components in LOS and LOA frames
        VBEO = TOL @ VBEL
        VBEF = TFL @ VBEL

        # nonlinear gain (ramps up as vehicle approaches waypoint)
        nl_gain = (self.nl_gain_fact
                   * (1.0 - math.exp(-wp_sltrange / max(self.decrement, EPS))))

        # line guidance law
        algv1 = grav * math.sin(thtvlx * RAD)
        algv2 = self.line_gain * (-VBEO[1] + nl_gain * VBEF[1])
        algv3 = (self.line_gain * (-VBEO[2] + nl_gain * VBEF[2])
                 - grav * math.cos(thtvlx * RAD))

        # store diagnostics
        self.nl_gain = nl_gain
        self.VBEO    = VBEO
        self.VBEF    = VBEF

        return np.array([algv1, algv2, algv3])
