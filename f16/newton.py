"""
F-16 Newton module — translational equations of motion.

Body-frame velocity ODE:
    dVBEB/dt = FSPB − ω×VBEB + TBL·[0, 0, g]

where
    FSPB  = FAPB / mass          (specific force, body axes)
    ω×VBEB                       (centripetal / Coriolis term, body axes)
    TBL·[0,0,g]                  (gravity rotated into body frame)

Position is integrated in local NED:
    dSBEL/dt = VBEL = TLB · VBEB

Altitude (positive up):
    hbe = −SBEL[2]   (NED: down is positive, so hbe = −z_NED)
"""

import math
import numpy as np

RAD = math.pi / 180.0
DEG = 180.0 / math.pi
EPS = 1e-10


def _integrate_vec(dydx_new, dydx, y, dt):
    return y + (dydx_new + dydx) * 0.5 * dt


def _mat2tr(psi, gamma):
    """Velocity-wrt-local DCM (heading ψ, flight-path angle γ)."""
    cp = math.cos(psi);  sp = math.sin(psi)
    cg = math.cos(gamma); sg = math.sin(gamma)
    return np.array([
        [ cg*cp,  cg*sp, -sg],
        [   -sp,     cp,  0.],
        [ sg*cp,  sg*sp,  cg],
    ])


class Newton:

    def __init__(self, f16):
        self.f16 = f16

        # state
        self.VBEB  = np.zeros(3)    # velocity in body axes - m/s
        self.VBEBD = np.zeros(3)    # derivative - m/s²
        self.SBEL  = np.zeros(3)    # position in local NED - m
        self.SBELD = np.zeros(3)    # derivative - m/s

        # outputs
        self.VBEL     = np.zeros(3)
        self.FSPB     = np.zeros(3)
        self.dvbe     = 0.0          # speed wrt Earth - m/s
        self.hbe      = 0.0          # altitude - m
        self.psivlx   = 0.0          # heading - deg
        self.thtvlx   = 0.0          # flight-path angle - deg
        self.anx      = 0.0          # normal specific force - g's
        self.ayx      = 0.0          # lateral specific force - g's
        self.alx      = 0.0          # horizontal specific force - g's

        # freeze state
        self._mfreeze_prev = 0
        self._dvbef        = 0.0
        self._SBELM        = np.zeros(3)
        self.groundrange   = 0.0

    # ── initialisation ─────────────────────────────────────────────────────────

    def init(self):
        f = self.f16
        dvbe   = f.dvbe
        alpha0 = f.alpha0x * RAD
        beta0  = f.beta0x  * RAD
        TBL    = f.TBL

        # initial velocity in body axes
        ca, sa = math.cos(alpha0), math.sin(alpha0)
        cb, sb = math.cos(beta0),  math.sin(beta0)
        self.VBEB = np.array([ca*cb*dvbe, sb*dvbe, sa*cb*dvbe])

        # initial velocity in local NED
        VBEL = TBL.T @ self.VBEB

        # initial position
        self.SBEL  = np.array(f.SBEL_INIT, dtype=float)
        self._SBELM = self.SBEL.copy()

        # altitude
        self.hbe = -self.SBEL[2]

        # flight-path angles
        self.VBEL = VBEL
        self._update_path_angles(VBEL)

        # write to parent
        f.VBEB    = self.VBEB.copy()
        f.VBEL    = VBEL.copy()
        f.SBEL    = self.SBEL.copy()
        f.hbe     = self.hbe
        f.dvbe    = float(np.linalg.norm(VBEL))
        f.psivlx  = self.psivlx
        f.thtvlx  = self.thtvlx

    # ── main newton step ───────────────────────────────────────────────────────

    def newton(self, int_step):
        f    = self.f16
        grav = f.grav
        TBL  = f.TBL
        WBEB = f.WBEB
        FAPB = f.FAPB
        mass = f.mass

        # ── tangential acceleration (centripetal term) ─────────────────────────
        ATB = np.cross(WBEB, self.VBEB)

        # ── specific force in body axes ────────────────────────────────────────
        FSPB = FAPB / mass

        # ── gravity rotated into body frame ────────────────────────────────────
        GRAVL = np.array([0.0, 0.0, grav])   # NED: positive down

        # ── body-frame velocity ODE ────────────────────────────────────────────
        VBEBD_NEW = FSPB - ATB + TBL @ GRAVL
        self.VBEB  = _integrate_vec(VBEBD_NEW, self.VBEBD, self.VBEB, int_step)
        self.VBEBD = VBEBD_NEW

        # ── local-frame velocity and position ─────────────────────────────────
        VBEL      = TBL.T @ self.VBEB
        SBELD_NEW = VBEL
        self.SBEL  = _integrate_vec(SBELD_NEW, self.SBELD, self.SBEL, int_step)
        self.SBELD = SBELD_NEW

        # ── derived quantities ────────────────────────────────────────────────
        dvbe = float(np.linalg.norm(VBEL))
        hbe  = -self.SBEL[2]
        self._update_path_angles(VBEL)

        # ── specific force in g's (normal, lateral, horizontal) ───────────────
        anx = -FSPB[2] / grav
        ayx =  FSPB[1] / grav

        psi_r = self.psivlx * RAD
        gam_r = self.thtvlx * RAD
        TVL  = _mat2tr(psi_r, gam_r)
        FSPV = TVL @ (TBL.T @ FSPB)
        alx  = FSPV[1] / grav

        # ── freeze for autopilot analysis ─────────────────────────────────────
        mfreeze = getattr(f, 'mfreeze', 0)
        if mfreeze == 0:
            self._mfreeze_prev = 0
        else:
            if mfreeze != self._mfreeze_prev:
                self._mfreeze_prev = mfreeze
                self._dvbef = dvbe
            dvbe = self._dvbef

        # ── ground range (horizontal distance travelled) ───────────────────────
        delta = self.SBEL - self._SBELM
        delta[2] = 0.0
        self.groundrange += float(np.linalg.norm(delta))
        self._SBELM = self.SBEL.copy()

        # ── store ─────────────────────────────────────────────────────────────
        self.VBEL   = VBEL
        self.FSPB   = FSPB
        self.dvbe   = dvbe
        self.hbe    = hbe
        self.anx    = anx
        self.ayx    = ayx
        self.alx    = alx

        f.VBEB    = self.VBEB.copy()
        f.VBEL    = VBEL.copy()
        f.SBEL    = self.SBEL.copy()
        f.FSPB    = FSPB.copy()
        f.dvbe    = dvbe
        f.hbe     = hbe
        f.psivlx  = self.psivlx
        f.thtvlx  = self.thtvlx
        f.anx     = anx
        f.ayx     = ayx

    def _update_path_angles(self, VBEL):
        v1, v2, v3 = float(VBEL[0]), float(VBEL[1]), float(VBEL[2])
        psivl = math.atan2(v2, v1) if (v1 != 0.0 or v2 != 0.0) else 0.0
        thtvl = math.atan2(-v3, math.sqrt(v1*v1 + v2*v2))
        self.psivlx = psivl * DEG
        self.thtvlx = thtvl * DEG
