import math
import numpy as np


def _integrate_vec(dydx_new, dydx, y, int_step):
    """Trapezoidal integration for 3-element numpy vectors."""
    return y + (dydx_new + dydx) / 2.0 * int_step


class Newton(object):

    def __init__(self, missile):
        self.missile = missile

        # state variables and their derivatives
        m = missile

        # VBEB is already initialised in Missile.__init__ from alpha0x/beta0x/dvbe
        self.VBEB  = m.VBEB.copy()
        self.VBEBD = np.zeros(3)   # derivative of VBEB - m/s^2

        # position in local level (NED) axes
        self.SBEL  = np.array([m.sbel1, m.sbel2, m.sbel3], dtype=float)
        self.SBELD = np.zeros(3)   # derivative of SBEL - m/s

        # launch point (fixed at init, never changes)
        self.SLEL  = self.SBEL.copy()

        # freeze state
        self.mfreeze_newt = 0
        self.dvbef        = 0.0

        # run init calculations to populate missile attributes
        self._init(m)


    def _init(self, m):
        """Populate initial VBEL, hbe, dvbe on the missile object."""
        VBEL = m.TBL.T @ self.VBEB
        hbe  = -self.SBEL[2]
        dvbe = float(np.linalg.norm(VBEL))

        m.VBEL   = VBEL
        m.SBEL   = self.SBEL.copy()
        m.VBEB   = self.VBEB.copy()
        m.hbe    = hbe
        m.dvbe   = dvbe
        m.SLEL   = self.SLEL.copy()


    # ── main update ────────────────────────────────────────────────────────────

    def newton(self, int_step):
        """Translational equations of motion (Newton's 2nd Law in body axes).

        Reads from missile:
            FAPB   - total non-gravitational force in body axes - N
            WBECB  - angular velocity [p, q, r] in body axes - rad/s
            TBL    - body-wrt-local DCM
            mass   - vehicle mass - kg
            grav   - gravity acceleration - m/s^2
            mfreeze

        Writes to missile:
            VBEB   - velocity in body axes - m/s
            VBEL   - velocity in local axes - m/s
            SBEL   - position in local axes - m
            sbel1/2/3 - scalar position components - m
            dvbe   - total speed - m/s
            hbe    - altitude - m
            FSPCB  - specific force in body axes - m/s^2
            psivlx - heading angle - deg
            thtvlx - vertical flight path angle - deg
            anx    - normal specific force - g's
            ayx    - lateral specific force - g's
        """
        m    = self.missile
        TBL  = m.TBL
        FAPB = m.FAPB
        mass = m.mass
        grav = m.grav
        mfreeze = m.mfreeze

        # ── tangential (Coriolis) term: WBECB × VBEB ───────────────────────
        ATB = np.cross(m.WBECB, self.VBEB)

        # ── gravity in body axes: TBL * [0, 0, grav] ───────────────────────
        GRAVL = np.array([0.0, 0.0, grav])   # NED: gravity acts downward

        # ── specific force (aerodynamic + propulsive) ───────────────────────
        FSPB = FAPB / mass

        # ── body-velocity derivative (Newton's 2nd in rotating frame) ───────
        VBEBD_NEW = FSPB - ATB + TBL @ GRAVL

        # integrate VBEB
        self.VBEB  = _integrate_vec(VBEBD_NEW, self.VBEBD, self.VBEB, int_step)
        self.VBEBD = VBEBD_NEW

        # ── position integration in local (NED) frame ────────────────────────
        VBEL = TBL.T @ self.VBEB

        SBELD_NEW = VBEL
        self.SBEL  = _integrate_vec(SBELD_NEW, self.SBELD, self.SBEL, int_step)
        self.SBELD = SBELD_NEW

        # ── flight path angles ───────────────────────────────────────────────
        vbel1 = VBEL[0]
        vbel2 = VBEL[1]
        vbel3 = VBEL[2]

        if vbel1 == 0.0 and vbel2 == 0.0:
            psivl = 0.0
        else:
            psivl = math.atan2(vbel2, vbel1)

        thtvl = math.atan2(-vbel3, math.sqrt(vbel1*vbel1 + vbel2*vbel2))

        psivlx = psivl * (180.0 / math.pi)
        thtvlx = thtvl * (180.0 / math.pi)

        # ── scalar speed and altitude ────────────────────────────────────────
        dvbe = float(np.linalg.norm(VBEL))
        hbe  = -self.SBEL[2]

        # ── diagnostic accelerations in g's ─────────────────────────────────
        anx = -FSPB[2] / grav    # normal (pitch plane)
        ayx =  FSPB[1] / grav    # lateral (yaw plane)

        # ── freeze for autopilot response calculations ───────────────────────
        if mfreeze == 0:
            self.mfreeze_newt = 0
        else:
            if mfreeze != self.mfreeze_newt:
                self.mfreeze_newt = mfreeze
                self.dvbef        = dvbe
            dvbe = self.dvbef

        # ── push outputs to missile ─────────────────────────────────────────
        m.VBEB   = self.VBEB
        m.VBEL   = VBEL
        m.SBEL   = self.SBEL
        m.sbel1  = self.SBEL[0]
        m.sbel2  = self.SBEL[1]
        m.sbel3  = self.SBEL[2]
        m.dvbe   = dvbe
        m.hbe    = hbe

        # truth channel (read only by INS)
        m.FSPB   = FSPB
        # estimate channel — passthrough; INS overwrites when mins=1
        m.FSPCB  = FSPB

        m.psivlx = psivlx
        m.thtvlx = thtvlx
        m.anx    = anx
        m.ayx    = ayx
