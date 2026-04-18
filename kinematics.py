import math
import numpy as np

RAD   = math.pi / 180.0
DEG   = 180.0 / math.pi
EPS   = 1e-10   # avoid divide-by-zero
SMALL = 1e-7    # near-zero check for phip branch


def _sign(x):
    return 1.0 if x >= 0.0 else -1.0


def _integrate(dydx_new, dydx, y, int_step):
    """Trapezoidal integration (CADAC standard)."""
    return y + (dydx_new + dydx) / 2.0 * int_step


def _mat3tr(psi, tht, phi):
    """Direction cosine matrix body-wrt-local for 3-2-1 (Z-Y-X) Euler sequence.

    Args:
        psi: yaw  angle - rad
        tht: pitch angle - rad
        phi: roll  angle - rad
    Returns:
        TBL: 3×3 numpy array
    """
    c1, s1 = math.cos(psi), math.sin(psi)
    c2, s2 = math.cos(tht), math.sin(tht)
    c3, s3 = math.cos(phi), math.sin(phi)

    return np.array([
        [ c1*c2,               s1*c2,              -s2    ],
        [ c1*s2*s3 - s1*c3,   s1*s2*s3 + c1*c3,   c2*s3  ],
        [ c1*s2*c3 + s1*s3,   s1*s2*c3 - c1*s3,   c2*c3  ],
    ])


class Kinematics(object):

    def __init__(self, missile):
        self.missile = missile

        # orthogonality correction gain
        self.ck = 50.0

        # quaternion states and their derivatives
        self.q0 = 1.0;  self.q0d = 0.0
        self.q1 = 0.0;  self.q1d = 0.0
        self.q2 = 0.0;  self.q2d = 0.0
        self.q3 = 0.0;  self.q3d = 0.0

        # DCM outputs
        self.TBL = np.eye(3)
        self.TLB = np.eye(3)

        # diagnostics
        self.ortho_error = 0.0
        self.etbl        = 0.0

        # initialise everything from missile Euler angles
        self._init_from_euler()


    # ── initialisation ─────────────────────────────────────────────────────────

    def _init_from_euler(self):
        """Set quaternion and TBL from missile initial Euler angles (psix/thtx/phix)."""
        m = self.missile

        psi = m.psix * RAD
        tht = m.thtx * RAD
        phi = m.phix * RAD

        # quaternion from half-angle products (3-2-1 sequence)
        cpsi = math.cos(psi / 2.0);  spsi = math.sin(psi / 2.0)
        ctht = math.cos(tht / 2.0);  stht = math.sin(tht / 2.0)
        cphi = math.cos(phi / 2.0);  sphi = math.sin(phi / 2.0)

        self.q0 =  cpsi*ctht*cphi + spsi*stht*sphi
        self.q1 =  cpsi*ctht*sphi - spsi*stht*cphi
        self.q2 =  cpsi*stht*cphi + spsi*ctht*sphi
        self.q3 = -cpsi*stht*sphi + spsi*ctht*cphi

        TBL = _mat3tr(psi, tht, phi)
        self.TBL = TBL
        self.TLB = TBL.T.copy()

        # push initial attitude to missile
        m.TBL    = self.TBL
        m.TLB    = self.TLB
        m.psiblx = m.psix
        m.thtblx = m.thtx
        m.phiblx = m.phix


    # ── main update ────────────────────────────────────────────────────────────

    def kinematics(self, int_step):
        """Integrates quaternion ODEs; updates DCM, Euler angles, and incidence angles.

        Reads from missile:
            WBECB   - angular velocity [p, q, r] - rad/s
            VBEB    - velocity in body frame - m/s

        Writes to missile:
            TBL, TLB          - direction cosine matrices
            psiblx, thtblx, phiblx  - Euler angles - deg
            alphax, betax     - incidence angles - deg
            alppx, alpp       - total angle of attack - deg / rad
            phipx, phip       - aerodynamic roll angle - deg / rad
            dvbe              - total speed - m/s
        """
        m  = self.missile
        ck = self.ck

        pp = m.WBECB[0]   # roll rate  - rad/s
        qq = m.WBECB[1]   # pitch rate - rad/s
        rr = m.WBECB[2]   # yaw rate   - rad/s

        # ── quaternion integration ──────────────────────────────────────────
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        ortho_error = 1.0 - (q0*q0 + q1*q1 + q2*q2 + q3*q3)

        new_q0d = 0.5*(-pp*q1 - qq*q2 - rr*q3) + ck*ortho_error*q0
        new_q1d = 0.5*( pp*q0 + rr*q2 - qq*q3) + ck*ortho_error*q1
        new_q2d = 0.5*( qq*q0 - rr*q1 + pp*q3) + ck*ortho_error*q2
        new_q3d = 0.5*( rr*q0 + qq*q1 - pp*q2) + ck*ortho_error*q3

        self.q0 = _integrate(new_q0d, self.q0d, q0, int_step)
        self.q1 = _integrate(new_q1d, self.q1d, q1, int_step)
        self.q2 = _integrate(new_q2d, self.q2d, q2, int_step)
        self.q3 = _integrate(new_q3d, self.q3d, q3, int_step)

        self.q0d = new_q0d
        self.q1d = new_q1d
        self.q2d = new_q2d
        self.q3d = new_q3d

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        # ── DCM from updated quaternion ─────────────────────────────────────
        TBL = np.array([
            [q0*q0+q1*q1-q2*q2-q3*q3,  2.*(q1*q2+q0*q3),           2.*(q1*q3-q0*q2)          ],
            [2.*(q1*q2-q0*q3),           q0*q0-q1*q1+q2*q2-q3*q3,  2.*(q2*q3+q0*q1)           ],
            [2.*(q1*q3+q0*q2),           2.*(q2*q3-q0*q1),           q0*q0-q1*q1-q2*q2+q3*q3  ],
        ])
        TLB = TBL.T

        # orthogonality check (diagnostic)
        UBL  = TLB @ TBL
        e1   = UBL[0, 0] - 1.0
        e2   = UBL[1, 1] - 1.0
        e3   = UBL[2, 2] - 1.0
        etbl = math.sqrt(e1*e1 + e2*e2 + e3*e3)

        # ── Euler angles from TBL ───────────────────────────────────────────
        tbl13 = TBL[0, 2]
        tbl11 = TBL[0, 0]
        tbl33 = TBL[2, 2]
        tbl12 = TBL[0, 1]
        tbl23 = TBL[1, 2]

        if abs(tbl13) < 1.0:
            thtbl  = math.asin(-tbl13)
            cthtbl = math.cos(thtbl)
        else:
            thtbl  = math.pi / 2.0 * _sign(-tbl13)
            cthtbl = EPS

        cpsi = tbl11 / cthtbl
        if abs(cpsi) >= 1.0:
            cpsi = (1.0 - EPS) * _sign(cpsi)
        cphi = tbl33 / cthtbl
        if abs(cphi) >= 1.0:
            cphi = (1.0 - EPS) * _sign(cphi)

        psibl = math.acos(cpsi) * _sign(tbl12)
        phibl = math.acos(cphi) * _sign(tbl23)

        psiblx = psibl * DEG
        thtblx = thtbl * DEG
        phiblx = phibl * DEG

        # ── incidence angles from body velocity ─────────────────────────────
        VBEB  = m.VBEB
        vbeb1 = float(VBEB[0])
        vbeb2 = float(VBEB[1])
        vbeb3 = float(VBEB[2])

        dvbe = math.sqrt(vbeb1*vbeb1 + vbeb2*vbeb2 + vbeb3*vbeb3)
        if dvbe < EPS:
            dvbe = EPS

        alpha = math.atan2(vbeb3, vbeb1)
        beta  = math.asin(max(-1.0, min(1.0, vbeb2 / dvbe)))

        dum = vbeb1 / dvbe
        if abs(dum) >= 1.0:
            dum = (1.0 - EPS) * _sign(dum)
        alpp = math.acos(dum)

        if abs(vbeb2) < EPS and abs(vbeb3) < EPS:
            phip = 0.0
        elif abs(vbeb2) < SMALL:
            phip = math.atan2(SMALL, vbeb3)
        else:
            phip = math.atan2(vbeb2, vbeb3)

        alphax = alpha * DEG
        betax  = beta  * DEG
        alppx  = alpp  * DEG
        phipx  = phip  * DEG

        # ── store ───────────────────────────────────────────────────────────
        self.TBL         = TBL
        self.TLB         = TLB
        self.ortho_error = ortho_error
        self.etbl        = etbl

        # push to missile
        m.TBL    = TBL
        m.TLB    = TLB
        m.dvbe   = dvbe
        m.psiblx = psiblx
        m.thtblx = thtblx
        m.phiblx = phiblx
        m.alphax = alphax
        m.betax  = betax
        m.alppx  = alppx
        m.alpp   = alpp
        m.phipx  = phipx
        m.phip   = phip
