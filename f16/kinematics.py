"""
F-16 Kinematics module.

Propagates attitude via quaternion differential equations (numerically
more stable than Euler-angle integration for large rotations).

Quaternion convention  q = [q0, q1, q2, q3]  where q0 is the scalar part.

Quaternion ODE (with orthogonality correction gain ck):
    erq  = 1 − (q0²+q1²+q2²+q3²)
    q0̇ =  0.5·(−p·q1 − q·q2 − r·q3) + ck·erq·q0
    q1̇ =  0.5·( p·q0 + r·q2 − q·q3) + ck·erq·q1
    q2̇ =  0.5·( q·q0 − r·q1 + p·q3) + ck·erq·q2
    q3̇ =  0.5·( r·q0 + q·q1 − p·q2) + ck·erq·q3

TBL (body wrt local-level, ZYX Euler) from quaternions:
    TBL[0,0] = q0²+q1²−q2²−q3²   TBL[0,1] = 2(q1q2+q0q3)  TBL[0,2] = 2(q1q3−q0q2)
    TBL[1,0] = 2(q1q2−q0q3)       TBL[1,1] = q0²−q1²+q2²−q3²  TBL[1,2] = 2(q2q3+q0q1)
    TBL[2,0] = 2(q1q3+q0q2)       TBL[2,1] = 2(q2q3−q0q1)  TBL[2,2] = q0²−q1²−q2²+q3²

Incidence angles use VBAL (velocity wrt air in local axes) → VBAB = TBL·VBAL.
"""

import math
import numpy as np

RAD = math.pi / 180.0
DEG = 180.0 / math.pi
EPS = 1e-10


def _integrate(dydx_new, dydx, y, dt):
    return y + (dydx_new + dydx) * 0.5 * dt


def _sign(x):
    return math.copysign(1.0, x) if x != 0.0 else 1.0


def _mat3tr(psi, tht, phi):
    """Body-wrt-local DCM for 3-2-1 (ZYX) Euler sequence."""
    cp = math.cos(psi);  sp = math.sin(psi)
    ct = math.cos(tht);  st = math.sin(tht)
    cf = math.cos(phi);  sf = math.sin(phi)
    return np.array([
        [ct*cp,              ct*sp,            -st   ],
        [sf*st*cp - cf*sp,   sf*st*sp + cf*cp,  sf*ct],
        [cf*st*cp + sf*sp,   cf*st*sp - sf*cp,  cf*ct],
    ])


class Kinematics:

    def __init__(self, f16):
        self.f16 = f16

        self.ck = 50.0    # quaternion orthogonalising gain

        # quaternion states
        self.q0  = 1.0;  self.q0d = 0.0
        self.q1  = 0.0;  self.q1d = 0.0
        self.q2  = 0.0;  self.q2d = 0.0
        self.q3  = 0.0;  self.q3d = 0.0

        # DCM and transpose
        self.TBL = np.eye(3)
        self.TLB = np.eye(3)

        # Euler angles
        self.psiblx = 0.0   # yaw  - deg
        self.thtblx = 0.0   # pitch - deg
        self.phiblx = 0.0   # roll  - deg

        # incidence angles
        self.alphax = 0.0   # angle of attack - deg
        self.betax  = 0.0   # sideslip - deg
        self.alppx  = 0.0   # total AoA - deg
        self.phipx  = 0.0   # aero roll angle - deg

        # diagnostics
        self.erq  = 0.0    # quaternion orthogonality error
        self.etbl = 0.0    # DCM non-orthogonality error

    # ── initialisation ─────────────────────────────────────────────────────────

    def init(self):
        """Build quaternions and TBL from initial Euler angles on f16."""
        f = self.f16
        psi = f.psiblx * RAD
        tht = f.thtblx * RAD
        phi = f.phiblx * RAD

        sp = math.sin(psi / 2.0); cp = math.cos(psi / 2.0)
        st = math.sin(tht / 2.0); ct = math.cos(tht / 2.0)
        sf = math.sin(phi / 2.0); cf = math.cos(phi / 2.0)

        self.q0 = cp*ct*cf + sp*st*sf
        self.q1 = cp*ct*sf - sp*st*cf
        self.q2 = cp*st*cf + sp*ct*sf
        self.q3 = -cp*st*sf + sp*ct*cf

        self.TBL = _mat3tr(psi, tht, phi)
        self.TLB = self.TBL.T.copy()

        f.TBL    = self.TBL.copy()
        f.TLB    = self.TLB.copy()
        f.alphax = f.alpha0x
        f.betax  = f.beta0x

    # ── main kinematics step ───────────────────────────────────────────────────

    def kinematics(self, int_step):
        f    = self.f16
        WBEB = f.WBEB       # [p, q, r] rad/s
        VBAL = f.environment.VBAL   # velocity wrt air in local axes - m/s
        dvba = f.environment.dvba   # airspeed magnitude - m/s

        p, q, r = WBEB[0], WBEB[1], WBEB[2]
        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        ck = self.ck

        # ── quaternion ODE ────────────────────────────────────────────────────
        erq = 1.0 - (q0*q0 + q1*q1 + q2*q2 + q3*q3)

        new_q0d =  0.5*(-p*q1 - q*q2 - r*q3) + ck*erq*q0
        new_q1d =  0.5*( p*q0 + r*q2 - q*q3) + ck*erq*q1
        new_q2d =  0.5*( q*q0 - r*q1 + p*q3) + ck*erq*q2
        new_q3d =  0.5*( r*q0 + q*q1 - p*q2) + ck*erq*q3

        self.q0 = _integrate(new_q0d, self.q0d, q0, int_step)
        self.q1 = _integrate(new_q1d, self.q1d, q1, int_step)
        self.q2 = _integrate(new_q2d, self.q2d, q2, int_step)
        self.q3 = _integrate(new_q3d, self.q3d, q3, int_step)

        self.q0d = new_q0d
        self.q1d = new_q1d
        self.q2d = new_q2d
        self.q3d = new_q3d

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3

        # ── TBL from quaternions ──────────────────────────────────────────────
        TBL = np.array([
            [q0*q0+q1*q1-q2*q2-q3*q3, 2*(q1*q2+q0*q3),          2*(q1*q3-q0*q2)         ],
            [2*(q1*q2-q0*q3),          q0*q0-q1*q1+q2*q2-q3*q3,  2*(q2*q3+q0*q1)         ],
            [2*(q1*q3+q0*q2),          2*(q2*q3-q0*q1),           q0*q0-q1*q1-q2*q2+q3*q3],
        ])
        TLB = TBL.T

        # DCM orthogonality check (diagonal of TLB@TBL should be [1,1,1])
        diag = np.diag(TLB @ TBL)
        e = diag - 1.0
        etbl = math.sqrt(float(np.dot(e, e)))

        # ── Euler angles from TBL ─────────────────────────────────────────────
        tbl13 = TBL[0, 2]
        if abs(tbl13) < 1.0:
            thtbl  = math.asin(-tbl13)
            cthtbl = math.cos(thtbl)
        else:
            thtbl  = math.pi / 2.0 * _sign(-tbl13)
            cthtbl = EPS

        cpsi = TBL[0, 0] / cthtbl
        if abs(cpsi) >= 1.0:
            cpsi = (1.0 - EPS) * _sign(cpsi)
        cphi = TBL[2, 2] / cthtbl
        if abs(cphi) >= 1.0:
            cphi = (1.0 - EPS) * _sign(cphi)

        psibl = math.acos(cpsi) * _sign(TBL[0, 1])
        phibl = math.acos(cphi) * _sign(TBL[1, 2])

        # ── incidence angles ──────────────────────────────────────────────────
        VBAB  = TBL @ VBAL
        vbab1, vbab2, vbab3 = VBAB[0], VBAB[1], VBAB[2]

        alpha = math.atan2(vbab3, vbab1)
        beta  = math.asin(max(-1.0, min(1.0, vbab2 / max(dvba, EPS))))

        dum  = max(-1.0, min(1.0, vbab1 / max(dvba, EPS)))
        alpp = math.acos(dum)

        if vbab2 == 0.0 and vbab3 == 0.0:
            phip = 0.0
        elif abs(vbab2) < EPS:
            phip = 0.0 if vbab3 > 0.0 else math.pi
        else:
            phip = math.atan2(vbab2, vbab3)

        # ── store ─────────────────────────────────────────────────────────────
        self.TBL    = TBL
        self.TLB    = TLB
        self.erq    = erq
        self.etbl   = etbl
        self.psiblx = psibl * DEG
        self.thtblx = thtbl * DEG
        self.phiblx = phibl * DEG
        self.alphax = alpha * DEG
        self.betax  = beta  * DEG
        self.alppx  = alpp  * DEG
        self.phipx  = phip  * DEG

        f.TBL    = TBL
        f.TLB    = TLB
        f.alphax = alpha * DEG
        f.betax  = beta  * DEG
        f.psiblx = psibl * DEG
        f.thtblx = thtbl * DEG
        f.phiblx = phibl * DEG
        f.alppx  = alpp  * DEG
