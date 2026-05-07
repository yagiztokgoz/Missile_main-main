"""
INS (Inertial Navigation System) — SRAAM6 port of CADAC ins.cpp.

Ported with SRAAM-scale simplifications:
  - No 9×9 Cholesky transfer-alignment init (start at zero error)
  - Earth-curvature coupling (REARTH, tanlat, Schuler) omitted
    — negligible for <10 s engagements at Mach 4

Produces 'computed' estimates (SBELC, VBELC, TBLC, WBECB, FSPCB, hbem ...)
from the truth physics states (SBEL, VBEL, TBL, WBEB, FSPB, hbe).

mins = 0 : ideal INS — estimates = truth (bit-identical to pre-INS behavior)
mins = 1 : real INS  — bias + random walk + scale factor + misalignment
                     + gyro mass unbalance + altimeter bias/noise

Call order: ins.ins(dt) must run AFTER forces.forces() (needs FAPB/mass → FSPB)
            and BEFORE guidance.guidance()/control.control() (they consume estimates).

Error-parameter realizations are drawn ONCE per Missile() construction (a
physical IMU has fixed per-unit tolerances); random walk noise ticks each call.
"""

import math
import numpy as np

DEG = 180.0 / math.pi


def _integrate_vec(dydx_new, dydx, y, dt):
    """Trapezoidal integration — matches the CADAC convention used elsewhere."""
    return y + (dydx_new + dydx) * 0.5 * dt


def _skew(v):
    return np.array([
        [0.0,   -v[2],  v[1]],
        [v[2],   0.0,  -v[0]],
        [-v[1],  v[0],  0.0],
    ])


class INS(object):

    def __init__(self, missile):
        self.m = missile

        # ── IMU error realizations (drawn once per flight) ───────────────────
        # σ values are tactical-grade defaults from CADAC (gauss(0, σ)).
        # User can overwrite any of these on the missile object before init().
        rng = np.random.default_rng(missile.ins_seed) if missile.ins_seed is not None else np.random.default_rng()
        self.rng = rng

        # gyro
        self.EBIASG = rng.normal(0.0, 3.2e-6, 3)   # rad/s        — turn-on bias
        self.ESCALG = rng.normal(0.0, 2.5e-5, 3)   # parts        — scale factor
        self.EMISG  = rng.normal(0.0, 1.1e-4, 3)   # rad          — cluster misalign
        self.EWALKG = np.full(3, 1.0e-6)           # rad/√s       — random walk std
        self.EUNBG  = np.zeros(3)                  # (rad/s)/(m/s²) — mass unbalance
        # accel
        self.EBIASA = rng.normal(0.0, 3.56e-3, 3)  # m/s²
        self.ESCALA = rng.normal(0.0, 5.0e-4, 3)
        self.EMISA  = rng.normal(0.0, 1.1e-4, 3)
        self.EWALKA = np.full(3, 1.0e-4)           # (m/s)/√s
        # altimeter
        self.biasal = 0.0                          # m — constant bias
        self.randal = 0.0                          # m — per-tick noise std

        # ── error states + derivatives ───────────────────────────────────────
        self.RECE   = np.zeros(3);  self.RECED   = np.zeros(3)   # tilt - rad
        self.EVBE   = np.zeros(3);  self.EVBED   = np.zeros(3)   # vel  - m/s
        self.ESTTC  = np.zeros(3);  self.ESTTCD  = np.zeros(3)   # pos  - m

        # diagnostics
        self.EFSPB = np.zeros(3)
        self.EWBEB = np.zeros(3)


    # ── one-shot init (called from Missile.__init__) ─────────────────────────

    def init(self):
        """Seed estimate channels with truth at t=0 (perfect alignment)."""
        m = self.m
        m.SBELC = m.SBEL.copy()
        m.VBELC = m.VBEL.copy()
        m.TBLC  = m.TBL.copy()
        m.VBEBC = m.VBEB.copy()
        m.WBECB = np.zeros(3)       # body rates unknown at t=0
        m.FSPCB = np.zeros(3)       # spec force  unknown at t=0
        m.dvbec   = m.dvbe
        m.hbem    = m.hbe
        m.thtblc  = m.thtblx * math.pi / 180.0
        m.thtblcx = m.thtblx
        m.phiblcx = m.phix
        m.psivlcx = 0.0
        m.thtvlcx = 0.0
        m.alphaxc = 0.0             # α estimate - deg
        m.betaxc  = 0.0             # β estimate - deg


    # ── main tick ────────────────────────────────────────────────────────────

    def ins(self, dt):
        m = self.m
        mins = m.mins

        # truth inputs (from last step's physics modules)
        TBL  = m.TBL
        TLB  = m.TLB
        WBEB = m.WBEB          # truth body rates  (euler)
        FSPB = m.FSPB          # truth spec force  (newton)
        SBEL = m.SBEL
        VBEL = m.VBEL
        dvbe = m.dvbe
        hbe  = m.hbe

        if mins == 0:
            # ── ideal: passthrough ──────────────────────────────────────────
            SBELC = SBEL.copy()
            VBELC = VBEL.copy()
            TBLC  = TBL.copy()
            WBECB = WBEB.copy()
            FSPCB = FSPB.copy()
            dvbec = dvbe
            hbem  = hbe
            # keep error states at zero (they're not propagated)
        else:
            # ── 1) raw sensor measurements ─────────────────────────────────
            EFSPB            = self._accl_errors(FSPB)
            EWBEB, WBECB     = self._gyro_errors(WBEB, FSPB, dt)

            # ── 2) tilt error dynamics (skipping Schuler terms) ────────────
            EWBEL      = TLB @ EWBEB
            RECED_NEW  = EWBEL
            self.RECE  = _integrate_vec(RECED_NEW, self.RECED, self.RECE, dt)
            self.RECED = RECED_NEW

            # INS-derived DCM: TBLC = TBL · (I + skew(RECE))   (small-angle)
            RERE = _skew(self.RECE)
            TBLC = TBL @ (np.eye(3) + RERE)
            TLCB = TBLC.T

            # ── 3) velocity error dynamics ─────────────────────────────────
            # accel random walk added directly to the spec-force channel
            ewalka_tick = self.EWALKA * self.rng.standard_normal(3) / math.sqrt(dt)
            FSPCB = FSPB + EFSPB + ewalka_tick
            EF    = TLCB @ EFSPB - RERE @ TLCB @ FSPCB
            EVBED_NEW  = EF
            self.EVBE  = _integrate_vec(EVBED_NEW, self.EVBED, self.EVBE, dt)
            self.EVBED = EVBED_NEW

            # ── 4) position error dynamics ─────────────────────────────────
            ESTTCD_NEW  = self.EVBE
            self.ESTTC  = _integrate_vec(ESTTCD_NEW, self.ESTTCD, self.ESTTC, dt)
            self.ESTTCD = ESTTCD_NEW

            # ── 5) assemble estimates ──────────────────────────────────────
            SBELC = self.ESTTC + SBEL
            VBELC = self.EVBE  + VBEL
            dvbec = float(np.linalg.norm(VBELC))

            # altimeter channel: baro output, separate from INS vertical
            ehbe  = self.biasal + self.randal * self.rng.standard_normal()
            hbem  = hbe + ehbe

            self.EFSPB = EFSPB
            self.EWBEB = EWBEB

        # ── derived estimate quantities (both modes) ─────────────────────────
        v1, v2, v3 = float(VBELC[0]), float(VBELC[1]), float(VBELC[2])
        if v1 == 0.0 and v2 == 0.0:
            psivlc = 0.0;  thtvlc = 0.0
        else:
            psivlc = math.atan2(v2, v1)
            thtvlc = math.atan2(-v3, math.sqrt(v1*v1 + v2*v2))

        # incidence angles from VBELC rotated into body frame by TBLC
        VBEBC = TBLC @ VBELC
        vb1 = float(VBEBC[0]); vb2 = float(VBEBC[1]); vb3 = float(VBEBC[2])
        dvbebc = math.sqrt(vb1*vb1 + vb2*vb2 + vb3*vb3)
        if dvbebc < 1e-6:
            dvbebc = 1e-6
        alphac = math.atan2(vb3, vb1)
        betac  = math.asin(max(-1.0, min(1.0, vb2 / dvbebc)))

        tblc13 = TBLC[0, 2]
        if abs(tblc13) < 1.0:
            thtblc = math.asin(-tblc13)
        else:
            thtblc = math.pi * 0.5 * (-1.0 if tblc13 > 0.0 else 1.0)
        phiblc = math.atan2(TBLC[1, 2], TBLC[2, 2])

        # ── store estimate outputs on missile ────────────────────────────────
        m.SBELC   = SBELC
        m.VBELC   = VBELC
        m.TBLC    = TBLC
        m.WBECB   = WBECB
        m.FSPCB   = FSPCB
        m.dvbec   = dvbec
        m.hbem    = hbem
        m.thtblc  = thtblc
        m.thtblcx = thtblc * DEG
        m.phiblcx = phiblc * DEG
        m.psivlcx = psivlc * DEG
        m.thtvlcx = thtvlc * DEG
        m.alphaxc = alphac * DEG
        m.betaxc  = betac  * DEG
        m.VBEBC   = VBEBC


    # ── sensor error models ──────────────────────────────────────────────────

    def _gyro_errors(self, WBEB, FSPB, dt):
        """Returns (EWBEB, WBECB_meas). EWBEB = measurement error - rad/s."""
        EGB    = np.diag(self.ESCALG) + _skew(self.EMISG)
        EMISCG = EGB @ WBEB
        EMSBG  = self.EBIASG + EMISCG
        EUG    = self.EUNBG * FSPB       # mass unbalance (g-sensitive)
        if np.any(self.EWALKG != 0.0):
            EWG = self.EWALKG * self.rng.standard_normal(3) / math.sqrt(dt)
        else:
            EWG = np.zeros(3)
        EWBEB = EMSBG + EUG + EWG
        WBECB = WBEB + EWBEB
        return EWBEB, WBECB

    def _accl_errors(self, FSPB):
        """Returns accelerometer measurement error EFSPB (random walk applied later)."""
        EAB   = np.diag(self.ESCALA) + _skew(self.EMISA)
        EFSPB = self.EBIASA + EAB @ FSPB
        return EFSPB
