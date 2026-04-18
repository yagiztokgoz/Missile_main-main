"""
Kinematic (ideal) seeker — SRAAM6 port of CADAC sensor.cpp (sensor_ir_kin).

Phase-1 port: perfect LOS data from truth geometry — no gimbal dynamics,
no blur/bias, no Kalman filter. Useful for validating the terminal-homing
pipeline (handover from midcourse pronav to terminal compensated pronav)
before adding error models in Phase 2 (dynamic IIR seeker).

Produces:
    psipb, thtpb        — gimbal pointing angles (body frame) - rad
    sigdpy, sigdpz      — inertial LOS rates (pointing frame) - rad/s

State machine (mseek):
    0: OFF         — outputs zero, guidance uses midcourse only
    2: ENABLED     — armed, waiting to enter acquisition range
    3: ACQUISITION — within racq, accumulating dtimac before lock
    4: LOCK-ON     — valid outputs to guidance (terminal pronav fires)
    5: BLIND       — within dblind, outputs held constant

Call order: sensor.sensor() must run AFTER ins.ins() (time/geometry ready)
            and BEFORE guidance.guidance() (consumes LOS data).
"""

import math
import numpy as np

DEG   = 180.0 / math.pi
SMALL = 1e-7


def _pol_from_cart(v):
    """Cartesian → (range, azimuth, elevation) in CADAC convention."""
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    r = math.sqrt(x*x + y*y + z*z)
    if r > SMALL:
        el = math.asin(max(-1.0, min(1.0, -z / r)))
        d  = math.sqrt(x*x + y*y)
        az = math.atan2(y, x) if d > SMALL else 0.0
    else:
        az, el = 0.0, 0.0
    return r, az, el


def _mat2tr(az, el):
    """2-axis rotation DCM (az then el, no roll). Same as guidance.mat2tr."""
    ca, sa = math.cos(az), math.sin(az)
    ce, se = math.cos(el), math.sin(el)
    return np.array([
        [ ca*ce,  sa*ce, -se ],
        [ -sa,    ca,     0. ],
        [ ca*se,  sa*se,  ce ],
    ])


class Sensor(object):

    def __init__(self, missile):
        self.m = missile

        # state machine timer
        self.epchac = 0.0    # epoch acquisition mode started - s
        self.timeac = 0.0    # elapsed acquisition time        - s

        # latched outputs (held during BLIND, updated in ACQUISITION/LOCK)
        self.psipb  = 0.0
        self.thtpb  = 0.0
        self.sigdpy = 0.0
        self.sigdpz = 0.0

        # diagnostics
        self.dbtk  = 0.0     # true range - m
        self.dvbtc = 0.0     # closing speed - m/s


    def init(self):
        """Zero the seeker output channels on the missile object."""
        m = self.m
        m.psipb  = 0.0
        m.thtpb  = 0.0
        m.sigdpy = 0.0
        m.sigdpz = 0.0
        m.psipbx = 0.0
        m.thtpbx = 0.0


    # ── main tick ────────────────────────────────────────────────────────────

    def sensor(self, dt):
        m     = self.m
        mseek = m.mseek

        if mseek == 0:
            # OFF — publish zeros, no state transitions
            m.psipb = 0.0; m.thtpb = 0.0
            m.sigdpy = 0.0; m.sigdpz = 0.0
            m.psipbx = 0.0; m.thtpbx = 0.0
            return

        # truth geometry (kinematic seeker uses truth; Phase 2 will corrupt)
        SBTL = m.SBEL - m.STEL
        dbtk = float(np.linalg.norm(SBTL))
        self.dbtk = dbtk

        # ── state machine transitions ────────────────────────────────────────
        if mseek == 2 and dbtk < m.racq:
            mseek       = 3
            self.epchac = m.time

        if mseek == 3:
            self.timeac = m.time - self.epchac
            if self.timeac > m.dtimac:
                mseek = 4

        if mseek == 4 and dbtk < m.dblind:
            mseek = 5        # hold outputs, below blind range

        # ── compute LOS data when tracking ───────────────────────────────────
        if mseek in (3, 4):
            psipb, thtpb, sigdpy, sigdpz, dvbtc = self._sensor_kin(SBTL, dbtk)
            self.psipb  = psipb
            self.thtpb  = thtpb
            self.sigdpy = sigdpy
            self.sigdpz = sigdpz
            self.dvbtc  = dvbtc
        # else (mseek == 2 or 5): keep last values (zeros / latched)

        # ── publish ──────────────────────────────────────────────────────────
        m.mseek   = mseek
        m.psipb   = self.psipb
        m.thtpb   = self.thtpb
        m.sigdpy  = self.sigdpy
        m.sigdpz  = self.sigdpz
        m.psipbx  = self.psipb * DEG
        m.thtpbx  = self.thtpb * DEG


    # ── kinematic LOS computation (truth-based) ──────────────────────────────

    def _sensor_kin(self, SBTL, dbtk):
        """Ideal seeker: LOS angles + inertial LOS rates from geometry.

        CADAC sensor_ir_kin convention:
            STBL = -SBTL                (target wrt missile, local)
            STBB = TBL · STBL           (rotated to body)
            psipb, thtpb = pol_from_cart(STBB)[1:]
            WOEB = TBL · (UTBL × VTBL) / dbtk
            sigdpy, sigdpz = (TPB · WOEB)[1], [2]
        """
        m    = self.m
        TBL  = m.TBL          # truth DCM (kinematic seeker is ideal)
        VBEL = m.VBEL
        VTEL = m.VTEL

        STBL = -SBTL                              # target wrt missile, local
        UTBL = STBL / dbtk
        STBB = TBL @ STBL                         # target wrt missile, body

        # relative velocity (target wrt missile)
        VTBL = VTEL - VBEL
        dvbtc = abs(float(np.dot(UTBL, VTBL)))

        # inertial LOS rate rotated to body then to pointing frame
        WOEL = np.cross(UTBL, VTBL) / dbtk
        WOEB = TBL @ WOEL

        _, psipb, thtpb = _pol_from_cart(STBB)

        TPB  = _mat2tr(psipb, thtpb)
        WOEP = TPB @ WOEB
        sigdpy = float(WOEP[1])
        sigdpz = float(WOEP[2])

        return psipb, thtpb, sigdpy, sigdpz, dvbtc
