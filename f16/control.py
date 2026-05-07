"""
F-16 Control module — autopilot with multiple loop options.

maut encoding  |mauty|mautp|  (two-digit integer):
    mauty = maut // 10   lateral / yaw
    mautp = maut  % 10   longitudinal / pitch

    mauty: 0=none  2=yaw-rate SAS  3=yaw-accel  4=heading hold + SAS
    mautp: 0=none  2=pitch-rate SAS  3=pitch-accel  4=gamma hold  5=alt hold

    mroll: 0=roll position control  1=roll rate control

Roll controller (always active when maut > 0):
    mroll=0  pole-placement on bank angle error
    mroll=1  roll-rate command tracking

Pitch accel gain scheduling (Stevens & Lewis):
    waclp = 2 + 0.0001226·(q − 5500)
    zaclp = 0.7 − 0.0000245·(q − 5500)
    paclp = 5 + 0.0003067·(q − 5500)
    (q in Pa; nominal at 10 km / M0.55: q≈5500 Pa → waclp=2, zaclp=0.7, paclp=5)
"""

import math
import numpy as np

RAD   = math.pi / 180.0
DEG   = 180.0 / math.pi
AGRAV = 9.80665
SMALL = 1e-7


def _sign(x):
    return math.copysign(1.0, x) if x != 0.0 else 1.0


def _integrate(dydx_new, dydx, y, dt):
    return y + (dydx_new + dydx) * 0.5 * dt


class Control:

    def __init__(self, f16):
        self.f16 = f16

        # ── mode flags ─────────────────────────────────────────────────────────
        self.maut    = 43     # |mauty|mautp| e.g. 43 = heading + pitch accel
        self.mroll   = 0      # 0: roll position  1: roll rate
        self.mfreeze = 0      # freeze flag (read by env/prop/newton)

        # ── surface limits ──────────────────────────────────────────────────────
        self.dalimx  = 20.0   # aileron  limit - deg
        self.delimx  = 25.0   # elevator limit - deg
        self.drlimx  = 30.0   # rudder   limit - deg
        self.philimx = 70.0   # bank angle limit - deg
        self.anlimpx =  7.0   # pos g limit
        self.anlimnx =  3.0   # neg g limit (stored positive)

        # ── roll controller ─────────────────────────────────────────────────────
        self.wrcl    = 3.0    # roll CL nat freq - rad/s
        self.zrcl    = 0.7    # roll CL damping
        self.tp      = 0.1    # roll-rate CL time constant - s

        # ── rate controller ─────────────────────────────────────────────────────
        self.zetlagr = 0.7    # desired rate-loop damping

        # ── pitch accel controller ──────────────────────────────────────────────
        self.gainp   = 0.0    # proportional feed-through gain - s²/m

        # ── lateral accel → roll ────────────────────────────────────────────────
        self.gainl   = 1.0    # lateral accel gain - rad/g

        # ── gamma hold ──────────────────────────────────────────────────────────
        self.pgam    = 10.0   # real pole     - rad/s
        self.wgam    =  3.0   # nat freq      - rad/s
        self.zgam    =  0.7   # damping

        # ── heading hold ────────────────────────────────────────────────────────
        self.facthead = 0.0   # heading gain boost factor

        # ── altitude hold ───────────────────────────────────────────────────────
        self.gainalt     = 0.1    # alt error gain  - 1/s
        self.gainaltrate = 0.5    # alt rate gain   - 1/s
        self.max_altrate = 50.0   # desired climb/descent rate saturation - m/s
        # Physics: ancomx ≈ gainaltrate × max_altrate / g + 1
        # 50 m/s → ~2.3 g max pitch-up command — aggressive but controllable.

        # ── commands (set by guidance or externally) ────────────────────────────
        self.alcomx    = 0.0   # lateral accel command - g's
        self.ancomx    = 0.0   # normal  accel command - g's
        self.psivlcomx = 0.0   # heading command       - deg
        self.thtvlcomx = 0.0   # gamma command         - deg
        self.altcom    = 0.0   # altitude command      - m
        self.phicomx   = 0.0   # roll angle command    - deg
        self.pcomx     = 0.0   # roll rate command     - deg/s
        self.qcomx     = 0.0   # pitch rate command    - deg/s
        self.rcomx     = 0.0   # yaw rate command      - deg/s

        # ── integrator states ───────────────────────────────────────────────────
        self.zz  = 0.0        # pitch accel integrator - m/s
        self.zzd = 0.0

        # ── outputs ─────────────────────────────────────────────────────────────
        self.delacx = 0.0     # aileron  command - deg
        self.delecx = 0.0     # elevator command - deg
        self.delrcx = 0.0     # rudder   command - deg

        # ── diagnostics ─────────────────────────────────────────────────────────
        self.gainpsi = 0.0
        self.gkp     = 0.0
        self.gkphi   = 0.0
        self.GAINGAM = np.zeros(3)
        self.gainff  = 0.0

    # ── main dispatcher ────────────────────────────────────────────────────────

    def control(self, int_step):
        f = self.f16

        if self.maut == 0:
            return

        mauty = self.maut // 10
        mautp = self.maut  % 10

        aero  = f.aerodynamics
        gmax  = aero.gmax
        gminx = aero.gminx

        delacx = 0.0
        delecx = 0.0
        delrcx = 0.0
        phicomx = self.phicomx
        ancomx  = self.ancomx
        alcomx  = self.alcomx

        # ── yaw control ────────────────────────────────────────────────────────
        if mauty == 2:
            delrcx = self._yaw_rate(self.rcomx)

        elif mauty == 3:
            phicomx = self._lateral_accel(alcomx)

        elif mauty == 4:
            phicomx = self._heading(self.psivlcomx)
            delrcx  = self._yaw_rate(self.rcomx)

        # ── pitch control ──────────────────────────────────────────────────────
        if mautp == 2:
            delecx = self._pitch_rate(self.qcomx)

        elif mautp == 3:
            ancomx = max(gminx, min(ancomx, gmax))
            delecx = self._normal_accel(ancomx, int_step)

        elif mautp == 4:
            delecx = self._gamma(self.thtvlcomx)

        elif mautp == 5:
            ancomx = self._altitude(self.altcom)
            ancomx = max(-self.anlimnx, min(ancomx, self.anlimpx))
            ancomx = max(gminx, min(ancomx, gmax))
            delecx = self._normal_accel(ancomx, int_step)

        # ── roll control ───────────────────────────────────────────────────────
        if self.mroll == 0:
            phicomx = max(-self.philimx, min(phicomx, self.philimx))
            delacx = self._roll(phicomx)
        else:
            delacx = self._roll_rate(self.pcomx)

        # ── surface limits ─────────────────────────────────────────────────────
        delacx = max(-self.dalimx, min(delacx, self.dalimx))
        delecx = max(-self.delimx, min(delecx, self.delimx))
        delrcx = max(-self.drlimx, min(delrcx, self.drlimx))

        self.delacx = delacx
        self.delecx = delecx
        self.delrcx = delrcx

        f.delacx = delacx
        f.delecx = delecx
        f.delrcx = delrcx

    # ── roll position controller ───────────────────────────────────────────────

    def _roll(self, phicomx):
        f    = self.f16
        aero = f.aerodynamics
        dllp  = aero.dllp
        dllda = aero.dllda

        gkp   = (2.0*self.zrcl*self.wrcl + dllp) / dllda
        gkphi = self.wrcl**2 / dllda

        ephi   = gkphi * (phicomx - f.phiblx) * RAD
        dpc    = ephi - gkp * f.ppx * RAD
        delacx = dpc * DEG

        self.gkp   = gkp
        self.gkphi = gkphi
        return delacx

    # ── roll rate controller ───────────────────────────────────────────────────

    def _roll_rate(self, pcomx):
        f    = self.f16
        aero = f.aerodynamics
        kp     = (1.0 / self.tp + aero.dllp) / aero.dllda
        return kp * (pcomx - f.ppx)

    # ── pitch rate SAS ─────────────────────────────────────────────────────────

    def _pitch_rate(self, qcomx):
        f    = self.f16
        aero = f.aerodynamics
        dvbe = max(f.dvbe, 1.0)

        dla  = aero.dla;  dlde = aero.dlde
        dma  = aero.dma;  dmq  = aero.dmq;  dmde = max(abs(aero.dmde), SMALL) * _sign(aero.dmde)

        zrate = dla/dvbe - dma*dlde/(dvbe*dmde)
        aa    = dla/dvbe - dmq
        bb    = -dma - dmq*dla/dvbe

        dum1  = aa - 2.0*self.zetlagr**2 * zrate
        dum2  = aa**2 - 4.0*self.zetlagr**2 * bb
        radix = max(dum1**2 - dum2, 0.0)
        grate = (-dum1 + math.sqrt(radix)) / (-dmde)

        return grate * (f.qqx - qcomx)

    # ── yaw rate SAS ───────────────────────────────────────────────────────────

    def _yaw_rate(self, rcomx):
        f    = self.f16
        aero = f.aerodynamics
        dvbe = max(f.dvbe, 1.0)

        dyb  = aero.dyb;  dydr = aero.dydr
        dnb  = aero.dnb;  dnr  = aero.dnr
        dndr = aero.dndr if abs(aero.dndr) > SMALL else SMALL * _sign(aero.dndr)

        zrate = -dyb/dvbe + dnb*dydr/(dvbe*dndr)
        aa    = -dyb/dvbe - dnr
        bb    =  dnb + dyb*dnr/dvbe

        dum1  = aa - 2.0*self.zetlagr**2 * zrate
        dum2  = aa**2 - 4.0*self.zetlagr**2 * bb
        radix = max(dum1**2 - dum2, 0.0)
        grate = (-dum1 + math.sqrt(radix)) / (-dndr)

        return grate * (f.rrx - rcomx)

    # ── pitch acceleration controller (pole placement) ─────────────────────────

    def _normal_accel(self, ancomx, int_step):
        f    = self.f16
        aero = f.aerodynamics
        dvbe   = max(f.dvbe, 1.0)
        pdynmc = f.pdynmc

        dla  = aero.dla;  dma  = aero.dma
        dmq  = aero.dmq;  dmde = aero.dmde

        # dynamic gain scheduling with dynamic pressure
        waclp = 2.0 + 0.0001226 * (pdynmc - 5500.0)
        zaclp = 0.7 - 0.0000245 * (pdynmc - 5500.0)
        paclp = 5.0 + 0.0003067 * (pdynmc - 5500.0)

        # gains
        denom_ad = dla * dmde if abs(dla * dmde) > SMALL else SMALL
        gainfb3 = waclp**2 * paclp / denom_ad
        gainfb2 = (2.0*zaclp*waclp + paclp + dmq - dla/dvbe) / dmde
        gainfb1 = ((waclp**2 + 2.0*zaclp*waclp*paclp + dma + dmq*dla/dvbe
                    - gainfb2*dmde*dla/dvbe) / denom_ad) - self.gainp

        # integrator state
        fspb3    = f.FSPB[2]
        zzd_new  = AGRAV * ancomx + fspb3
        self.zz  = _integrate(zzd_new, self.zzd, self.zz, int_step)
        self.zzd = zzd_new

        dqc    = (-gainfb1*(-fspb3)
                  - gainfb2 * f.qqx * RAD
                  + gainfb3 * self.zz
                  + self.gainp * zzd_new)
        return dqc * DEG

    # ── lateral accel → roll angle ─────────────────────────────────────────────

    def _lateral_accel(self, alcomx):
        f     = self.f16
        fspb3 = f.FSPB[2]
        return -DEG * self.gainl * math.atan(alcomx) * _sign(fspb3)

    # ── flight-path-angle (gamma) hold — full state feedback ──────────────────

    def _gamma(self, thtvlcomx):
        f    = self.f16
        aero = f.aerodynamics
        dvbe = max(f.dvbe, 1.0)

        dla  = aero.dla;  dlde = aero.dlde
        dma  = aero.dma;  dmq  = aero.dmq;  dmde = aero.dmde

        V = dvbe
        AA = np.array([
            [dmq,    dma,    -dma   ],
            [1.,     0.,      0.    ],
            [0.,     dla/V,  -dla/V ],
        ])
        BB = np.array([dmde, 0., dlde/V])

        pgam = self.pgam;  wgam = self.wgam;  zgam = self.zgam
        am = 2.0*zgam*wgam + pgam
        bm = wgam**2 + 2.0*zgam*wgam*pgam
        cm = wgam**2 * pgam

        v21 = dmde*dla/V - dlde*dma/V
        DP  = np.array([
            [dmde,   0.,    dlde/V         ],
            [v21,    dmde, -dmq*dlde/V     ],
            [0.,     v21,   v21            ],
        ])
        DD = np.array([am + dmq - dla/V,
                       bm + dma + dmq*dla/V,
                       cm])

        try:
            GAINGAM = np.linalg.solve(DP, DD)
            DUM33   = AA - np.outer(BB, GAINGAM)
            DUM3    = np.linalg.solve(DUM33, BB)
            gainff  = -1.0 / DUM3[2] if abs(DUM3[2]) > SMALL else 0.0
        except np.linalg.LinAlgError:
            GAINGAM = np.zeros(3)
            gainff  = 0.0

        self.GAINGAM = GAINGAM
        self.gainff  = gainff

        thtc   = gainff * thtvlcomx * RAD
        delec  = (thtc
                  - GAINGAM[0]*f.qqx*RAD
                  - GAINGAM[1]*f.thtblx*RAD
                  - GAINGAM[2]*f.thtvlx*RAD)
        return delec * DEG

    # ── heading hold ───────────────────────────────────────────────────────────

    def _heading(self, psivlcomx):
        f    = self.f16
        dvbe = max(f.dvbe, 1.0)
        gainpsi = ((dvbe / f.grav)
                   * self.zrcl * self.wrcl
                   * (1.0 - self.zrcl**2)
                   * (1.0 + self.facthead))
        self.gainpsi = gainpsi
        # wrap heading error to [-180, 180] to avoid discontinuity at ±180°
        err = psivlcomx - f.psivlx
        err = (err + 180.0) % 360.0 - 180.0
        return gainpsi * err

    # ── altitude hold ──────────────────────────────────────────────────────────

    def _altitude(self, altcom):
        f       = self.f16
        altrate = -f.VBEL[2]               # positive = climbing
        eh      = self.gainalt * (altcom - f.hbe)
        # Clamp the desired climb/descent rate to a physically achievable range.
        # Without this, a large altitude step (e.g. 2500 m) drives eh to 175 m/s
        # and ancomx to 44 g, instantly departing the aircraft.
        # max_altrate ≈ 25 m/s ≈ 4900 ft/min — aggressive but achievable.
        eh = max(-self.max_altrate, min(self.max_altrate, eh))
        cphi    = math.cos(f.phiblx * RAD)
        if abs(cphi) < SMALL:
            cphi = SMALL
        return (1.0 / cphi) * (self.gainaltrate * (eh - altrate) + f.grav) / AGRAV
