"""
F-16 Aerodynamics module.

Ref: Stevens & Lewis, "Aircraft Control and Simulation", Wiley, 1992.
Data: Nguyen et al., NASA TP-1538, 1979.

Refa = 27.87 m², refb = 9.14 m, refc = 3.45 m
Reference CG = 0.35 * refc = 1.2075 m from nose.

Coefficient sign conventions (body axes, NED-like):
  CX — axial force  (+nose, i.e. drag is negative CX)
  CY — side force   (+starboard)
  CZ — normal force (+down, so lift is negative CZ)
  CL — rolling moment  (+right-wing-down)
  CM — pitching moment (+nose-up)
  CN — yawing moment   (+nose-right)

Notable sign correction in rolling moment:
  The raw clda table gives a sign that is valid for aileron (wing flex).
  At high subsonic the F-16 uses spoilers instead of ailerons, so the
  sign of clda is REVERSED (as in the original CADAC code).

Bug in original CADAC aerodynamics.cpp:
  'cllr' (roll due to yaw rate contribution in CLT) is initialised to 0
  and never assigned. The correct variable is 'clr' from the table lookup.
  This module uses 'clr' directly.

Double-counting fix (CADAC aerodynamics_der):
  The original code stores cllp = b2v*clp and clnr = b2v*cnr, then applies
  the b2v factor again in dimensional derivative computation, yielding b2v².
  This module applies the b2v factor exactly once in the dimensional step.
"""

import math
import numpy as np

RAD   = math.pi / 180.0
DEG   = 180.0 / math.pi
AGRAV = 9.80665
SMALL = 1e-7


class Aerodynamics:

    def __init__(self, f16):
        self.f16 = f16

        # reference geometry
        self.refa = 27.87    # m^2
        self.refb = 9.14     # m
        self.refc = 3.45     # m

        # CG locations (m from nose)
        self.xcg  = 0.35 * 3.45   # actual CG — may be updated externally
        self.xcgr = 0.35 * 3.45   # reference CG (fixed)

        # manoeuvre limits
        self.alplimpx = 21.0    # max positive alpha - deg
        self.alplimnx = -6.0    # min negative alpha - deg
        self.trbetx   = 5.0     # max absolute sideslip - deg

        # total force / moment coefficients (outputs)
        self.cxt = 0.0
        self.cyt = 0.0
        self.czt = 0.0
        self.clt = 0.0
        self.cmt = 0.0
        self.cnt = 0.0

        # diagnostics
        self.cdrag    = 0.0
        self.clift    = 0.0
        self.clovercd = 0.0
        self.gmax     = 0.0
        self.gminx    = 0.0
        self.stmarg   = 0.0   # static margin (+stable, -unstable) - caliber

        # dimensional derivatives (used by autopilot)
        self.dla   = 0.0   # lift / alpha             - m/s²/rad
        self.dlde  = 0.0   # lift / elevator          - m/s²/rad
        self.dma   = 0.0   # pitch moment / alpha     - 1/s²
        self.dmq   = 0.0   # pitch damping            - 1/s
        self.dmde  = 0.0   # pitch moment / elevator  - 1/s²
        self.dyb   = 0.0   # side force / beta        - m/s²/rad
        self.dydr  = 0.0   # side force / rudder      - m/s²/rad
        self.dnb   = 0.0   # yaw moment / beta        - 1/s²
        self.dnr   = 0.0   # yaw damping              - 1/s
        self.dndr  = 0.0   # yaw moment / rudder      - 1/s²
        self.dllp  = 0.0   # roll damping             - 1/s
        self.dllda = 0.0   # roll / aileron           - 1/s²

        # dynamic mode properties (diagnostics)
        self.wnp   = 0.0   # pitch natural frequency  - rad/s
        self.zetp  = 0.0   # pitch damping ratio
        self.wny   = 0.0   # yaw natural frequency    - rad/s
        self.zety  = 0.0   # yaw damping ratio

    # ── table lookup helpers ───────────────────────────────────────────────────

    def _look1(self, name, x):
        return float(self.f16.db.aero_db[name](x))

    def _look2(self, name, x1, x2):
        return float(self.f16.db.aero_db[name]([x1, x2])[0])

    def _look_cl(self, beta_deg, alpha_deg):
        """Rolling moment: odd in beta → cl(-β) = -cl(|β|)."""
        sign = 1.0 if beta_deg >= 0.0 else -1.0
        return sign * self._look2('cl_vs_beta_alpha', abs(beta_deg), alpha_deg)

    def _look_cn(self, beta_deg, alpha_deg):
        """Yawing moment: odd in beta → cn(-β) = -cn(|β|)."""
        sign = 1.0 if beta_deg >= 0.0 else -1.0
        return sign * self._look2('cn_vs_beta_alpha', abs(beta_deg), alpha_deg)

    # ── main aerodynamics ──────────────────────────────────────────────────────

    def aerodynamics(self):
        f      = self.f16
        alphax = f.alphax
        betax  = f.betax
        dvba   = max(f.dvba, 1.0)    # airspeed (wrt air) - m/s
        pdynmc = f.pdynmc
        ppx    = f.ppx
        qqx    = f.qqx
        rrx    = f.rrx
        delax  = f.delax
        delex  = f.delex
        delrx  = f.delrx
        xcg    = self.xcg
        xcgr   = self.xcgr

        refa = self.refa
        refb = self.refb
        refc = self.refc
        c2v  = refc / (2.0 * dvba)
        b2v  = refb / (2.0 * dvba)

        # ── axial force (CX) ──────────────────────────────────────────────────
        cx  = self._look2('cx_vs_elev_alpha', delex, alphax)
        cxq = self._look1('cxq_vs_alpha', alphax)
        cxt = cx + c2v * cxq * qqx * RAD

        # ── side force (CY) ───────────────────────────────────────────────────
        cyr = self._look1('cyr_vs_alpha', alphax)
        cyp = self._look1('cyp_vs_alpha', alphax)
        cyt = (-0.02 * betax
               + 0.021 * delax / 20.0
               + 0.086 * delrx / 30.0
               + b2v * (cyr * rrx * RAD + cyp * ppx * RAD))

        # ── normal (down) force (CZ) ──────────────────────────────────────────
        cz  = self._look1('cz_vs_alpha', alphax)
        czq = self._look1('czq_vs_alpha', alphax)
        czt = (cz * (1.0 - (betax * RAD) ** 2)
               - 0.19 * delex / 25.0
               + c2v * czq * qqx * RAD)

        # ── rolling moment (CL) ───────────────────────────────────────────────
        cl   = self._look_cl(betax, alphax)
        cldr = self._look2('cldr_vs_beta_alpha', betax, alphax)
        # sign reversed: F-16 uses spoilers at high subsonic, not ailerons
        clda = -self._look2('clda_vs_beta_alpha', betax, alphax)
        clr  = self._look1('clr_vs_alpha', alphax)    # roll due to yaw rate
        clp  = self._look1('clp_vs_alpha', alphax)    # roll damping
        clt  = (cl
                + clda * delax / 20.0
                + cldr * delrx / 30.0
                + b2v * (clr * rrx * RAD + clp * ppx * RAD))

        # ── pitching moment (CM) ──────────────────────────────────────────────
        cm  = self._look2('cm_vs_elev_alpha', delex, alphax)
        cmq = self._look1('cmq_vs_alpha', alphax)
        cmt = (cm
               + c2v * cmq * qqx * RAD
               + czt * (xcgr - xcg) / refc)

        # ── yawing moment (CN) ────────────────────────────────────────────────
        cn   = self._look_cn(betax, alphax)
        cnda = self._look2('cnda_vs_beta_alpha', betax, alphax)
        cndr = self._look2('cndr_vs_beta_alpha', betax, alphax)
        cnr  = self._look1('cnr_vs_alpha', alphax)    # yaw damping
        cnp  = self._look1('cnp_vs_alpha', alphax)    # yaw due to roll rate
        cnt  = (cn
                + cnda * delax / 20.0
                + cndr * delrx / 30.0
                - cyt * (xcgr - xcg) / refb
                + b2v * (cnr * rrx * RAD + cnp * ppx * RAD))

        # ── load factors ──────────────────────────────────────────────────────
        vmass = f.mass
        czp   = self._look1('cz_vs_alpha', self.alplimpx)
        czn   = self._look1('cz_vs_alpha', self.alplimnx)
        gmax  = -czp * pdynmc * refa / (vmass * AGRAV)
        gminx = -czn * pdynmc * refa / (vmass * AGRAV)

        # ── diagnostics ───────────────────────────────────────────────────────
        cosa  = math.cos(alphax * RAD)
        sina  = math.sin(alphax * RAD)
        cdrag = -cxt * cosa - czt * sina
        clift =  cxt * sina - czt * cosa
        clovercd = clift / cdrag if abs(cdrag) > SMALL else 0.0

        # store all coefficients
        self.cxt = cxt; self.cyt = cyt; self.czt = czt
        self.clt = clt; self.cmt = cmt; self.cnt = cnt
        self.cdrag = cdrag; self.clift = clift; self.clovercd = clovercd
        self.gmax  = gmax;  self.gminx = gminx

        # cache raw table values needed by _der
        self._cmq  = cmq
        self._clp  = clp
        self._clr  = clr
        self._clda = clda   # sign already reversed
        self._cldr = cldr
        self._cnr  = cnr
        self._cnp  = cnp
        self._cnda = cnda
        self._cndr = cndr

        self._der()

    # ── dimensional derivative calculation ─────────────────────────────────────

    def _der(self):
        """
        Computes dimensional stability and control derivatives for the autopilot.
        Called automatically at the end of aerodynamics().

        All derivatives are in SI (rad, m, s) unless noted.
        Finite-difference step of ±1.5° is used for alpha, beta, delex.
        """
        f      = self.f16
        alphax = f.alphax
        betax  = f.betax
        dvba   = max(f.dvba, 1.0)
        pdynmc = f.pdynmc
        delex  = f.delex
        xcg    = self.xcg
        xcgr   = self.xcgr

        refa = self.refa
        refb = self.refb
        refc = self.refc
        vmass = f.mass
        Ixx   = f.ai11
        Iyy   = f.ai22
        Izz   = f.ai33

        # ── nondimensional derivatives (finite difference, Δ = ±1.5°) ─────────

        # lift slope: cla = -d(cz)/d(alpha)  [per deg]
        czp = self._look1('cz_vs_alpha', alphax + 1.5)
        czn = self._look1('cz_vs_alpha', alphax - 1.5)
        cza = (czp - czn) / 3.0
        cla = -cza

        # pitch moment / alpha [per deg], corrected for actual vs reference CG
        cmp = self._look2('cm_vs_elev_alpha', delex, alphax + 1.5)
        cmn = self._look2('cm_vs_elev_alpha', delex, alphax - 1.5)
        cma = (cmp - cmn) / 3.0 + cza * (xcgr - xcg) / refc

        # pitch moment / elevator [per deg]
        cmp = self._look2('cm_vs_elev_alpha', delex + 1.5, alphax)
        cmn = self._look2('cm_vs_elev_alpha', delex - 1.5, alphax)
        cmde = (cmp - cmn) / 3.0

        # yaw moment / beta [per deg], corrected for CG
        cyb_rad = -0.02 * RAD     # dCY/dbeta [per rad]
        cnp = self._look_cn(betax + 1.5, alphax)
        cnn = self._look_cn(betax - 1.5, alphax)
        clnb = (cnp - cnn) / 3.0 - cyb_rad * (xcgr - xcg) / refb

        # fixed linear derivatives (per deg)
        clde_d  =  0.19 / 25.0    # dCZ/d(elev) → lift per deg
        cydr_d  = -0.086 / 30.0   # dCY/d(rudder) per deg (CADAC sign)

        # ── dimensional derivatives ────────────────────────────────────────────

        # pitch plane  [duml: (q·S/m)/RAD → m/s²/rad per unit nondim deriv]
        duml = (pdynmc * refa / vmass) / RAD
        dla  = duml * cla
        dlde = duml * clde_d

        dumm = pdynmc * refa * refc / Iyy
        dma  = dumm * cma / RAD
        dmq  = dumm * (refc / (2.0 * dvba)) * self._cmq   # cmq is per rad
        dmde = dumm * cmde / RAD

        # lateral plane
        # CADAC convention: cyb = -0.02 * RAD stored, then dyb = dumy * cyb / RAD
        # → dyb = dumy * (-0.02)  [m/s² per degree — beta used in deg throughout F-16]
        dumy = pdynmc * refa / vmass
        dyb  = dumy * (-0.02)
        dydr = dumy * (cydr_d / RAD)

        dumn = pdynmc * refa * refb / Izz
        dnb  = dumn * clnb / RAD
        dnr  = dumn * (refb / (2.0 * dvba)) * self._cnr   # cnr per rad
        dndr = dumn * (self._cndr / 30.0) / RAD            # cndr per frac → per deg → per rad

        # roll
        dumll = pdynmc * refa * refb / Ixx
        dllp  = dumll * (refb / (2.0 * dvba)) * self._clp  # clp per rad
        dllda = dumll * (self._clda / 20.0) / RAD           # clda per frac → per deg → per rad

        # static margin (+stable when cma/cla < 0)
        stmarg = -cma / cla if abs(cla) > SMALL else 0.0

        # ── pitch plane dynamic modes ─────────────────────────────────────────
        wnp, zetp = 0.0, 0.0
        if abs(dla) > SMALL:
            a11 = dmq
            a12 = dma / dla
            a21 = dla
            a22 = -dla / dvba
            arg = (a11 + a22) ** 2 - 4.0 * (a11 * a22 - a12 * a21)
            if arg < 0.0:
                wnp_sq = a11 * a22 - a12 * a21
                wnp  = math.sqrt(max(wnp_sq, 0.0))
                zetp = -(a11 + a22) / (2.0 * wnp) if wnp > SMALL else 0.0

        # ── yaw plane dynamic modes ───────────────────────────────────────────
        wny, zety = 0.0, 0.0
        if abs(dyb) > SMALL:
            a11 = dnr
            a12 = dnb / dyb
            a21 = -dyb
            a22 = dyb / dvba
            arg = (a11 + a22) ** 2 - 4.0 * (a11 * a22 - a12 * a21)
            if arg < 0.0:
                wny_sq = a11 * a22 - a12 * a21
                wny  = math.sqrt(max(wny_sq, 0.0))
                zety = -(a11 + a22) / (2.0 * wny) if wny > SMALL else 0.0

        # store
        self.dla   = dla;   self.dlde = dlde
        self.dma   = dma;   self.dmq  = dmq;  self.dmde = dmde
        self.dyb   = dyb;   self.dydr = dydr
        self.dnb   = dnb;   self.dnr  = dnr;  self.dndr = dndr
        self.dllp  = dllp;  self.dllda = dllda
        self.stmarg = stmarg
        self.wnp   = wnp;   self.zetp = zetp
        self.wny   = wny;   self.zety = zety
