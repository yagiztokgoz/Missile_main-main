import math
import numpy as np
import config

RAD   = math.pi / 180.0
DEG   = 180.0 / math.pi
AGRAV = config.AGRAV
SMALL = 1e-7


def _sign(x):
    return 1.0 if x >= 0.0 else -1.0


def _integrate(dydx_new, dydx, y, int_step):
    """Trapezoidal integration (CADAC standard)."""
    return y + (dydx_new + dydx) / 2.0 * int_step


class Control(object):

    def __init__(self, missile):
        self.missile = missile

        # diagnostics
        self.gkp    = 0.0    # Roll rate feedback gain - s
        self.gkphi  = 0.0    # Roll angle feedback gain
        self.zrate  = 0.0    # Zero pole of rate TF - 1/rad
        self.grate  = 0.0    # Feedback gain of rate loop - ND
        self.wnlagr = 0.0    # Nat freq of closed rate loop - rad/s
        self.GAINFB = np.zeros(3)  # Feedback gains [fb1, fb2, fb3]

        # online pitch/yaw nat freq computation (state variables)
        self.wclqd  = 0.0    # pitch freq derivative
        self.wclq   = 0.0    # pitch freq - rad/s
        self.wclrd  = 0.0    # yaw freq derivative
        self.wclr   = 0.0    # yaw freq - rad/s

        # feed-forward integrator states
        self.yyd    = 0.0    # yaw FF derivative - m/s
        self.yy     = 0.0    # yaw FF integral - m/s
        self.zzd    = 0.0    # pitch FF derivative - m/s
        self.zz     = 0.0    # pitch FF integral - m/s

        # computed closed-loop frequencies (diagnostics)
        self.waclq  = 0.0    # pitch closed-loop nat freq - rad/s
        self.waclr  = 0.0    # yaw closed-loop nat freq - rad/s

        # ── NDI diagnostics (tuning constants set inside control_ndi) ─────────
        self.wn_ndi_q = 0.0    # inner moment-loop bandwidth  - rad/s
        self.wn_ndi_r = 0.0    # inner moment-loop bandwidth  - rad/s
        self.k_alpha  = 0.0    # middle alpha-loop gain       - 1/s
        self.k_beta   = 0.0    # middle beta-loop gain        - 1/s

        # ── NDI-CoP diagnostics ───────────────────────────────────────────────
        self.x_cop_q  = 0.0    # pitch CoP position from CG (+ = nose) - m
        self.x_cop_r  = 0.0    # yaw   CoP position from CG (+ = nose) - m
        self.dna_cop  = 0.0    # effective normal force deriv at CoP    - m/s²/rad
        self.dyb_cop  = 0.0    # effective lateral force deriv at CoP   - m/s²/rad

        # ── INDI filter states (used by control_indi, maut==6) ────────────────
        # Matched first-order LPFs on measured q, r, q̇, ṙ and on measured fin
        # deflections so control input and angular-accel feedback stay phase-
        # aligned — this is what makes the incremental form valid.
        self.wn_indi_filt = config.WN_INDI_FILT   # filter bandwidth - rad/s
        self.q_filt       = 0.0
        self.r_filt       = 0.0
        self.q_dot_filt   = 0.0
        self.r_dot_filt   = 0.0
        self.dqx_filt1    = 0.0    # first-stage LPF on pitch fin - rad
        self.drx_filt1    = 0.0    # first-stage LPF on yaw fin   - rad
        self.dqx_filt     = 0.0    # second-stage LPF on pitch fin (matches q̇₀ delay) - rad
        self.drx_filt     = 0.0    # second-stage LPF on yaw fin   - rad


    # ── main dispatcher ────────────────────────────────────────────────────────

    def control(self, int_step):
        """Selects and calls control sub-modules."""
        m    = self.missile
        maut = m.maut

        if maut == 0:
            return

        self.control_roll()

        if maut == 2:
            self.control_rate()

        elif maut == 3:
            self.control_accel(int_step)

        elif maut == 4:
            self.control_rate()
            self.control_accel(int_step)

        elif maut == 5:
            self.control_ndi(int_step)

        elif maut == 6:
            self.control_indi(int_step)

        elif maut == 7:
            self.control_ndi_cop(int_step)


    # ── roll position controller ───────────────────────────────────────────────

    def control_roll(self):
        """Calculates roll gains and commanded roll fin deflection."""
        m = self.missile

        phicomx  = m.phicomx    # commanded roll angle - deg
        zrcl     = m.zrcl       # desired roll damping
        factwrcl = m.factwrcl   # roll bandwidth factor
        pdynmc   = m.pdynmc     # dynamic pressure - Pa
        phiblx   = m.phiblcx    # current bank angle (INS estimate) - deg
        dlp      = m.aerodynamics.dlp   # roll damping derivative - 1/s
        dld      = m.aerodynamics.dld   # roll control derivative - 1/s^2
        pp       = m.WBECB[0]           # roll rate - rad/s

        # variable roll bandwidth
        wrcl = (0.00024 * pdynmc + 10.0) * (1.0 + factwrcl)

        # gains
        gkp   = (2.0 * zrcl * wrcl + dlp) / dld
        gkphi = wrcl * wrcl / dld

        # roll position command
        ephi = gkphi * (phicomx - phiblx) * RAD
        dpc  = ephi - gkp * pp
        dpcx = dpc * DEG

        # store
        m.dpcx     = dpcx
        m.wrcl     = wrcl
        self.gkp   = gkp
        self.gkphi = gkphi


    # ── rate controller ────────────────────────────────────────────────────────

    def control_rate(self):
        """Calculates rate gyro feedback gain and pitch/yaw rate commands."""
        m    = self.missile
        aero = m.aerodynamics

        zetlagr = m.zetlagr
        dvbe    = m.dvbec       # INS-estimated speed

        dna = aero.dna;  dnd = aero.dnd
        dma = aero.dma;  dmq = aero.dmq;  dmd = aero.dmd
        qq  = m.WBECB[1]   # pitch rate - rad/s
        rr  = m.WBECB[2]   # yaw  rate  - rad/s

        # open-loop angular rate TF parameters
        zrate = dna / dvbe - dma * dnd / (dvbe * dmd + SMALL)
        aa    = dna / dvbe - dmq
        bb    = -dma - dmq * dna / dvbe

        # feedback gain for desired closed-loop damping
        dum1  = aa - 2.0 * zetlagr * zetlagr * zrate
        dum2  = aa * aa - 4.0 * zetlagr * zetlagr * bb
        radix = dum1 * dum1 - dum2
        if radix < 0.0:
            radix = 0.0

        dmd_safe = dmd if abs(dmd) > SMALL else SMALL * _sign(dmd)
        grate = -(-dum1 + math.sqrt(radix)) / dmd_safe

        # natural frequency of closed rate loop (diagnostic)
        dum3  = grate * dmd * zrate
        radix2 = bb + dum3
        wnlagr = math.sqrt(radix2) if radix2 > 0.0 else 0.0

        # commanded pitch and yaw deflections
        dqcx = DEG * grate * qq
        drcx = DEG * grate * rr

        # store outputs
        m.dqcx     = dqcx
        m.drcx     = drcx
        m.dqcx_rcs = dqcx
        m.drcx_rcs = drcx

        self.zrate  = zrate
        self.grate  = grate
        self.wnlagr = wnlagr


    # ── acceleration controller ────────────────────────────────────────────────

    def control_accel(self, int_step):
        """Pole-placement acceleration controller for pitch and yaw planes."""
        m    = self.missile
        aero = m.aerodynamics

        # input data
        zacl      = m.zacl
        pacl      = m.pacl
        alimit    = m.alimit
        gainp     = m.gainp
        factwacl  = m.factwacl
        twcl      = m.twcl
        wacl_bias = m.wacl_bias

        dvbe    = m.dvbec     # INS-estimated speed
        ancomx  = m.ancomx    # normal accel command (guidance) - g's
        alcomx  = m.alcomx    # lateral accel command (guidance) - g's

        # aerodynamic dimensional derivatives
        dna = aero.dna;  dnd = aero.dnd
        dma = aero.dma;  dmq = aero.dmq;  dmd = aero.dmd
        dlnr = aero.dlnr; dyb = aero.dyb
        dlnb = aero.dlnb; dlnd = aero.dlnd
        wnq  = aero.wnq;  wnr  = aero.wnr
        realq1 = aero.realq1; realq2 = aero.realq2
        realr1 = aero.realr1; realr2 = aero.realr2

        WBECB = m.WBECB     # [p, q, r] rad/s
        FSPCB = m.FSPCB     # specific force in body axes [x, y, z] m/s^2

        # ── structural acceleration limiter ────────────────────────────────────
        aa = math.sqrt(alcomx * alcomx + ancomx * ancomx)
        if aa > alimit:
            aa = alimit
        if abs(ancomx) < SMALL and abs(alcomx) < SMALL:
            phi = 0.0
        else:
            phi = math.atan2(ancomx, alcomx)
        alcomx = aa * math.cos(phi)
        ancomx = aa * math.sin(phi)

        # ── pitch loop ─────────────────────────────────────────────────────────
        # wacl tracks wnq through a first-order lag smoother
        wn = wnq if wnq > 0.0 else abs((realq1 + realq2) / 2.0)

        wclqd_new = (wn - self.wclq) / twcl
        self.wclq  = _integrate(wclqd_new, self.wclqd, self.wclq, int_step)
        self.wclqd = wclqd_new
        waclq = self.wclq * (factwacl + 1.0) + wacl_bias

        # pitch gains
        gainfb3_q = waclq * waclq * pacl / (dna * dmd + SMALL)
        gainfb2_q = (2.0 * zacl * waclq + pacl + dmq - dna / dvbe) / (dmd + SMALL)
        gainfb1_q = (waclq * waclq + 2.0 * zacl * waclq * pacl + dma
                     + dmq * dna / dvbe - gainfb2_q * dna * dmd / dvbe
                     ) / (dna * dmd + SMALL) - gainp

        # pitch feed-forward integrator
        qq      = WBECB[1]
        fspb3   = FSPCB[2]
        zzd_new = AGRAV * ancomx + fspb3
        self.zz  = _integrate(zzd_new, self.zzd, self.zz, int_step)
        self.zzd = zzd_new

        dqc  = -gainfb1_q * (-fspb3) - gainfb2_q * qq + gainfb3_q * self.zz + gainp * zzd_new
        dqcx = dqc * DEG

        # ── yaw loop ───────────────────────────────────────────────────────────
        wn = wnr if wnr > 0.0 else abs((realr1 + realr2) / 2.0)

        wclrd_new = (wn - self.wclr) / twcl
        self.wclr  = _integrate(wclrd_new, self.wclrd, self.wclr, int_step)
        self.wclrd = wclrd_new
        waclr = self.wclr * (factwacl + 1.0) + wacl_bias

        # yaw gains
        gainfb3_r = -waclr * waclr * pacl / (dyb * dlnd + SMALL)
        gainfb2_r = (2.0 * zacl * waclr + pacl + dlnr + dyb / dvbe) / (dlnd + SMALL)
        gainfb1_r = (-waclr * waclr - 2.0 * zacl * waclr * pacl + dlnb
                     + dlnr * dyb / dvbe - gainfb2_r * dyb * dlnd / dvbe
                     ) / (dyb * dlnd + SMALL) - gainp

        self.GAINFB = np.array([gainfb1_r, gainfb2_r, gainfb3_r])

        # yaw feed-forward integrator
        rr      = WBECB[2]
        fspb2   = FSPCB[1]
        yyd_new = AGRAV * alcomx - fspb2
        self.yy  = _integrate(yyd_new, self.yyd, self.yy, int_step)
        self.yyd = yyd_new

        drc  = -gainfb1_r * fspb2 - gainfb2_r * rr + gainfb3_r * self.yy + gainp * yyd_new
        drcx = drc * DEG

        # store outputs
        m.dqcx   = dqcx
        m.drcx   = drcx

        self.waclq = waclq
        self.waclr = waclr

    def control_ndi(self, int_step):
            """
            Nonlinear Dynamic Inversion style accel autopilot.
            Outer loop: accel error -> desired body rates
            Inner loop: invert q_dot / r_dot dynamics -> fin commands
            """
            m    = self.missile
            aero = m.aerodynamics

            # -------------------------------------------------
            # Inputs
            # -------------------------------------------------
            alimit = m.alimit
            ancomx = m.ancomx     # commanded normal accel [g]
            alcomx = m.alcomx     # commanded lateral accel [g]

            q = m.WBECB[1]        # pitch rate [rad/s]
            r = m.WBECB[2]        # yaw rate   [rad/s]

            # body specific force (CADAC convention)
            fspb2 = m.FSPCB[1]
            fspb3 = m.FSPCB[2]

            an_meas = -fspb3 / AGRAV  # Upward normal accel
            al_meas =  fspb2 / AGRAV  # Starboard lateral accel

            alpha = m.alphaxc * RAD   # INS-estimated α
            beta  = m.betaxc  * RAD   # INS-estimated β

            # dimensional derivatives
            dma  = aero.dma       # 1/s^2    pitch moment / α
            dmq  = aero.dmq       # 1/s      pitch damping / q
            dmd  = aero.dmd       # 1/s^2    pitch moment / δq

            dlnb = aero.dlnb      # 1/s^2    yaw moment / β
            dlnr = aero.dlnr      # 1/s      yaw damping / r
            dlnd = aero.dlnd      # 1/s^2    yaw moment / δr

            # force derivatives (for outer-outer force inversion)
            dna  = aero.dna       # m/s^2    body-Z specific force / α
            dyb  = aero.dyb       # m/s^2    body-Y specific force / β

            # -------------------------------------------------
            # Structural accel limit
            # -------------------------------------------------
            aa = math.sqrt(ancomx * ancomx + alcomx * alcomx)
            if aa > alimit:
                aa = alimit

            if abs(ancomx) < SMALL and abs(alcomx) < SMALL:
                phi = 0.0
            else:
                phi = math.atan2(ancomx, alcomx)

            alcomx = aa * math.cos(phi)
            ancomx = aa * math.sin(phi)

            # ── NDI tuning (config.WN_NDI_Q/R, K_ALPHA/K_BETA) ───────────────
            wn_ndi_q = config.WN_NDI_Q
            wn_ndi_r = config.WN_NDI_R
            k_alpha  = config.K_ALPHA
            k_beta   = config.K_BETA

            # -------------------------------------------------
            # STEP 1: FORCE INVERSION (accel -> alpha/beta command)
            # In CADAC sign convention (fspb3 = -q·S·cn/m, an = -fspb3/g):
            #   a_n ≈  dna·α/g  →  α_cmd =  g·ancomx/dna
            #   a_l ≈  dyb·β/g  →  β_cmd =  g·alcomx/dyb
            # -------------------------------------------------
            dna_safe = dna if abs(dna) > SMALL else SMALL * _sign(dna if dna != 0 else 1.0)
            dyb_safe = dyb if abs(dyb) > SMALL else SMALL * _sign(dyb if dyb != 0 else 1.0)

            alpha_cmd = AGRAV * ancomx / dna_safe
            beta_cmd  = AGRAV * alcomx / dyb_safe

            # incidence limit
            alp_lim = m.alplimx * RAD
            alpha_cmd = max(min(alpha_cmd, alp_lim), -alp_lim)
            beta_cmd  = max(min(beta_cmd,  alp_lim), -alp_lim)

            # -------------------------------------------------
            # STEP 2: KINEMATIC INVERSION (alpha/beta -> body rate)
            # α̇ = q + fspb3/V = q − g·a_n/V   →  q_cmd = K_α·(α_cmd−α) + g·a_n/V
            # β̇ = fspb2/V − r = g·a_l/V − r   →  r_cmd = K_β·(β−β_cmd) + g·a_l/V
            # -------------------------------------------------
            V_safe = max(m.dvbec, 30.0)   # INS-estimated speed

            q_cmd = k_alpha * (alpha_cmd - alpha) + AGRAV * an_meas / V_safe
            r_cmd = k_beta  * (beta - beta_cmd)   + AGRAV * al_meas / V_safe

            # Rate limiting (body rate cap from missile.py: m.wblimx in deg/s)
            rate_lim = m.wblimx * RAD
            q_cmd = max(min(q_cmd, rate_lim), -rate_lim)
            r_cmd = max(min(r_cmd, rate_lim), -rate_lim)

            # Inner Loop Desired Dynamics (First Order Error Decay)
            q_dot_des = wn_ndi_q * (q_cmd - q)
            r_dot_des = wn_ndi_r * (r_cmd - r)

            # -------------------------------------------------
            # INNER LOOP NDI (Dynamic Inversion)
            # q_dot = dma*alpha + dmq*q + dmd*delta_q
            # delta_q = (q_dot_des - (dma*alpha + dmq*q)) / dmd
            # -------------------------------------------------
            # Protect against divide-by-zero or loss of control authority
            dmd_safe  = dmd  if abs(dmd)  > SMALL else SMALL * _sign(dmd if dmd != 0 else -1.0)
            dlnd_safe = dlnd if abs(dlnd) > SMALL else SMALL * _sign(dlnd if dlnd != 0 else -1.0)

            # Natural airplane/missile dynamics (what the airframe wants to do)
            q_dot_model = dma * alpha + dmq * q
            r_dot_model = dlnb * beta + dlnr * r

            # Invert the dynamics to find required fin deflection (in radians)
            dqc = (q_dot_des - q_dot_model) / dmd_safe
            drc = (r_dot_des - r_dot_model) / dlnd_safe


            # -------------------------------------------------
            # Convert to deg and limit
            # -------------------------------------------------
            dqcx = dqc * DEG
            drcx = drc * DEG

            dqcx = max(min(dqcx, m.dqlimx), -m.dqlimx)
            drcx = max(min(drcx, m.drlimx), -m.drlimx)

            # -------------------------------------------------
            # Outputs
            # -------------------------------------------------
            m.dqcx = dqcx
            m.drcx = drcx

            # Diagnostics
            self.wn_ndi_q = wn_ndi_q
            self.wn_ndi_r = wn_ndi_r
            self.k_alpha  = k_alpha
            self.k_beta   = k_beta


    # ── INDI: Incremental Nonlinear Dynamic Inversion ──────────────────────────

    def control_indi(self, int_step):
        """
        Incremental Nonlinear Dynamic Inversion autopilot.

        Same outer/middle loops as NDI (force→α_cmd, kinematic→q_cmd), but the
        inner moment loop is INCREMENTAL:

            q̇ ≈ q̇₀ + dmd·(δ − δ₀)
            →  δ_cmd = δ₀ + (q̇_des − q̇₀) / dmd

        where q̇₀ and δ₀ are matched-filtered measurements. Removes
        dependence on dma, dmq, dlnb, dlnr — only control effectiveness
        (dmd, dlnd) is used, making it robust to aero-model mismatch and to
        noisy α/β estimates from the INS.
        """
        m    = self.missile
        aero = m.aerodynamics

        # ── inputs ─────────────────────────────────────────────────────────────
        alimit = m.alimit
        ancomx = m.ancomx
        alcomx = m.alcomx

        q = m.WBECB[1]
        r = m.WBECB[2]

        fspb2 = m.FSPCB[1]
        fspb3 = m.FSPCB[2]

        an_meas = -fspb3 / AGRAV
        al_meas =  fspb2 / AGRAV

        alpha = m.alphaxc * RAD
        beta  = m.betaxc  * RAD

        dmd  = aero.dmd
        dlnd = aero.dlnd
        dna  = aero.dna
        dyb  = aero.dyb

        # ── structural limit ───────────────────────────────────────────────────
        aa = math.sqrt(ancomx * ancomx + alcomx * alcomx)
        if aa > alimit:
            aa = alimit
        if abs(ancomx) < SMALL and abs(alcomx) < SMALL:
            phi = 0.0
        else:
            phi = math.atan2(ancomx, alcomx)
        alcomx = aa * math.cos(phi)
        ancomx = aa * math.sin(phi)

        # ── tuning (config.WN_NDI_Q/R, K_ALPHA/K_BETA) ─────────────────────────
        wn_ndi_q = config.WN_NDI_Q
        wn_ndi_r = config.WN_NDI_R
        k_alpha  = config.K_ALPHA
        k_beta   = config.K_BETA

        # ── force inversion (α/β command from accel command) ───────────────────
        dna_safe = dna if abs(dna) > SMALL else SMALL * _sign(dna if dna != 0 else 1.0)
        dyb_safe = dyb if abs(dyb) > SMALL else SMALL * _sign(dyb if dyb != 0 else 1.0)

        alpha_cmd = AGRAV * ancomx / dna_safe
        beta_cmd  = AGRAV * alcomx / dyb_safe

        alp_lim = m.alplimx * RAD
        alpha_cmd = max(min(alpha_cmd, alp_lim), -alp_lim)
        beta_cmd  = max(min(beta_cmd,  alp_lim), -alp_lim)

        # ── kinematic inversion (body-rate command from α/β command) ───────────
        V_safe = max(m.dvbec, 30.0)

        q_cmd = k_alpha * (alpha_cmd - alpha) + AGRAV * an_meas / V_safe
        r_cmd = k_beta  * (beta - beta_cmd)   + AGRAV * al_meas / V_safe

        rate_lim = m.wblimx * RAD
        q_cmd = max(min(q_cmd, rate_lim), -rate_lim)
        r_cmd = max(min(r_cmd, rate_lim), -rate_lim)

        # desired angular accel (first-order error decay → inner bandwidth)
        q_dot_des = wn_ndi_q * (q_cmd - q)
        r_dot_des = wn_ndi_r * (r_cmd - r)

        # ── matched first-order LPFs on q,r and on fin deflections ─────────────
        # Using one-pole Euler form: y += (dt/τ)·(u − y).  With τ = 1/ωf and
        # dt=1 ms, ωf=50 rad/s → dt/τ = 0.05, well below instability.
        a_f = int_step * self.wn_indi_filt
        if a_f > 1.0:
            a_f = 1.0

        q_prev_filt = self.q_filt
        r_prev_filt = self.r_filt
        self.q_filt += a_f * (q - self.q_filt)
        self.r_filt += a_f * (r - self.r_filt)

        # angular accel estimate: backward diff on the filtered rate, then
        # pass through the same filter to remove the differentiator's HF
        # content (standard INDI "washout" construction).
        q_dot_raw = (self.q_filt - q_prev_filt) / int_step
        r_dot_raw = (self.r_filt - r_prev_filt) / int_step
        self.q_dot_filt += a_f * (q_dot_raw - self.q_dot_filt)
        self.r_dot_filt += a_f * (r_dot_raw - self.r_dot_filt)

        # filtered measured fin deflection
        dqx_meas = m.dqx * RAD
        drx_meas = m.drx * RAD
        if config.INDI_TWO_STAGE:
            # two-stage LPF: group delay 2/ωf matches q̇₀ derivation (LPF→diff→LPF)
            self.dqx_filt1 += a_f * (dqx_meas       - self.dqx_filt1)
            self.drx_filt1 += a_f * (drx_meas        - self.drx_filt1)
            self.dqx_filt  += a_f * (self.dqx_filt1  - self.dqx_filt)
            self.drx_filt  += a_f * (self.drx_filt1  - self.drx_filt)
        else:
            # legacy single-stage LPF: group delay 1/ωf (phase mismatch with q̇₀)
            self.dqx_filt += a_f * (dqx_meas - self.dqx_filt)
            self.drx_filt += a_f * (drx_meas - self.drx_filt)

        # ── incremental inversion ──────────────────────────────────────────────
        dmd_safe  = dmd  if abs(dmd)  > SMALL else SMALL * _sign(dmd  if dmd  != 0 else -1.0)
        dlnd_safe = dlnd if abs(dlnd) > SMALL else SMALL * _sign(dlnd if dlnd != 0 else -1.0)

        dqc = self.dqx_filt + (q_dot_des - self.q_dot_filt) / dmd_safe
        drc = self.drx_filt + (r_dot_des - self.r_dot_filt) / dlnd_safe

        # ── limit and output ───────────────────────────────────────────────────
        dqcx = dqc * DEG
        drcx = drc * DEG
        dqcx = max(min(dqcx, m.dqlimx), -m.dqlimx)
        drcx = max(min(drcx, m.drlimx), -m.drlimx)

        m.dqcx = dqcx
        m.drcx = drcx

        # diagnostics
        self.wn_ndi_q = wn_ndi_q
        self.wn_ndi_r = wn_ndi_r
        self.k_alpha  = k_alpha
        self.k_beta   = k_beta


    # ── NDI-CoP: NDI reformulated about the Center of Percussion ──────────────

    def control_ndi_cop(self, int_step):
        """
        NDI autopilot reformulated about the Center of Percussion (CoP).

        The CoP is the point on the body-axis where a fin deflection produces
        zero instantaneous linear acceleration, thereby removing the RHP zero
        (non-minimum-phase behaviour) that exists in the CG-referenced
        transfer function δ → aₙ_CG for a tail-controlled missile.

        CoP position forward of CG (metres):
            x_cop = −dnd_rad / dmd        dnd_rad = dnd·DEG  [m/s²/rad]

        Effective normal-force derivative at CoP:
            dna_cop = dna + x_cop·dma

        Outer loop  : CoP accel command → α/β command  (uses dna_cop, dyb_cop)
        Middle loop : α/β command → q/r command  (CG kinematic feedforward — an_meas,
                      not an_cop: α̇ = q − g·aₙ_CG/V is a CG-referenced identity)
        Inner loop  : q/r command → fin command        (identical to NDI — MP)
        """
        m    = self.missile
        aero = m.aerodynamics

        # ── inputs ─────────────────────────────────────────────────────────────
        alimit = m.alimit
        ancomx = m.ancomx
        alcomx = m.alcomx

        q = m.WBECB[1]
        r = m.WBECB[2]

        fspb2 = m.FSPCB[1]
        fspb3 = m.FSPCB[2]

        an_meas = -fspb3 / AGRAV
        al_meas =  fspb2 / AGRAV

        alpha = m.alphaxc * RAD
        beta  = m.betaxc  * RAD

        dma  = aero.dma
        dmq  = aero.dmq
        dmd  = aero.dmd
        dnd  = aero.dnd
        dna  = aero.dna

        dlnb = aero.dlnb
        dlnr = aero.dlnr
        dlnd = aero.dlnd
        dyb  = aero.dyb

        # ── structural limit ───────────────────────────────────────────────────
        aa = math.sqrt(ancomx * ancomx + alcomx * alcomx)
        if aa > alimit:
            aa = alimit
        if abs(ancomx) < SMALL and abs(alcomx) < SMALL:
            phi = 0.0
        else:
            phi = math.atan2(ancomx, alcomx)
        alcomx = aa * math.cos(phi)
        ancomx = aa * math.sin(phi)

        # ── tuning ─────────────────────────────────────────────────────────────
        wn_ndi_q = config.WN_NDI_Q
        wn_ndi_r = config.WN_NDI_R
        k_alpha  = config.K_ALPHA
        k_beta   = config.K_BETA

        # ── Center of Percussion ───────────────────────────────────────────────
        # dnd [m/s²/deg] → convert to [m/s²/rad] for dimensional consistency
        dnd_rad = dnd * DEG

        dmd_safe  = dmd  if abs(dmd)  > SMALL else SMALL * _sign(dmd  if dmd  != 0 else -1.0)
        dlnd_safe = dlnd if abs(dlnd) > SMALL else SMALL * _sign(dlnd if dlnd != 0 else -1.0)

        # CoP position from CG (positive = forward/nose direction), clamped to
        # physical missile length for robustness near zero dynamic pressure.
        x_cop_q = max(min(-dnd_rad / dmd_safe,  3.0), -3.0)
        # yaw plane: cruciform symmetry → same fin normal force effectiveness
        x_cop_r = max(min(-dnd_rad / dlnd_safe, 3.0), -3.0)

        # Effective force derivatives at CoP (NMP zero cancelled by construction)
        dna_cop = dna + x_cop_q * dma
        dyb_cop = dyb + x_cop_r * dlnb

        # ── outer loop: force inversion at CoP ────────────────────────────────
        # α_cmd = g·ancomx / dna_cop  (NMP zero in δ→aₙ_CG cancelled at CoP)
        dna_cop_safe = dna_cop if abs(dna_cop) > SMALL else SMALL * _sign(dna_cop if dna_cop != 0 else 1.0)
        dyb_cop_safe = dyb_cop if abs(dyb_cop) > SMALL else SMALL * _sign(dyb_cop if dyb_cop != 0 else 1.0)

        alpha_cmd = AGRAV * ancomx / dna_cop_safe
        beta_cmd  = AGRAV * alcomx / dyb_cop_safe

        alp_lim = m.alplimx * RAD
        alpha_cmd = max(min(alpha_cmd, alp_lim), -alp_lim)
        beta_cmd  = max(min(beta_cmd,  alp_lim), -alp_lim)

        # ── middle loop: kinematic inversion ───────────────────────────────────
        # α̇ = q − (g/V)·aₙ_CG  →  kinematic identity is CG-referenced.
        # an_meas (CG) is the correct feedforward here, not an_cop.
        V_safe = max(m.dvbec, 30.0)

        q_cmd = k_alpha * (alpha_cmd - alpha) + AGRAV * an_meas / V_safe
        r_cmd = k_beta  * (beta - beta_cmd)   + AGRAV * al_meas / V_safe

        rate_lim = m.wblimx * RAD
        q_cmd = max(min(q_cmd, rate_lim), -rate_lim)
        r_cmd = max(min(r_cmd, rate_lim), -rate_lim)

        q_dot_des = wn_ndi_q * (q_cmd - q)
        r_dot_des = wn_ndi_r * (r_cmd - r)

        # ── inner loop: NDI moment inversion (identical to standard NDI) ───────
        q_dot_aero = dma * alpha + dmq * q
        r_dot_aero = dlnb * beta  + dlnr * r

        dqc = (q_dot_des - q_dot_aero) / dmd_safe
        drc = (r_dot_des - r_dot_aero) / dlnd_safe

        # ── limit and output ───────────────────────────────────────────────────
        dqcx = max(min(dqc * DEG, m.dqlimx), -m.dqlimx)
        drcx = max(min(drc * DEG, m.drlimx), -m.drlimx)

        m.dqcx = dqcx
        m.drcx = drcx

        # diagnostics
        self.wn_ndi_q = wn_ndi_q
        self.wn_ndi_r = wn_ndi_r
        self.k_alpha  = k_alpha
        self.k_beta   = k_beta
        self.x_cop_q  = x_cop_q
        self.x_cop_r  = x_cop_r
        self.dna_cop  = dna_cop
        self.dyb_cop  = dyb_cop
