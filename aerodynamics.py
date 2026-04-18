import math

RAD   = math.pi / 180.0   # deg  -> rad
DEG   = 180.0 / math.pi   # rad  -> deg
AGRAV = 9.80665            # m/s^2


class Aerodynamics(object):

    def __init__(self, missile):
        self.missile = missile

        # termination conditions (init_aerodynamics)
        self.trcond  = 0
        self.trortho = 1e-3
        self.tralp   = 1.047   # max total alpha - rad (~60 deg)
        self.trdynm  = 1e+4    # min dynamic pressure - Pa
        self.trload  = 3.0     # min load capacity - g's

        # 6-axis aerodynamic coefficient outputs
        self.ca   = 0.0    # Axial force coefficient - ND
        self.cy   = 0.0    # Side force coefficient - ND
        self.cn   = 0.0    # Normal force coefficient - ND
        self.cll  = 0.0    # Rolling moment coefficient - ND
        self.clm  = 0.0    # Pitching moment coefficient - ND
        self.cln  = 0.0    # Yawing moment coefficient - ND

        # ND derivative diagnostics
        self.cyb   = 0.0   # Yaw force derivative - 1/deg
        self.clnb  = 0.0   # Yaw moment derivative - 1/deg
        self.clnr  = 0.0   # Yaw damping derivative - 1/deg
        self.clndr = 0.0   # Yaw control derivative - 1/deg
        self.ca0   = 0.0   # Axial force coeff at zero incidence
        self.caa   = 0.0   # Axial force coeff of induced drag - ND
        self.cad   = 0.0   # Axial force coeff of control surface drag - 1/deg^2
        self.cndq  = 0.0   # Normal force coeff of pitch control deflection - 1/deg
        self.clmdq = 0.0   # Pitch control derivative - 1/deg
        self.clmq  = 0.0   # Pitching damping derivative - 1/deg
        self.clldp = 0.0   # Roll control derivative - 1/deg
        self.cllp  = 0.0   # Roll damping derivative - 1/deg
        self.cna   = 0.0   # Normal force derivative - 1/deg
        self.clma  = 0.0   # Pitch moment derivative - 1/deg
        self.gmax  = 0.0   # Max maneuverability limited by alplimx - g's
        self.gavail= 0.0   # Maneuver headroom - g's

        # static margins
        self.stmarg_pitch = 0.0   # Static margin in pitch (+ stable) - diameter
        self.stmarg_yaw   = 0.0   # Static margin in yaw  (+ stable) - diameter

        # dimensional derivatives - pitch
        self.dna  = 0.0    # Normal force derivative - m/s^2
        self.dnd  = 0.0    # Pitch control force derivative - m/s^2
        self.dma  = 0.0    # Pitch moment derivative - 1/s^2
        self.dmq  = 0.0    # Pitch damping derivative - 1/s
        self.dmd  = 0.0    # Pitch moment control derivative - 1/s^2

        # dimensional derivatives - roll
        self.dlp  = 0.0    # Roll damping derivative - 1/s
        self.dld  = 0.0    # Roll control derivative - 1/s^2

        # dimensional derivatives - yaw
        self.dlnr = 0.0    # Yaw damping derivative - 1/s
        self.dyb  = 0.0    # Yaw force derivative - m/s^2
        self.dlnd = 0.0    # Yaw moment control derivative - 1/s^2
        self.dlnb = 0.0    # Yaw moment derivative - 1/s^2

        # airframe dynamic roots - pitch
        self.realq1 = 0.0
        self.realq2 = 0.0
        self.wnq    = 0.0  # Natural frequency in pitch - rad/s
        self.zetq   = 0.0  # Damping in pitch - ND
        self.pqreal = 0.0

        # airframe dynamic roots - yaw
        self.wnr    = 0.0  # Natural frequency in yaw - rad/s
        self.zetr   = 0.0  # Damping in yaw - ND
        self.realr1 = 0.0
        self.realr2 = 0.0
        self.prreal = 0.0

        # airframe dynamic roots - roll
        self.realp  = 0.0  # Real root of roll dynamics - rad/s


    def aerodynamics(self):
        """Calculates aerodynamic coefficients from look-up tables."""
        m  = self.missile
        db = m.db.aero_db

        # localize inputs
        alplimx = m.alplimx
        refl    = m.refl
        refa    = m.refa
        mach    = m.mach
        pdynmc  = m.pdynmc
        alphax  = m.alphax
        betax   = m.betax
        dvbe    = m.dvbe
        ppx     = m.ppx
        qqx     = m.qqx
        rrx     = m.rrx
        mprop   = m.mprop
        mass    = m.mass
        xcgref  = m.xcgref
        xcg     = m.xcg
        alimit  = m.alimit
        dpx     = m.dpx
        dqx     = m.dqx
        drx     = m.drx

        # effective fin deflection for drag (average of pitch and yaw fins)
        deff = (abs(dqx) + abs(drx)) / 2.0

        # --- axial force ---
        ca0 = float(db['ca0_vs_mach'](mach))
        caa = float(db['caa_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        cad = float(db['cad_vs_mach'](mach))
        ca  = ca0 + caa + cad * deff * deff

        # base drag reduction when motor is on
        ca0b = 0.0
        if mprop:
            ca0b = float(db['ca0b_vs_mach'](mach))
            ca  += ca0b

        # --- side force ---
        cydr = float(db['cydr_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        cy0  = float(db['cy0_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        cy   = cy0 + cydr * drx

        # --- normal force ---
        cn0  = float(db['cn0_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        cndq = float(db['cndq_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        cn   = cn0 + cndq * dqx

        # --- rolling moment ---
        cllp  = float(db['cllp_vs_mach'](mach)) * RAD
        cll0  = float(db['cll0_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        clldp = float(db['clldp_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        cll   = cllp * ppx * refl / (2.0 * dvbe) + cll0 + clldp * dpx

        # --- pitching moment ---
        clm0  = float(db['clm0_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        clmq  = float(db['clmq_vs_mach'](mach)) * RAD
        clmdq = float(db['clmdq_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        clm   = clm0 + clmq * qqx * refl / (2.0 * dvbe) + clmdq * dqx - cn / refl * (xcgref - xcg)

        # --- yawing moment ---
        cln0  = float(db['cln0_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        clnr  = float(db['clnr_vs_mach'](mach)) * RAD
        clndr = float(db['clndr_vs_mach_alpha_beta']([mach, alphax, betax])[0])
        cln   = cln0 + clnr * rrx * refl / (2.0 * dvbe) + clndr * drx - cy / refl * (xcgref - xcg)

        # --- load factor available ---
        cn0mx  = float(db['cn0_vs_mach_alpha_beta']([mach, alplimx, 0.0])[0])
        almx   = cn0mx * pdynmc * refa
        weight = mass * AGRAV
        gmax   = almx / weight
        if gmax >= alimit:
            gmax = alimit

        aload  = math.sqrt(cn0 * cn0 + cy0 * cy0) * pdynmc * refa
        gg     = aload / weight
        gavail = gmax - gg
        if gavail > alimit:
            gavail = alimit
        if gavail < 0.0:
            gavail = 0.0

        # run-termination condition
        if gmax < self.trload:
            self.trcond = 4

        # store outputs
        self.ca    = ca
        self.cy    = cy
        self.cn    = cn
        self.cll   = cll
        self.clm   = clm
        self.cln   = cln
        self.gmax  = gmax
        self.gavail= gavail

        # store diagnostics
        self.ca0   = ca0
        self.caa   = caa
        self.cad   = cad
        self.cndq  = cndq
        self.clmdq = clmdq
        self.clmq  = clmq
        self.clldp = clldp
        self.cllp  = cllp
        self.clnr  = clnr
        self.clndr = clndr

        # call derivative calculations
        self.aerodynamics_der()


    def aerodynamics_der(self):
        """On-line calculation of aero derivatives for the aero-adaptive autopilot.
        (1) Non-dimensional derivatives
        (2) Dimensional derivatives (rad, m, s)
        (3) Airframe dynamic roots
        """
        m  = self.missile
        db = m.db.aero_db

        # localize inputs
        alplimx = m.alplimx
        refl    = m.refl
        refa    = m.refa
        mach    = m.mach
        pdynmc  = m.pdynmc
        alphax  = m.alphax
        betax   = m.betax
        alppx   = m.alppx
        dvbe    = m.dvbe
        mass    = m.mass
        xcgref  = m.xcgref
        xcg     = m.xcg
        ai11    = m.ai11
        ai33    = m.ai33
        thrust  = float(m.db.prop_db['thrust_vs_time'](m.time))
        mtvc    = m.mtvc
        parm    = m.parm
        gtvc    = m.gtvc

        # from aerodynamics() outputs (already computed this step)
        clnr  = self.clnr
        clndr = self.clndr
        cndq  = self.cndq
        clmdq = self.clmdq
        clmq  = self.clmq
        cllp  = self.cllp
        clldp = self.clldp

        # saved values — kept from previous step if alppx >= alplimx-3
        cyb         = self.cyb
        clnb        = self.clnb
        cna         = self.cna
        clma        = self.clma
        stmarg_pitch= self.stmarg_pitch
        stmarg_yaw  = self.stmarg_yaw

        # compute ND derivatives only while within alpha limits
        if alppx < (alplimx - 3.0):

            # pitch: ±3 deg perturbation, clamped to table bounds
            alpha = abs(alphax)
            alpp  = max(alpha + 3.0, 3.0)
            alpm  = max(alpha - 3.0, 0.0)

            cn0p  = float(db['cn0_vs_mach_alpha_beta']([mach, alpp, betax])[0])
            cn0m  = float(db['cn0_vs_mach_alpha_beta']([mach, alpm, betax])[0])
            cna   = DEG * (cn0p - cn0m) / (alpp - alpm)

            clm0p = float(db['clm0_vs_mach_alpha_beta']([mach, alpp, betax])[0])
            clm0m = float(db['clm0_vs_mach_alpha_beta']([mach, alpm, betax])[0])
            clma  = DEG * (clm0p - clm0m) / (alpp - alpm) - cna / refl * (xcgref - xcg)

            # yaw: ±3 deg perturbation, clamped to table bounds
            beta = abs(betax)
            betp = max(beta + 3.0, 3.0)
            betm = max(beta - 3.0, 0.0)

            cy0p  = float(db['cy0_vs_mach_alpha_beta']([mach, alphax, betp])[0])
            cy0m  = float(db['cy0_vs_mach_alpha_beta']([mach, alphax, betm])[0])
            cyb   = DEG * (cy0p - cy0m) / (betp - betm)

            cln0p = float(db['cln0_vs_mach_alpha_beta']([mach, alphax, betp])[0])
            cln0m = float(db['cln0_vs_mach_alpha_beta']([mach, alphax, betm])[0])
            clnb  = DEG * (cln0p - cln0m) / (betp - betm) - cyb / refl * (xcgref - xcg)

        # dimensional derivatives - pitch
        dna = (pdynmc * refa / mass) * cna
        dma = (pdynmc * refa * refl / ai33) * clma
        dmq = DEG * (pdynmc * refa * refl / ai33) * (refl / (2.0 * dvbe)) * clmq
        dmd = DEG * (pdynmc * refa * refl / ai33) * clmdq
        dnd = (pdynmc * refa / mass) * cndq

        # dimensional derivatives - yaw
        dyb  = (pdynmc * refa / mass) * cyb
        dlnb = (pdynmc * refa * refl / ai33) * clnb
        dlnr = DEG * (pdynmc * refa * refl / ai33) * (refl / (2.0 * dvbe)) * clnr
        dlnd = DEG * (pdynmc * refa * refl / ai33) * clndr

        # dimensional derivatives - roll
        dlp = DEG * (pdynmc * refa * refl / ai11) * (refl / (2.0 * dvbe)) * cllp
        dld = DEG * (pdynmc * refa * refl / ai11) * clldp

        # TVC augmented derivatives
        if mtvc > 0:
            dmz   = -(parm - xcg) * thrust / ai33
            dmd  += dmz * gtvc
            dlnd += dmz * gtvc

        # Monte Carlo: apply multiplicative perturbation to controller's model
        # (physical forces in aerodynamics() are unperturbed — this simulates
        # aero model mismatch between truth and what the controller "knows")
        if m.aero_pert:
            p = m.aero_pert
            dna  *= p.get('dna',  1.0);  dma  *= p.get('dma',  1.0)
            dmq  *= p.get('dmq',  1.0);  dmd  *= p.get('dmd',  1.0)
            dnd  *= p.get('dnd',  1.0);  dyb  *= p.get('dyb',  1.0)
            dlnb *= p.get('dlnb', 1.0);  dlnr *= p.get('dlnr', 1.0)
            dlnd *= p.get('dlnd', 1.0);  dlp  *= p.get('dlp',  1.0)
            dld  *= p.get('dld',  1.0)

        # static margins
        if dna != 0.0:
            stmarg_pitch = -(dma / dna) * (ai33 / (refl * mass))
        if dyb != 0.0:
            stmarg_yaw   = -(dlnb / dyb) * (ai33 / (refl * mass))

        # airframe dynamic roots - pitch
        a11 = dmq
        a12 = dma / dna if dna != 0.0 else 0.0
        a21 = dna
        a22 = -dna / dvbe if dvbe != 0.0 else 0.0

        arg = (a11 + a22) ** 2 - 4.0 * (a11 * a22 - a12 * a21)
        if arg >= 0.0:
            wnq, zetq = 0.0, 0.0
            dum    = a11 + a22
            realq1 = (dum + math.sqrt(arg)) / 2.0
            realq2 = (dum - math.sqrt(arg)) / 2.0
            pqreal = (realq1 + realq2) / 2.0
        else:
            realq1, realq2 = 0.0, 0.0
            wnq    = math.sqrt(a11 * a22 - a12 * a21)
            zetq   = -(a11 + a22) / (2.0 * wnq)
            pqreal = -zetq * wnq

        # airframe dynamic roots - yaw
        a11 = dlnr
        a12 = dlnb / dyb if dyb != 0.0 else 0.0
        a21 = -dyb
        a22 =  dyb / dvbe if dvbe != 0.0 else 0.0

        arg = (a11 + a22) ** 2 - 4.0 * (a11 * a22 - a12 * a21)
        if arg >= 0.0:
            wnr, zetr = 0.0, 0.0
            dum    = a11 + a22
            realr1 = (dum + math.sqrt(arg)) / 2.0
            realr2 = (dum - math.sqrt(arg)) / 2.0
            prreal = (realr1 + realr2) / 2.0
        else:
            realr1, realr2 = 0.0, 0.0
            wnr    = math.sqrt(a11 * a22 - a12 * a21)
            zetr   = -(a11 + a22) / (2.0 * wnr)
            prreal = -zetr * wnr

        # roll rate root
        realp = dlp

        # store dimensional derivative outputs
        self.dna  = dna
        self.dnd  = dnd
        self.dma  = dma
        self.dmq  = dmq
        self.dmd  = dmd
        self.dlp  = dlp
        self.dld  = dld
        self.dlnd = dlnd
        self.dlnr = dlnr
        self.dyb  = dyb
        self.dlnb = dlnb

        # store (saved) ND derivatives
        self.cyb  = cyb
        self.clnb = clnb
        self.cna  = cna
        self.clma = clma

        # store static margins
        self.stmarg_pitch = stmarg_pitch
        self.stmarg_yaw   = stmarg_yaw

        # store dynamic roots - pitch
        self.realq1 = realq1
        self.realq2 = realq2
        self.wnq    = wnq
        self.zetq   = zetq
        self.pqreal = pqreal
        self.realp  = realp

        # store dynamic roots - yaw
        self.wnr    = wnr
        self.zetr   = zetr
        self.realr1 = realr1
        self.realr2 = realr2
        self.prreal = prreal
