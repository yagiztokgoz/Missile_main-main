PSL = 101325.0   # sea level pressure - Pa


class Propulsion(object):

    def __init__(self, missile):
        self.missile = missile

        # saved state (updated each step)
        self.pulse1_on    = False   # True while pulse1 still has fuel
        self.pulse1_epoch = 0.0     # time when pulse1 started - s
        self.pulse2_epoch = 0.0     # time when pulse2 started - s
        self.propflag1    = True    # False after pulse1 epoch is recorded
        self.propflag2    = True    # False after pulse2 epoch is recorded

        # freeze state (for autopilot response calculations)
        self.mfreeze_prop = 0
        self.thrustf      = 0.0
        self.massf        = 0.0
        self.xcgf         = 0.0
        self.ai11f        = 0.0
        self.ai33f        = 0.0


    def propulsion(self):
        """Calculates mass properties and rocket thrust at altitude.

        mprop modes:
            0 : no thrusting
            1 : 1st pulse / boost (single pulse motor)
            2 : 2nd pulse (lights only after pulse1 burns out)
            3 : constant thrust from input (thrust_input)
        """
        m  = self.missile
        db = m.db.prop_db

        # localize inputs
        mprop         = m.mprop
        aexit         = m.aexit
        thrust_input  = m.thrust_input
        launch_time   = m.launch_time
        press         = m.press         # atmospheric pressure at altitude - Pa
        mfreeze       = m.mfreeze

        # load saved mass properties (updated below if motor is firing)
        mass  = m.mass
        xcg   = m.xcg
        ai11  = m.ai11
        ai33  = m.ai33

        thrust = 0.0

        # ── pulse 1 / boost ──────────────────────────────────────────────────
        if mprop == 1 or self.pulse1_on:
            self.pulse1_on = True

            if self.propflag1:
                self.pulse1_epoch = launch_time
                self.propflag1    = False

            pulse1_time = launch_time - self.pulse1_epoch

            tsl = float(db['thrust_vs_time'](pulse1_time))

            if tsl != 0.0:
                thrust = tsl + (PSL - press) * aexit

            mass  = float(db['mass_vs_time'](pulse1_time))
            xcg   = float(db['cg_vs_time'](pulse1_time))
            ai33  = float(db['moipitch_vs_time'](pulse1_time))
            ai11  = float(db['moiroll_vs_time'](pulse1_time))

            # pulse1 exhausted when tsl drops to zero after ignition
            if tsl == 0.0 and pulse1_time > 0.0:
                self.pulse1_on = False
                mprop          = 0

        # ── pulse 2 / sustain ────────────────────────────────────────────────
        elif mprop == 2 and not self.pulse1_on:

            if self.propflag2:
                self.pulse2_epoch = launch_time
                self.propflag2    = False

            pulse2_time = launch_time - self.pulse2_epoch

            tsl    = float(db['thrust2_vs_time'](pulse2_time))
            thrust = tsl + (PSL - press) * aexit

            mass  = float(db['mass2_vs_time'](pulse2_time))
            xcg   = float(db['cg2_vs_time'](pulse2_time))
            ai33  = float(db['moipitch2_vs_time'](pulse2_time))
            ai11  = float(db['moiroll2_vs_time'](pulse2_time))

            if tsl == 0.0 and pulse2_time > 0.0:
                mprop = 0

        # ── constant thrust from input ────────────────────────────────────────
        elif mprop == 3:
            thrust = thrust_input

        # ── no thrust ─────────────────────────────────────────────────────────
        else:
            thrust = 0.0

        # ── freeze for autopilot response calculations ────────────────────────
        if mfreeze == 0:
            self.mfreeze_prop = 0
        else:
            if mfreeze != self.mfreeze_prop:
                self.mfreeze_prop = mfreeze
                self.thrustf      = thrust
                self.massf        = mass
                self.xcgf         = xcg
                self.ai11f        = ai11
                self.ai33f        = ai33

            thrust = self.thrustf
            mass   = self.massf
            xcg    = self.xcgf
            ai11   = self.ai11f
            ai33   = self.ai33f

        # ── push outputs to missile ───────────────────────────────────────────
        m.mprop  = mprop
        m.thrust = thrust
        m.mass   = mass
        m.xcg    = xcg
        m.ai11   = ai11
        m.ai33   = ai33
