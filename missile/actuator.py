import math


def _sign(x):
    """Returns +1 for positive, -1 for negative, 0 for zero."""
    if x > 0:
        return 1.0
    elif x < 0:
        return -1.0
    return 0.0


class Actuator(object):

    def __init__(self, missile):
        self.missile = missile

        # autopilot commands (inputs from control module)
        self.dpcx = 0.0   # Roll command - deg
        self.dqcx = 0.0   # Pitch command - deg
        self.drcx = 0.0   # Yaw command - deg

        # actual fin deflections (outputs to aerodynamics)
        self.dpx  = 0.0   # Roll fin deflection - deg
        self.dqx  = 0.0   # Pitch fin deflection - deg
        self.drx  = 0.0   # Yaw fin deflection - deg

        # individual fin deflections (diagnostics)
        self.delx1  = 0.0
        self.delx2  = 0.0
        self.delx3  = 0.0
        self.delx4  = 0.0

        # individual fin commands (diagnostics)
        self.delcx1 = 0.0
        self.delcx2 = 0.0
        self.delcx3 = 0.0
        self.delcx4 = 0.0

        # second order actuator states (for mact==2)
        self._delx1_d = 0.0
        self._delx2_d = 0.0
        self._delx3_d = 0.0
        self._delx4_d = 0.0
        self._delx1_dot = 0.0
        self._delx2_dot = 0.0
        self._delx3_dot = 0.0
        self._delx4_dot = 0.0


    def actuator(self, int_step):
        """Converts autopilot commands to fin deflections."""
        m     = self.missile
        mact  = m.mact
        dlimx = m.dlimx

        # read autopilot commands from missile (written by control module)
        dpcx = m.dpcx
        dqcx = m.dqcx
        drcx = m.drcx

        # convert to 4-fin deflection commands
        delcx1 = -dpcx + dqcx - drcx
        delcx2 = -dpcx + dqcx + drcx
        delcx3 = +dpcx + dqcx - drcx
        delcx4 = +dpcx + dqcx + drcx

        if mact < 2:
            # no actuator dynamics — apply limiter only
            delx1 = delcx1
            if abs(delx1) > dlimx:
                delx1 = dlimx * _sign(delx1)

            delx2 = delcx2
            if abs(delx2) > dlimx:
                delx2 = dlimx * _sign(delx2)

            delx3 = delcx3
            if abs(delx3) > dlimx:
                delx3 = dlimx * _sign(delx3)

            delx4 = delcx4
            if abs(delx4) > dlimx:
                delx4 = dlimx * _sign(delx4)

            # convert back to roll/pitch/yaw deflections
            dpx = (-delx1 - delx2 + delx3 + delx4) / 4.0
            dqx = (+delx1 + delx2 + delx3 + delx4) / 4.0
            drx = (-delx1 + delx2 - delx3 + delx4) / 4.0

            # store outputs
            self.dpx    = dpx
            self.dqx    = dqx
            self.drx    = drx
            self.delx1  = delx1
            self.delx2  = delx2
            self.delx3  = delx3
            self.delx4  = delx4
            self.delcx1 = delcx1
            self.delcx2 = delcx2
            self.delcx3 = delcx3
            self.delcx4 = delcx4

            # push to missile so aerodynamics can read them
            m.dpx = dpx
            m.dqx = dqx
            m.drx = drx

        elif mact == 2:
            # store commands for second order dynamics
            self.delcx1 = delcx1
            self.delcx2 = delcx2
            self.delcx3 = delcx3
            self.delcx4 = delcx4
            self.actuator_scnd(int_step)


    def actuator_scnd(self, int_step):
        """Second order actuator dynamics (Euler integration)."""
        m      = self.missile
        dlimx  = m.dlimx
        ddlimx = m.ddlimx
        wnact  = m.wnact
        zetact = m.zetact

        wn2 = wnact * wnact

        for i in range(1, 5):
            delcx = getattr(self, f'delcx{i}')
            delx  = getattr(self, f'_delx{i}_d')
            delxd = getattr(self, f'_delx{i}_dot')

            # second order ODE: ddot = wn^2*(cmd - x) - 2*zet*wn*xdot
            delxdd = wn2 * (delcx - delx) - 2.0 * zetact * wnact * delxd

            # Euler integration
            delxd_new = delxd + delxdd * int_step
            delx_new  = delx  + delxd  * int_step

            # rate limiter
            if abs(delxd_new) > ddlimx:
                delxd_new = ddlimx * _sign(delxd_new)

            # position limiter
            if abs(delx_new) > dlimx:
                delx_new = dlimx * _sign(delx_new)

            setattr(self, f'_delx{i}_d',   delx_new)
            setattr(self, f'_delx{i}_dot', delxd_new)
            setattr(self, f'delx{i}',      delx_new)

        delx1 = self.delx1
        delx2 = self.delx2
        delx3 = self.delx3
        delx4 = self.delx4

        # convert fin deflections back to roll/pitch/yaw
        dpx = (-delx1 - delx2 + delx3 + delx4) / 4.0
        dqx = (+delx1 + delx2 + delx3 + delx4) / 4.0
        drx = (-delx1 + delx2 - delx3 + delx4) / 4.0

        self.dpx = dpx
        self.dqx = dqx
        self.drx = drx

        # push to missile so aerodynamics can read them
        m.dpx = dpx
        m.dqx = dqx
        m.drx = drx
