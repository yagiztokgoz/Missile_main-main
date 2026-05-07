DEG = 180.0 / 3.141592653589793


def _integrate(dydx_new, dydx, y, int_step):
    """Trapezoidal integration (CADAC standard)."""
    return y + (dydx_new + dydx) / 2.0 * int_step


class Euler(object):

    def __init__(self, missile):
        self.missile = missile

        # state variables and their derivatives (all start at zero)
        self.pp  = 0.0;  self.ppd = 0.0   # roll  rate - rad/s
        self.qq  = 0.0;  self.qqd = 0.0   # pitch rate - rad/s
        self.rr  = 0.0;  self.rrd = 0.0   # yaw   rate - rad/s


    def euler(self, int_step):
        """Rotational equations of motion for an axisymmetric missile.

        Euler equations (I2 = I3 = ai33, I1 = ai11):
            ppd = fmb1 / ai11
            qqd = ((ai33 - ai11)*pp*rr + fmb2) / ai33
            rrd = (-(ai33 - ai11)*pp*qq + fmb3) / ai33

        Reads from missile:
            FMB   - total moments in body axes [roll, pitch, yaw] - N*m
            ai11  - roll moment of inertia  - kg*m^2
            ai33  - pitch/yaw moment of inertia - kg*m^2

        Writes to missile:
            WBECB - angular velocity [p, q, r] - rad/s
            ppx, qqx, rrx - angular rates - deg/s
        """
        m    = self.missile
        FMB  = m.FMB
        ai11 = m.ai11
        ai33 = m.ai33

        fmb1, fmb2, fmb3 = float(FMB[0]), float(FMB[1]), float(FMB[2])

        # ── Euler rotational ODEs ────────────────────────────────────────────
        ppd_new = fmb1 / ai11
        qqd_new = ((ai33 - ai11) * self.pp * self.rr + fmb2) / ai33
        rrd_new = (-(ai33 - ai11) * self.pp * self.qq + fmb3) / ai33

        self.pp = _integrate(ppd_new, self.ppd, self.pp, int_step)
        self.qq = _integrate(qqd_new, self.qqd, self.qq, int_step)
        self.rr = _integrate(rrd_new, self.rrd, self.rr, int_step)

        self.ppd = ppd_new
        self.qqd = qqd_new
        self.rrd = rrd_new

        # ── push outputs ─────────────────────────────────────────────────────
        # truth channel — consumed only by INS
        m.WBEB[0] = self.pp
        m.WBEB[1] = self.qq
        m.WBEB[2] = self.rr

        # estimate channel — passthrough; INS overwrites when mins=1
        m.WBECB[0] = self.pp
        m.WBECB[1] = self.qq
        m.WBECB[2] = self.rr

        m.ppx = self.pp * DEG
        m.qqx = self.qq * DEG
        m.rrx = self.rr * DEG
