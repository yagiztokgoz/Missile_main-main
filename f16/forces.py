import numpy as np


class Forces:

    def __init__(self, f16):
        self.f16 = f16
        self.FAPB = np.zeros(3)   # aero + propulsive forces in body axes - N
        self.FMB  = np.zeros(3)   # aero + propulsive moments in body axes - N·m

    def forces(self):
        f      = self.f16
        aero   = f.aerodynamics
        q      = f.pdynmc
        S      = aero.refa
        b      = aero.refb
        c      = aero.refc

        FAPB = np.array([
            q * S * aero.cxt + f.thrust,
            q * S * aero.cyt,
            q * S * aero.czt,
        ])

        FMB = np.array([
            q * S * b * aero.clt,
            q * S * c * aero.cmt,
            q * S * b * aero.cnt,
        ])

        self.FAPB = FAPB
        self.FMB  = FMB
        f.FAPB    = FAPB
        f.FMB     = FMB
