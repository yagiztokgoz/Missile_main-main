import numpy as np


class Forces(object):

    def __init__(self, missile):
        self.missile = missile

        # outputs - forces and moments in body axes
        self.FAPB = np.zeros(3)   # Aerodynamic + propulsive forces - N  [x, y, z]
        self.FMB  = np.zeros(3)   # Aerodynamic + propulsive moments - N*m [l, m, n]


    def forces(self):
        """Calculates non-gravitational forces and aerodynamic moments in body axes."""
        m    = self.missile
        aero = m.aerodynamics

        pdynmc = m.pdynmc
        refa   = m.refa
        refl   = m.refl
        thrust = m.thrust
        mtvc   = m.mtvc

        ca  = aero.ca
        cy  = aero.cy
        cn  = aero.cn
        cll = aero.cll
        clm = aero.clm
        cln = aero.cln

        # aerodynamic forces in body axes
        FAPB = np.array([
            -pdynmc * refa * ca,    # x: axial (drag opposes forward motion)
             pdynmc * refa * cy,    # y: side
            -pdynmc * refa * cn,    # z: normal
        ])

        # add thrust
        if mtvc == 0:
            # no TVC: thrust acts along body x-axis
            FAPB[0] += thrust
        else:
            # TVC: thrust direction modulated by nozzle
            FAPB += m.FPB

        # add RCS forces if present
        FAPB += m.FARCS

        # aerodynamic moments in body axes
        FMB = np.array([
            pdynmc * refa * refl * cll,   # roll
            pdynmc * refa * refl * clm,   # pitch
            pdynmc * refa * refl * cln,   # yaw
        ])

        # add TVC moments if active
        if mtvc != 0:
            FMB += m.FMPB

        # add RCS moments if active
        FMB += m.FMRCS

        # store outputs
        self.FAPB = FAPB
        self.FMB  = FMB

        # push to missile for newton/euler modules
        m.FAPB = FAPB
        m.FMB  = FMB
