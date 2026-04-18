import math

# Physical constants
REARTH     = 6.3781e6    # Earth mean radius - m
G_CONST    = 6.674e-11   # Gravitational constant - m^3/(kg*s^2)
EARTH_MASS = 5.9722e24   # Earth mass - kg
R_AIR      = 287.058     # Specific gas constant for air - J/(kg*K)
G0         = 9.80665     # Standard gravity - m/s^2


def atmosphere76(hbe):
    """US 1976 Standard Atmosphere.

    Args:
        hbe: geometric altitude above MSL - m  (clamped to >= 0)

    Returns:
        (rho, press, tempk): density kg/m^3, pressure Pa, temperature K
    """
    h = max(hbe, 0.0)

    if h <= 11000.0:
        # Troposphere: lapse rate -6.5 K/km
        T = 288.15 - 6.5e-3 * h
        P = 101325.0 * (T / 288.15) ** 5.25588

    elif h <= 20000.0:
        # Lower stratosphere: isothermal 216.65 K
        T = 216.65
        P = 22632.1 * math.exp(-G0 / (R_AIR * T) * (h - 11000.0))

    elif h <= 32000.0:
        # Upper stratosphere: lapse rate +1.0 K/km
        T = 216.65 + 1.0e-3 * (h - 20000.0)
        P = 5474.89 * (T / 216.65) ** (-G0 / (R_AIR * 1.0e-3))

    elif h <= 47000.0:
        # Stratopause region: lapse rate +2.8 K/km
        T = 228.65 + 2.8e-3 * (h - 32000.0)
        P = 868.019 * (T / 228.65) ** (-G0 / (R_AIR * 2.8e-3))

    elif h <= 51000.0:
        # Lower mesosphere: isothermal 270.65 K
        T = 270.65
        P = 110.906 * math.exp(-G0 / (R_AIR * T) * (h - 47000.0))

    elif h <= 71000.0:
        # Middle mesosphere: lapse rate -2.8 K/km
        T = 270.65 - 2.8e-3 * (h - 51000.0)
        P = 66.9389 * (T / 270.65) ** (G0 / (R_AIR * 2.8e-3))

    else:
        # Upper mesosphere (to ~86 km): lapse rate -2.0 K/km
        T = 214.65 - 2.0e-3 * (h - 71000.0)
        P = 3.95642 * (T / 214.65) ** (G0 / (R_AIR * 2.0e-3))

    rho = P / (R_AIR * T)
    return rho, P, T


class Environment(object):

    def __init__(self, missile):
        self.missile = missile

        # outputs
        self.rho    = 0.0   # air density - kg/m^3
        self.press  = 0.0   # atmospheric pressure - Pa
        self.tempk  = 0.0   # temperature - K
        self.vsound = 0.0   # speed of sound - m/s
        self.grav   = 0.0   # gravity acceleration - m/s^2
        self.mach   = 0.0   # Mach number
        self.pdynmc = 0.0   # dynamic pressure - Pa

        # freeze state (same pattern as propulsion)
        self.mfreeze_environ = 0
        self.machf           = 0.0
        self.pdynmcf         = 0.0


    def environment(self):
        """Computes atmosphere, gravity, Mach number, and dynamic pressure.

        Reads from missile:
            sbel3   - down position in NED frame (altitude = -sbel3) - m
            dvbe    - total velocity - m/s
            mfreeze - autopilot freeze flag

        Writes to missile:
            press, rho, tempk, vsound, grav, mach, pdynmc
        """
        m = self.missile

        dvbe    = m.dvbe
        mfreeze = m.mfreeze

        # altitude above MSL: NED frame, sbel3 is Down component
        hbe = -m.sbel3

        # ── gravity (inverse-square law) ────────────────────────────────────
        rad  = REARTH + hbe
        grav = G_CONST * EARTH_MASS / (rad * rad)

        # ── US 1976 Standard Atmosphere ─────────────────────────────────────
        rho, press, tempk = atmosphere76(hbe)

        # ── speed of sound ──────────────────────────────────────────────────
        vsound = math.sqrt(1.4 * R_AIR * tempk)

        # ── Mach number and dynamic pressure ────────────────────────────────
        mach   = abs(dvbe / vsound)
        pdynmc = 0.5 * rho * dvbe * dvbe

        # ── freeze for autopilot response calculations ───────────────────────
        if mfreeze == 0:
            self.mfreeze_environ = 0
        else:
            if mfreeze != self.mfreeze_environ:
                self.mfreeze_environ = mfreeze
                self.machf   = mach
                self.pdynmcf = pdynmc
            mach   = self.machf
            pdynmc = self.pdynmcf

        # ── store outputs ───────────────────────────────────────────────────
        self.rho    = rho
        self.press  = press
        self.tempk  = tempk
        self.vsound = vsound
        self.grav   = grav
        self.mach   = mach
        self.pdynmc = pdynmc

        # push to missile (read by aerodynamics, propulsion, etc.)
        m.rho    = rho
        m.press  = press
        m.tempk  = tempk
        m.vsound = vsound
        m.grav   = grav
        m.mach   = mach
        m.pdynmc = pdynmc
        m.hbe    = hbe
