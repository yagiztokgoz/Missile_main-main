"""
F-16 Propulsion module — single turbofan (F100) with afterburner.

Ref: Stevens & Lewis, "Aircraft Control and Simulation", Wiley, 1992.
Data: Nguyen et al., NASA TP-1538, 1979.

Throttle → Power mapping:
    throttle ∈ [0.00, 0.77] → power ∈ [0, 50]  %  (idle → military)
    throttle ∈ [0.77, 1.00] → power ∈ [50, 100] %  (military → max/afterburner)

Spool-up lag:
    power < 50 % → τ = 1.0 s
    power ≥ 50 % → τ = 0.2 s

Thrust:
    power ∈ [0, 50]  → thrust = idle + power/50 * (mil  - idle)
    power ∈ [50,100] → thrust = mil  + (power-50)/50 * (max - mil)

Note: prop deck tables are converted to SI (N, m) at database load time,
so all queries here use Mach and altitude in metres, and return N.

mprop modes:
    0 — engine off  (thrust = 0)
    1 — direct throttle input  (0 → 1)
    2 — Mach hold controller  (throttle capped at 0.77 = mil power)
"""

import math

RAD   = math.pi / 180.0
SMALL = 1e-7


def _integrate(dydx_new, dydx, y, dt):
    """Trapezoidal integration (CADAC standard)."""
    return y + (dydx_new + dydx) * 0.5 * dt


class Propulsion:

    def __init__(self, f16):
        self.f16 = f16

        # mode
        self.mprop = 1        # 0: off | 1: manual throttle | 2: Mach hold

        # Mach hold parameters
        self.vmachcom  = 0.6    # commanded Mach
        self.gmach     = 1.0    # proportional gain  (Mach-error → throttle)
        self.ki_mach   = 0.0    # integral gain (eliminates steady-state Mach error)
        self._mach_int = 0.0    # integrator state

        # throttle (0 → 1)
        self.throttle = 0.3   # initial setting

        # power state — first-order lag output in %
        self.power  = 0.0     # achieved power setting - %
        self.powerd = 0.0     # derivative                - %/s

        # thrust output
        self.thrust = 0.0     # N

        # diagnostics
        self.power_com  = 0.0    # commanded power   - %
        self.tpower     = 1.0    # spool time const  - s
        self.idle       = 0.0    # idle thrust       - N
        self.mil        = 0.0    # military thrust   - N
        self.max_thrust = 0.0    # max/AB thrust     - N
        self.thrust_req = 0.0    # drag-balance thrust needed - N

        # freeze state (used when mfreeze != 0 for autopilot analysis)
        self.thrustf       = 0.0
        self._mfreeze_prev = 0

    # ── main propulsion step ───────────────────────────────────────────────────

    def propulsion(self, int_step):
        f = self.f16

        if self.mprop == 0:
            thrust = 0.0
            thrust_req = 0.0

        elif self.mprop == 1:
            thrust = self._thrust_from_throttle(self.throttle, int_step)
            thrust_req = 0.0

        else:  # mprop == 2 — Mach hold  (PI controller)
            emach = self.vmachcom - f.mach
            # integral with anti-windup clamp
            self._mach_int = max(-0.6, min(0.6,
                                           self._mach_int + emach * int_step))
            thr = self.gmach * emach + self.ki_mach * self._mach_int
            thr = max(0.05, min(thr, 0.77))   # floor at 5% to avoid idle drag
            self.throttle = thr
            thrust = self._thrust_from_throttle(thr, int_step)
            # thrust needed to overcome drag at current alpha
            cosa = max(math.cos(f.alphax * RAD), SMALL)
            thrust_req = (f.aerodynamics.cdrag * f.pdynmc
                          * f.aerodynamics.refa / cosa)

        # ── freeze for autopilot analysis ─────────────────────────────────────
        mfreeze = getattr(f, 'mfreeze', 0)
        if mfreeze == 0:
            self._mfreeze_prev = 0
        else:
            if mfreeze != self._mfreeze_prev:
                self._mfreeze_prev = mfreeze
                self.thrustf = thrust
            thrust = self.thrustf

        self.thrust     = thrust
        self.thrust_req = thrust_req
        f.thrust        = thrust

    # ── internal thrust calculator ─────────────────────────────────────────────

    def _thrust_from_throttle(self, throttle, int_step):
        f = self.f16

        # ── throttle → commanded power (%) ────────────────────────────────────
        if throttle <= 0.77:
            power_com = 64.94 * throttle           # 0 → 50 %
        else:
            power_com = 217.38 * throttle - 117.38  # 50 → 100 %
        power_com = max(0.0, min(power_com, 100.0))

        # ── spool-up time constant ────────────────────────────────────────────
        tpower = 1.0 if power_com <= 50.0 else 0.2

        # ── first-order lag on power ──────────────────────────────────────────
        powerd_new  = (power_com - self.power) / tpower
        self.power  = _integrate(powerd_new, self.powerd, self.power, int_step)
        self.powerd = powerd_new
        self.power  = max(0.0, min(self.power, 100.0))
        power       = self.power

        # ── table lookup (SI: Mach, alt_m → N) ───────────────────────────────
        mach  = f.mach
        alt_m = max(f.hbe, 0.0)   # clamp to sea level (tables start at 0 m)

        idle = float(f.db.prop_db['idle_vs_mach_alt']([mach, alt_m])[0])
        mil  = float(f.db.prop_db['mil_vs_mach_alt'] ([mach, alt_m])[0])
        maxt = float(f.db.prop_db['max_vs_mach_alt'] ([mach, alt_m])[0])

        # ── thrust from power setting ─────────────────────────────────────────
        if power < 50.0:
            thrust = idle + (power / 50.0) * (mil - idle)
        else:
            thrust = mil  + ((power - 50.0) / 50.0) * (maxt - mil)

        # store diagnostics
        self.power_com  = power_com
        self.tpower     = tpower
        self.idle       = idle
        self.mil        = mil
        self.max_thrust = maxt

        return thrust
