"""
Target scenarios for SRAAM6 simulation.

NED frame [North, East, Down],  hbe = -pos[2].
Scenario 1 uses the analytic (constant-velocity) form; all others advance
via an Euler step in Target.step().

Design fixes vs. original:
  - S2, S4, S6: acceleration is now velocity-perpendicular (centripetal) so
    airspeed is conserved.  Fixed-frame acceleration caused speed to grow
    without bound, which is unphysical for a turning aircraft.
  - S6: recovery phase added at t=8 s (acceleration zeroed after the break).
  - S9: lift coefficient increased to 3.0 g and drag to 0.7 g so that the
    total load factor is ~3.1 g — more representative of an afterburner climb.
"""

import math
import numpy as np

G = 9.80665


def _cw_perp(vel):
    """Unit vector 90° clockwise from the horizontal velocity component.

    For vel heading South (-N): returns East (+E).
    As the aircraft turns East the centripetal direction rotates with it,
    producing a coordinated right-hand break at constant airspeed.
    """
    vn, ve = float(vel[0]), float(vel[1])
    mag = math.hypot(vn, ve)
    if mag < 1.0:
        return np.zeros(3)
    return np.array([ve / mag, -vn / mag, 0.0])


class Target:
    NAMES = {
        1: "Constant speed      |  straight flight, lateral offset",
        2: "Accelerating escape |  t>1.5s  5-g right break turn",
        3: "Sinusoidal jink     |  3-g lateral + 2-g vertical weave",
        4: "Sharp L-maneuver    |  t>2s 8-g right break + 6-g pull-up",
        5: "Beaming / notch     |  90° crossing, high LOS rate",
        6: "Head-on + break     |  head-on, t=3s 7-g right break (recovery t=8s)",
        7: "Look-down engage    |  target at 7500m alt, dive intercept",
        8: "Last-ditch break    |  t=4s ~9-g combined break (endgame)",
        9: "Energy-bleed climb  |  afterburner ~3-g climbing turn",
       10: "Coordinated turn    |  sustained 4-g target turn",
    }

    # Initial positions (NED, m) and velocities (m/s).
    # S1-S4: same ~6200 m range, 72° aspect engagement geometry.
    # S5-S10: dedicated scenario geometries.
    _INIT = {
        #          North    East    Down         vN      vE     vD
        1: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        2: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        3: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        4: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        # S5  Beaming: target NW, flying due East (notch angle ~90°)
        5: dict(pos=( 5500, -2000, -10000), vel=(   0,  300,    0)),
        # S6  Head-on: closing at Mach ~1, head-on from North
        6: dict(pos=( 7000,     0, -10000), vel=(-300,    0,    0)),
        # S7  Look-down: target at 7500 m, missile at 10 000 m
        7: dict(pos=( 5500,  1000,  -7500), vel=(-220,    0,    0)),
        # S8  Last-ditch: same as S1 geometry, late combined break
        8: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        # S9  Energy-bleed: faster target, slight North offset
        9: dict(pos=( 6500,   800, -10000), vel=(-250,    0,    0)),
        # S10 Coordinated turn: long range, target already in banked turn
       10: dict(pos=(10000, -1000, -12000), vel=(-180,  120,    0)),
    }

    def __init__(self, scenario):
        ic       = self._INIT[scenario]
        self._p0 = np.array(ic['pos'], dtype=float)
        self._v0 = np.array(ic['vel'], dtype=float)
        self.pos = self._p0.copy()
        self.vel = self._v0.copy()
        self.s   = scenario

    def _accel(self, t):
        s = self.s

        # ── S1: no manoeuvre ──────────────────────────────────────────────────
        if s == 1:
            return np.zeros(3)

        # ── S2: 5-g right break turn at t=1.5 s ──────────────────────────────
        # Centripetal (velocity-perpendicular) so airspeed is conserved.
        elif s == 2:
            if t < 1.5:
                return np.zeros(3)
            return 5.0 * G * _cw_perp(self.vel)

        # ── S3: sinusoidal jink (3-g lateral + 2-g vertical, different freqs) ─
        elif s == 3:
            ay =  3.0 * G * math.sin(1.2 * t)
            az = -2.0 * G * math.cos(1.8 * t + math.pi / 3)
            return np.array([0.0, ay, az])

        # ── S4: 8-g right break then 6-g pull-up ──────────────────────────────
        # Break phase: centripetal so speed is conserved.
        # Pull-up phase: upward (-Down) acceleration, approximately
        # perpendicular to the now-mostly-East velocity.
        elif s == 4:
            if t < 2.0:
                return np.zeros(3)
            elif t < 3.5:
                return 8.0 * G * _cw_perp(self.vel)
            elif t < 4.8:
                return np.array([0.0, 0.0, -6.0 * G])   # 6 g up (NED Down < 0)
            else:
                return np.zeros(3)

        # ── S5: beaming / notch — constant velocity, max LOS rate ────────────
        elif s == 5:
            return np.zeros(3)

        # ── S6: head-on + 7-g right break at t=3 s, recovery at t=8 s ────────
        # Centripetal break so airspeed is conserved.
        # Recovery (zero accel) at t=8 s avoids unbounded speed growth.
        elif s == 6:
            if t < 3.0:
                return np.zeros(3)
            elif t < 8.0:
                return 7.0 * G * _cw_perp(self.vel)
            else:
                return np.zeros(3)

        # ── S7: look-down — straight constant-speed flight ────────────────────
        elif s == 7:
            return np.zeros(3)

        # ── S8: last-ditch ~9-g combined break at t=4 s ───────────────────────
        # |a| = sqrt(6.364² + 6.364²) = 9.0 g  (East + Up)
        elif s == 8:
            if t < 4.0:
                return np.zeros(3)
            return np.array([0.0, 6.364 * G, -6.364 * G])

        # ── S9: afterburner energy-bleed climb ───────────────────────────────
        # Physics: aerodynamic lift perpendicular to velocity (pull up) plus
        # net drag opposing velocity (afterburner partially offsets drag;
        # net = 0.7 g deceleration along track).
        # Total load: sqrt(3.0² + 0.7²) ≈ 3.08 g — representative of a
        # full-afterburner zoom climb.
        elif s == 9:
            v  = self.vel
            vm = float(np.linalg.norm(v))
            if vm < 1.0:
                return np.zeros(3)
            vhat = v / vm
            # net deceleration along velocity (drag > thrust residual)
            a_drag = -0.7 * G * vhat
            # lift perpendicular to velocity, in the vertical plane, pointing up
            up   = np.array([0.0, 0.0, -1.0])
            perp = up - np.dot(up, vhat) * vhat
            pm   = float(np.linalg.norm(perp))
            a_lift = (3.0 * G * perp / pm) if pm > 1e-6 else np.zeros(3)
            return a_drag + a_lift

        # ── S10: coordinated 4-g horizontal turn ─────────────────────────────
        # Centripetal acceleration perpendicular to horizontal velocity,
        # producing a constant-radius sustained turn.
        elif s == 10:
            v_mag = math.hypot(self.vel[0], self.vel[1])
            if v_mag < 0.1:
                return np.zeros(3)
            ax = -self.vel[1] / v_mag * 4.0 * G
            ay =  self.vel[0] / v_mag * 4.0 * G
            return np.array([ax, ay, 0.0])

        else:
            return np.zeros(3)

    def state(self, t):
        """Return (pos, vel) at simulation time t.

        S1 uses the exact analytic (constant-velocity) solution.
        All others return the current Euler-integrated state.
        """
        if self.s == 1:
            return self._p0 + self._v0 * t, self._v0.copy()
        return self.pos.copy(), self.vel.copy()

    def step(self, t, dt):
        """Advance internal state by dt using a first-order Euler step."""
        if self.s == 1:
            return
        acc      = self._accel(t)
        self.vel = self.vel + acc * dt
        self.pos = self.pos + self.vel * dt
