"""
Target scenarios for SRAAM6 simulation.

NED frame [North, East, Down], hbe = -pos[2].
Scenario 1 uses analytic state; scenarios 2-10 advance via Euler step.
"""

import math
import numpy as np

G = 9.80665


class Target:
    NAMES = {
        1: "Constant speed      |  straight flight, lateral offset",
        2: "Accelerating escape |  t>1.5s 5-g lateral sprint",
        3: "Sinusoidal jink     |  3-g lateral + 2-g vertical weave",
        4: "Sharp L-maneuver    |  t>2s 8-g right brake + 6-g up",
        5: "Beaming / notch     |  90° crossing, high LOS rate",
        6: "Head-on + break     |  head-on, t=3s 7g right break",
        7: "Look-down engage    |  target at 7500m alt, dive intercept",
        8: "Last-ditch break    |  t=4s ~9g combined break (endgame)",
        9: "Energy-bleed climb  |  target climbs with afterburner",
       10: "Coordinated turn    |  sustained 4g target turn",
    }

    # S1-S4: ayni engagement geometrisinden (~6200m, 72° aspect).
    # S5-S10: gercek angajman senaryolari (beaming, head-on, vs).
    _INIT = {
        #          North    East    Down         vN      vE     vD
        1: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        2: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        3: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        4: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        # 5 Beaming: hedef kuzey-batida, tamamen Dogu'ya ucuyor (notch)
        5: dict(pos=( 5500, -2000, -10000), vel=(   0,  300,    0)),
        # 6 Head-on: tam karsidan Mach 0.9 ile yaklasiyor
        6: dict(pos=( 7000,     0, -10000), vel=(-300,    0,    0)),
        # 7 Look-down: hedef 7500m irtifada, missile 10000m'den dive ile yakalar
        7: dict(pos=( 5500,  1000,  -7500), vel=(-220,    0,    0)),
        # 8 Last-ditch: S1 geometrisi, son anda break
        8: dict(pos=( 6000,  1500, -10000), vel=(-200,    0,    0)),
        # 9 Enerji-bleed climb: hedef tirmanirken yavaslar
        9: dict(pos=( 6500,   800, -10000), vel=(-250,    0,    0)),
        # 10 Coordinated turn: hedef 4g banked turn (klasik evasion)
       10: dict(pos=( 10000, -1000, -12000), vel=(-180,  120,    0)),
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
        if s == 1:
            return np.zeros(3)
        elif s == 2:
            # t>1.5s'de 5-g Dogu sprint (pro-nav'i zorlar ama yakalanabilir)
            if t < 1.5:
                return np.zeros(3)
            return np.array([0.0, 5.0 * G, 0.0])
        elif s == 3:
            # 3-g yanal + 2-g dikey sinus jink (farkli frekanslar)
            ay =  3.0 * G * math.sin(1.2 * t)
            az = -2.0 * G * math.cos(1.8 * t + math.pi / 3)
            return np.array([0.0, ay, az])
        elif s == 4:
            # t=2..3.5:  8-g sag fren
            # t=3.5..4.8: 6-g yukari (NED'de -D = yukari)
            if t < 2.0:
                return np.zeros(3)
            elif t < 3.5:
                return np.array([0.0, 8.0 * G, 0.0])
            elif t < 4.8:
                return np.array([0.0, 0.0, -6.0 * G])
            else:
                return np.zeros(3)
        elif s == 5:
            # Beaming / notch: Sabit hizla 90 derece cross rotada ucus
            return np.zeros(3)
        elif s == 6:
            # Head-on + break: t=3s'de 7-g sag break (NED ekseninde Dogu/Y ivmesi)
            if t < 3.0:
                return np.zeros(3)
            return np.array([0.0, 7.0 * G, 0.0])
        elif s == 7:
            # Look-down engage: Sabit hizla alcaktan ucus
            return np.zeros(3)
        elif s == 8:
            # Last-ditch break: t=4s'de ~9-g combined break (6.36g yanal + 6.36g dikey)
            # |a| = sqrt(6.36^2 + 6.36^2) = 9.0 g
            if t < 4.0:
                return np.zeros(3)
            return np.array([0.0, 6.364 * G, -6.364 * G])
        elif s == 9:
            # Energy-bleed climb: hiz vektorune gore dikey 'lift' + drag (hem gerceklci hem
            # bagimsiz of frame). Target yon degistirse bile 'lift' hep hiz-dik yukari,
            # 'drag' hep hizla ters.
            v  = self.vel
            vm = float(np.linalg.norm(v))
            if vm < 1.0:
                return np.zeros(3)
            vhat = v / vm
            a_drag = -0.5 * G * vhat
            up = np.array([0.0, 0.0, -1.0])
            perp = up - np.dot(up, vhat) * vhat
            pm   = float(np.linalg.norm(perp))
            if pm > 1e-6:
                lift_dir = perp / pm
                a_lift = 1.5 * G * lift_dir
            else:
                a_lift = np.zeros(3)
            return a_drag + a_lift
        elif s == 10:
            # Coordinated turn: 4-g yatay sabit donus
            v_mag = math.hypot(self.vel[0], self.vel[1])
            if v_mag < 0.1:
                return np.zeros(3)
            ax = -self.vel[1] / v_mag * 4.0 * G
            ay =  self.vel[0] / v_mag * 4.0 * G
            return np.array([ax, ay, 0.0])
        else:
            return np.zeros(3)

    def state(self, t):
        """(pos, vel) at absolute time t.

        Scenario 1 uses exact analytic form; others return integrated state.
        """
        if self.s == 1:
            return self._p0 + self._v0 * t, self._v0.copy()
        return self.pos.copy(), self.vel.copy()

    def step(self, t, dt):
        """Advance internal Euler state by dt (scenarios 2-10)."""
        if self.s == 1:
            return
        acc      = self._accel(t)
        self.vel = self.vel + acc * dt
        self.pos = self.pos + self.vel * dt
