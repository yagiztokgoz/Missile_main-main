"""
F-16 main class — assembles all subsystems into a single 6-DoF model.

Integration order each step:
    1. environment   — atmosphere, Mach, dynamic pressure, wind
    2. kinematics    — quaternion propagation, TBL, alpha/beta
    3. aerodynamics  — force/moment coefficients + dimensional derivatives
    4. propulsion    — thrust (with power lag)
    5. forces        — FAPB (aero+thrust), FMB (moments)
    6. guidance      — acceleration commands from waypoint law
    7. control       — fin commands from autopilot
    8. actuator      — fin dynamics (second-order lag)
    9. euler         — angular velocity integration  (I·ω̇ = M − ω×(I·ω+h))
   10. newton        — translational velocity + position integration

Physical constants (NASA TP-1538 / Stevens & Lewis):
    mass        = 9 496 kg
    Ixx         = 12 875 kg·m²    Ixz = −1 331.4 kg·m²
    Iyy         = 75 673 kg·m²
    Izz         = 85 551 kg·m²
    eng_ang_mom = 70 000 kg·m²/s  (F100 rotor spin axis = body x)
"""

import os
import numpy as np

from .database     import F16_Database
from .environment  import Environment
from .kinematics   import Kinematics
from .aerodynamics import Aerodynamics
from .propulsion   import Propulsion
from .forces       import Forces
from .euler        import Euler
from .newton       import Newton
from .actuator     import Actuator
from .control      import Control
from .guidance     import Guidance


class F16:

    # ── physical constants ─────────────────────────────────────────────────────
    MASS        = 9496.0
    IBBB        = np.array([
        [12875.0,    0.0, -1331.4],
        [    0.0, 75673.0,    0.0],
        [-1331.4,    0.0, 85551.0],
    ])
    ENG_ANG_MOM = 70000.0

    def __init__(self):
        # ── database ───────────────────────────────────────────────────────────
        _data = os.path.join(os.path.dirname(__file__), 'data')
        self.db = F16_Database()
        self.db.load_aero_deck(os.path.join(_data, 'f16_aero_deck.asc'))
        self.db.load_prop_deck(os.path.join(_data, 'f16_prop_deck.asc'))

        # ── physical ───────────────────────────────────────────────────────────
        self.mass        = self.MASS
        self.IBBB        = self.IBBB.copy()
        self.eng_ang_mom = self.ENG_ANG_MOM
        self.ai11        = float(self.IBBB[0, 0])
        self.ai22        = float(self.IBBB[1, 1])
        self.ai33        = float(self.IBBB[2, 2])

        # ── initial conditions (overrideable before calling reset()) ───────────
        self.SBEL_INIT = [0.0, 0.0, -3000.0]   # NED position - m
        self.dvbe      = 200.0                   # initial speed - m/s
        self.alpha0x   = 2.0                     # AoA - deg
        self.beta0x    = 0.0                     # sideslip - deg
        self.psiblx    = 0.0                     # heading - deg
        self.thtblx    = 2.0                     # pitch - deg
        self.phiblx    = 0.0                     # roll - deg
        self.ppx       = 0.0                     # roll rate - deg/s
        self.qqx       = 0.0                     # pitch rate - deg/s
        self.rrx       = 0.0                     # yaw rate - deg/s

        # ── running state (written by subsystems each step) ────────────────────
        self.time   = 0.0

        # kinematics / attitude
        self.TBL    = np.eye(3)
        self.TLB    = np.eye(3)
        self.WBEB   = np.zeros(3)
        self.alphax = 0.0
        self.betax  = 0.0
        self.alppx  = 0.0
        self.psivlx = 0.0    # heading (velocity-frame) - deg
        self.thtvlx = 0.0    # flight-path angle - deg

        # translational
        self.VBEB   = np.zeros(3)
        self.VBEL   = np.zeros(3)
        self.SBEL   = np.zeros(3)
        self.VBAL   = np.zeros(3)   # velocity wrt air in local NED - m/s
        self.hbe    = 0.0           # altitude - m

        # speeds
        self.dvba   = 1.0           # airspeed (wrt air) - m/s
        # self.dvbe  already set above as initial condition, overwritten by newton

        # atmosphere / environment
        self.mach   = 0.0
        self.pdynmc = 0.0
        self.rho    = 1.225
        self.vsound = 340.3
        self.press  = 101325.0
        self.tempk  = 288.15
        self.grav   = 9.80665

        # forces
        self.FAPB   = np.zeros(3)
        self.FMB    = np.zeros(3)
        self.FSPB   = np.zeros(3)
        self.thrust = 0.0
        self.anx    = 0.0
        self.ayx    = 0.0

        # control surfaces
        self.delacx = 0.0   # aileron  command - deg
        self.delecx = 0.0   # elevator command - deg
        self.delrcx = 0.0   # rudder   command - deg
        self.delax  = 0.0   # aileron  actual  - deg
        self.delex  = 0.0   # elevator actual  - deg
        self.delrx  = 0.0   # rudder   actual  - deg

        # guidance / control commands
        self.alcomx = 0.0
        self.ancomx = 0.0

        # freeze flag (read by propulsion / environment)
        self.mfreeze = 0

        # ── subsystems ─────────────────────────────────────────────────────────
        self.environment  = Environment(self)
        self.kinematics   = Kinematics(self)
        self.aerodynamics = Aerodynamics(self)
        self.propulsion   = Propulsion(self)
        self.forces       = Forces(self)
        self.euler        = Euler(self)
        self.newton       = Newton(self)
        self.actuator     = Actuator(self)
        self.control      = Control(self)
        self.guidance     = Guidance(self)

        # ── initialise ─────────────────────────────────────────────────────────
        self._init()

    # ── initialisation ─────────────────────────────────────────────────────────

    def _init(self):
        """Initialise all subsystems from current initial-condition attributes."""
        self.kinematics.init()          # TBL, alphax, betax
        self.newton.init()              # VBEB, VBEL, SBEL, hbe
        self.euler.init()               # WBEB from ppx/qqx/rrx
        self.environment.environment(1e-3)  # atmosphere, mach, dvba
        self.aerodynamics.aerodynamics()    # coefficients + derivatives

    def reset(self):
        """Re-initialise from (possibly changed) initial-condition attributes."""
        self.time = 0.0
        self.WBEB = np.zeros(3)
        self.FAPB = np.zeros(3)
        self.FMB  = np.zeros(3)
        self.FSPB = np.zeros(3)
        self.actuator.DX   = np.zeros(3)
        self.actuator.DDX  = np.zeros(3)
        self.actuator.DXD  = np.zeros(3)
        self.actuator.DDXD = np.zeros(3)
        self.control.zz  = 0.0
        self.control.zzd = 0.0
        self._init()

    # ── main integration step ──────────────────────────────────────────────────

    def step(self, dt):
        self.environment.environment(dt)
        self.kinematics.kinematics(dt)
        self.aerodynamics.aerodynamics()
        self.propulsion.propulsion(dt)
        self.forces.forces()
        self.guidance.guidance()
        self.control.control(dt)
        self.actuator.actuator(dt)
        self.euler.euler(dt)
        self.newton.newton(dt)
        self.time += dt

    # ── target interface (compatible with simulate.py / target.py) ─────────────

    def state(self, t=None):
        """Return (pos_NED, vel_NED) — drop-in replacement for Target.state()."""
        return self.SBEL.copy(), self.VBEL.copy()
