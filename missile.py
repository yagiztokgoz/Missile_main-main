
import math
import numpy as np
import config
from database import SRAAM6_Database
from environment import Environment
from kinematics import Kinematics
from newton import Newton
from euler import Euler
from aerodynamics import Aerodynamics
from actuator import Actuator
from propulsion import Propulsion
from forces import Forces
from control import Control
from guidance import Guidance
from ins import INS
from sensor import Sensor

class Missile(object):
    def __init__(self):

        # ── initial state (config.SBEL_INIT, ATTITUDE_INIT, AERO_INIT, SPEED_INIT)
        self.sbel1, self.sbel2, self.sbel3 = config.SBEL_INIT
        self.psix,  self.thtx,  self.phix  = config.ATTITUDE_INIT
        self.alpha0x, self.beta0x          = config.AERO_INIT
        self.dvbe                          = config.SPEED_INIT

        # aerodynamics
        self.alplimx  = config.ALP_LIMX
        self.refl     = config.REFL
        self.refa     = config.REFA

        # propulsion / mass / inertia
        self.xcgref   = config.XCG_REF
        self.mprop    = config.MPROP
        self.aexit    = config.AEXIT
        self.mass     = config.MASS
        self.xcg      = config.XCG
        self.ai11     = config.AI11
        self.ai33     = config.AI33

        # actuator
        self.mact     = config.MACT
        self.dlimx    = config.DLIMX
        self.ddlimx   = config.DDLIMX
        self.wnact    = config.WNACT
        self.zetact   = config.ZETACT

        # autopilot
        self.maut     = config.MAUT
        self.alimit   = config.ALIMIT
        self.dqlimx   = config.DQLIMX
        self.drlimx   = config.DRLIMX
        self.dplimx   = config.DPLIMX
        self.wblimx   = config.WBLIMX

        # roll controller
        self.zrcl     = config.ZRCL
        self.factwrcl = config.FACT_WRCL

        # rate controller
        self.zetlagr  = config.ZETLAGR

        # acceleration controller
        self.twcl     = config.TWCL
        self.wacl_bias= config.WACL_BIAS
        self.factwacl = config.FACT_WACL
        self.pacl     = config.PACL
        self.zacl     = config.ZACL

        # flight state (updated each integration step)
        self.time   = 0.0    # simulation time - s
        self.mach   = 0.0    # Mach number
        self.pdynmc = 0.0    # dynamic pressure - Pa
        self.alphax = 0.0    # angle of attack - deg
        self.betax  = 0.0    # sideslip angle - deg
        self.alppx  = 0.0    # total angle of attack - deg
        self.alpp   = 0.0    # total angle of attack - rad
        self.phip   = 0.0    # aerodynamic roll angle - rad
        self.phipx  = 0.0    # aerodynamic roll angle - deg
        self.ppx    = 0.0    # roll rate - deg/s
        self.qqx    = 0.0    # pitch rate - deg/s
        self.rrx    = 0.0    # yaw rate - deg/s
        self.dpx    = 0.0    # roll fin deflection - deg
        self.dqx    = 0.0    # pitch fin deflection - deg
        self.drx    = 0.0    # yaw fin deflection - deg
        self.thrust = 0.0    # engine thrust - N

        # kinematics (updated by Kinematics module)
        _a0 = self.alpha0x * math.pi / 180.0
        _b0 = self.beta0x  * math.pi / 180.0
        self.VBEB   = np.array([            # velocity in body frame - m/s
            self.dvbe * math.cos(_a0) * math.cos(_b0),
            self.dvbe * math.sin(_b0),
            self.dvbe * math.sin(_a0) * math.cos(_b0),
        ])
        self.TBL    = np.eye(3)   # body-wrt-local DCM
        self.TLB    = np.eye(3)   # local-wrt-body DCM (TBL transpose)
        self.psiblx = self.psix   # yaw angle - deg
        self.thtblx = self.thtx   # pitch angle - deg

        # environment (updated by Environment module)
        self.hbe    = 0.0    # altitude above MSL - m
        self.rho    = 0.0    # air density - kg/m^3
        self.vsound = 0.0    # speed of sound - m/s
        self.tempk  = 0.0    # atmospheric temperature - K
        self.grav   = config.AGRAV

        # TVC
        self.mtvc   = config.MTVC
        self.parm   = config.PARM
        self.gtvc   = config.GTVC

        # propulsion runtime
        self.thrust_input = 0.0    # constant thrust if mprop=3 - N
        self.launch_time  = 0.0    # time of launch - s
        self.press        = 101325.0  # atmospheric pressure at altitude - Pa
        self.mfreeze      = 0      # freeze flag for autopilot calculations

        # Monte Carlo aero perturbation — set externally before sim loop
        # {param_name: multiplicative_factor}, e.g. {'dna': 1.12, 'dma': 0.93}
        # Empty dict = no perturbation (nominal run)
        self.aero_pert = {}

        self.db = SRAAM6_Database()
        self.db.load_aero_deck(config.AERO_DECK)
        self.db.load_propulsion_deck(config.PROP_DECK)

        # RCS (off by default - zero vectors)

        self.FARCS = np.zeros(3)   # RCS forces in body axes - N
        self.FMRCS = np.zeros(3)   # RCS moments in body axes - N*m

        # TVC force/moment vectors (used when mtvc != 0)
        self.FPB   = np.zeros(3)   # TVC thrust force in body axes - N
        self.FMPB  = np.zeros(3)   # TVC thrust moment in body axes - N*m

        # forces output
        self.FAPB  = np.zeros(3)   # Total non-grav forces in body axes - N
        self.FMB   = np.zeros(3)   # Total moments in body axes - N*m

        # guidance
        self.mguid       = 0              # set by simulate.run() (|mid|term|)
        self.gnav        = config.GNAV
        self.mnav        = config.MNAV
        self.STEL        = np.zeros(3)    # target position in local axes - m
        self.VTEL        = np.zeros(3)    # target velocity in local axes - m/s
        self.SAEL        = np.zeros(3)    # aircraft position in local axes - m
        self.VAEL        = np.zeros(3)    # aircraft velocity in local axes - m/s
        self.WOELC       = np.zeros(3)    # LOS angular rate - rad/s
        self.UTBLC       = np.zeros(3)    # LOS unit vector
        self.dtbc        = 0.0            # distance to target - m
        self.dvtbc       = 0.0            # closing speed - m/s
        self.tgoc        = 0.0            # time-to-go - s
        # line guidance parameters
        self.line_gain   = config.LINE_GAIN
        self.nl_gain_fact= config.NL_GAIN_FACT
        self.decrement   = config.DECREMENT
        self.thtflx      = config.THTFLX
        # seeker data (populated by sensor module when available)
        self.psipb       = 0.0            # seeker yaw body angle - rad
        self.thtpb       = 0.0            # seeker pitch body angle - rad
        self.psipbx      = 0.0            # seeker yaw body angle - deg
        self.thtpbx      = 0.0            # seeker pitch body angle - deg
        self.sigdpy      = 0.0            # seeker LOS rate y - rad/s
        self.sigdpz      = 0.0            # seeker LOS rate z - rad/s
        # seeker config (kinematic IR seeker)
        self.mseek       = config.MSEEK
        self.racq        = config.RACQ
        self.dtimac      = config.DTIMAC
        self.dblind      = config.DBLIND

        # guidance commands (written by guidance, read by control)
        self.ancomx  = 0.0           # normal accel command - g's
        self.alcomx  = 0.0           # lateral accel command - g's

        # roll control
        self.phicomx = 0.0           # commanded roll angle - deg
        self.phiblx  = 0.0           # current bank angle - deg
        self.wrcl    = 0.0           # roll nat freq - rad/s

        # control fin commands (written by control, read by actuator)
        self.dpcx    = 0.0           # roll command - deg
        self.dqcx    = 0.0           # pitch command - deg
        self.drcx    = 0.0           # yaw command - deg
        self.dqcx_rcs= 0.0           # pitch RCS command - deg
        self.drcx_rcs= 0.0           # yaw RCS command - deg

        # feed-forward gain (accel controller)
        self.gainp   = 0.0           # feed-forward gain - s^2/m

        # body-frame vectors — estimate channels (INS-computed)
        # Read by control + guidance; when mins=0 these are passthrough of truth.
        self.WBECB   = np.zeros(3)   # computed angular velocity [p,q,r] - rad/s
        self.FSPCB   = np.zeros(3)   # computed specific force in body axes - m/s^2

        # truth channels (written by euler/newton, consumed only by INS)
        self.WBEB    = np.zeros(3)   # truth angular velocity - rad/s
        self.FSPB    = np.zeros(3)   # truth specific force - m/s^2

        # newton outputs
        self.VBEL    = np.zeros(3)   # velocity in local (NED) axes - m/s
        self.SBEL    = np.array([self.sbel1, self.sbel2, self.sbel3], dtype=float)
        self.SLEL    = self.SBEL.copy()  # launch point - m
        self.psivlx  = 0.0           # heading angle - deg
        self.thtvlx  = 0.0           # vertical flight path angle - deg
        self.anx     = 0.0           # normal specific force - g's
        self.ayx     = 0.0           # lateral specific force - g's

        # ── INS ─────────────────────────────────────────────────────────────
        self.mins      = config.MINS
        self.ins_seed  = config.INS_SEED

        # INS estimate outputs (filled by INS.init(), then each ins() tick)
        self.SBELC   = self.SBEL.copy()      # computed missile pos   - m
        self.VBELC   = np.zeros(3)           # computed missile vel   - m/s
        self.TBLC    = np.eye(3)             # computed body-wrt-local DCM
        self.hbem    = 0.0                   # altimeter measurement  - m
        self.dvbec   = self.dvbe             # computed speed         - m/s
        self.thtblc  = 0.0;  self.thtblcx = 0.0
        self.phiblcx = 0.0;  self.psivlcx = 0.0;  self.thtvlcx = 0.0

        self.environment  = Environment(self)
        self.kinematics   = Kinematics(self)
        self.newton       = Newton(self)
        self.euler        = Euler(self)
        self.aerodynamics = Aerodynamics(self)
        self.actuator     = Actuator(self)
        self.propulsion   = Propulsion(self)
        self.forces       = Forces(self)
        self.control      = Control(self)
        self.guidance     = Guidance(self)

        self.ins          = INS(self)
        self.ins.init()
        
        self.sensor       = Sensor(self)
        self.sensor.init()


