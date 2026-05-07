import math
import numpy as np

RAD   = math.pi / 180.0
DEG   = 180.0 / math.pi
AGRAV = 9.80665
SMALL = 1e-7


# ── helpers ────────────────────────────────────────────────────────────────────

def pol_from_cart(v):
    """Cartesian → spherical (CADAC convention).

    Returns [range, azimuth, elevation] where
      az  = atan2(y, x)          (rad, measured from x-axis in x-y plane)
      el  = asin(-z / r)         (rad, positive upward in NED: z_NED is down)
    """
    x, y, z = float(v[0]), float(v[1]), float(v[2])
    r = math.sqrt(x*x + y*y + z*z)
    if r > SMALL:
        el = math.asin(max(-1.0, min(1.0, -z / r)))
        d  = math.sqrt(x*x + y*y)
        az = math.atan2(y, x) if d > SMALL else 0.0
    else:
        az, el = 0.0, 0.0
    return np.array([r, az, el])


def mat2tr(az, el):
    """2-axis rotation DCM (azimuth then elevation), no roll.

    Equivalent to mat3tr(az, el, 0):
        [[ ca*ce,  sa*ce, -se ],
         [ -sa,    ca,     0  ],
         [ ca*se,  sa*se,  ce ]]
    """
    ca, sa = math.cos(az), math.sin(az)
    ce, se = math.cos(el), math.sin(el)
    return np.array([
        [ ca*ce,  sa*ce, -se ],
        [ -sa,    ca,     0. ],
        [ ca*se,  sa*se,  ce ],
    ])


# ── guidance class ─────────────────────────────────────────────────────────────

class Guidance(object):
    """Guidance module for SRAAM6.

    mguid = |mid|term| where:
        mid  = 0  no midcourse guidance
               3  proportional navigation (kinematic data)
               2  line guidance air-to-ground
               5  line guidance air-to-air
        term = 0  no terminal guidance
               6  compensated pro-nav (requires seeker data)
    """

    def __init__(self, missile):
        self.missile = missile

        # saved state
        self.epchta       = 0.0          # epoch of last target data receipt - s
        self.STELM        = np.zeros(3)  # stored target position - m
        self.VTELC        = np.zeros(3)  # stored target velocity - m/s
        self.init_guide_line = True      # one-shot init flag for line guidance

        # diagnostics
        self.dtbc    = 0.0
        self.tgoc    = 0.0
        self.dvtbc   = 0.0
        self.psiobcx = 0.0
        self.thtobcx = 0.0
        self.WOELC   = np.zeros(3)
        self.all_acc = 0.0
        self.ann_acc = 0.0


    # ── main dispatcher ────────────────────────────────────────────────────────

    def guidance(self):
        """Compute acceleration commands ancomx, alcomx (g's).

        Reads from missile:
            mguid    - guidance mode flag
            gnav     - navigation gain - ND
            STEL     - target position in local axes - m
            VTEL     - target velocity in local axes - m/s
            SAEL     - aircraft position in local axes - m  (line guidance)
            VAEL     - aircraft velocity in local axes - m/s (line guidance)
            TBL      - body-wrt-local DCM
            VBEL     - missile velocity in local axes - m/s
            SBEL     - missile position in local axes - m
            SLEL     - missile launch point - m
            alimit   - structural g limiter (used as gmax)
            mfreeze
        """
        m     = self.missile
        mguid = m.mguid

        if mguid == 0:
            return

        # decode flag
        guid_mid  = mguid // 10
        guid_term = mguid %  10

        # target position: use direct data when available (mnav=0)
        # or extrapolate from last datalink update (mnav=3)
        mnav = getattr(m, 'mnav', 0)
        if mnav == 3:
            self.epchta = m.launch_time
            self.STELM  = m.STEL.copy()
            self.VTELC  = m.VTEL.copy()

        if mnav == 0:
            # direct target data — use m.STEL / m.VTEL as-is
            STELC = m.STEL.copy()
            VTELC = m.VTEL.copy()
        else:
            # extrapolate from last datalink epoch
            dtime = m.launch_time - self.epchta
            STELC = self.STELM + self.VTELC * dtime
            VTELC = self.VTELC.copy()

        STBLC  = STELC - m.SBELC          # target wrt missile (local axes, INS estimate)

        # aircraft wrt target (for line guidance)
        STALC  = STELC - m.SAEL           # target wrt aircraft (local axes)

        # ── midcourse ──────────────────────────────────────────────────────
        ACBX = np.zeros(3)

        if guid_mid == 3:
            ACBX = self._pronav_mid(STBLC, VTELC)

        elif guid_mid == 2:
            ACBX = self._line_mid(STALC, STBLC, m.VBELC, m.nl_gain_fact)

        elif guid_mid == 5:
            if self.init_guide_line:
                self.init_guide_line = False
                # nl_gain_fact stays at its input value for air-to-air
            ACBX = self._line_mid(STALC, STBLC, m.VBELC, m.nl_gain_fact)

        # ── terminal ───────────────────────────────────────────────────────
        # only fire compensated pronav when the seeker is locked-on (mseek==4);
        # otherwise fall through with the midcourse ACBX.
        if guid_term == 6 and getattr(m, 'mseek', 0) == 4:
            ACBX = self._pronav_term_comp()

        # ── lateral / normal from ACBX ─────────────────────────────────────
        all_acc =  ACBX[1]
        ann_acc = -ACBX[2]

        # circular acceleration limiter
        gmax = m.alimit   # use structural limiter as guidance cap
        aa   = math.sqrt(all_acc*all_acc + ann_acc*ann_acc)
        if aa > gmax:
            aa = gmax
        if abs(ann_acc) < SMALL and abs(all_acc) < SMALL:
            phi = 0.0
        else:
            phi = math.atan2(ann_acc, all_acc)

        alcomx = aa * math.cos(phi)
        ancomx = aa * math.sin(phi)

        # diagnostics
        self.all_acc = all_acc
        self.ann_acc = ann_acc

        # output
        m.alcomx = alcomx
        m.ancomx = ancomx


    # ── midcourse proportional navigation ─────────────────────────────────────

    def _pronav_mid(self, STBLC, VTELC):
        """Pro-nav from third-party kinematic data.

        Args:
            STBLC: target wrt missile in local axes - m
            VTELC: target velocity in local axes - m/s
        Returns:
            ACBX: acceleration command in body axes - g's (3-vector)
        """
        m    = self.missile
        TBL  = m.TBLC             # INS-estimated DCM (passthrough when mins=0)
        VBEL = m.VBELC            # INS-estimated velocity
        gnav = m.gnav

        dtbc = float(np.linalg.norm(STBLC))
        if dtbc < SMALL:
            return np.zeros(3)

        UTBLC = STBLC / dtbc

        # LOS angles for diagnostics
        UTBBC = TBL @ UTBLC
        POLAR = pol_from_cart(UTBBC)
        self.psiobcx = POLAR[1] * DEG
        self.thtobcx = POLAR[2] * DEG

        # relative velocity (target wrt missile)
        VTBLC = VTELC - VBEL

        # closing speed (positive = closing)
        dvtbc = abs(float(np.dot(UTBLC, VTBLC)))

        # time-to-go (diagnostic)
        tgoc = dtbc / dvtbc if dvtbc > SMALL else 0.0

        # inertial LOS rate in local axes
        WOELC = np.cross(UTBLC, VTBLC) / dtbc

        # pro-nav acceleration command (body axes) - g's
        ACBX = TBL @ np.cross(WOELC, UTBLC) * gnav * dvtbc / AGRAV

        # store diagnostics
        self.dtbc   = dtbc
        self.dvtbc  = dvtbc
        self.tgoc   = tgoc
        self.WOELC  = WOELC

        # push LOS data to missile
        m.WOELC  = WOELC
        m.UTBLC  = UTBLC
        m.dtbc   = dtbc
        m.dvtbc  = dvtbc
        m.tgoc   = tgoc

        return ACBX


    # ── midcourse line guidance ────────────────────────────────────────────────

    def _line_mid(self, STALC, STBLC, VBELC, nl_gain_fact):
        """Line guidance: steer missile along the target-aircraft line.

        Args:
            STALC:       target wrt aircraft in local axes - m
            STBLC:       target wrt missile in local axes - m
            VBELC:       missile velocity in local axes - m/s
            nl_gain_fact: nonlinear gain factor - ND
        Returns:
            ACBX: acceleration command in body axes - g's
        """
        m         = self.missile
        TBL       = m.TBLC        # INS-estimated DCM
        grav      = m.grav
        line_gain = m.line_gain
        decrement = m.decrement
        thtflx    = m.thtflx     # vertical LOA angle override (0 → compute on-line)

        thtvlx = m.thtvlx   # vertical flight path angle - deg (from newton)
        psivlx = m.psivlx   # heading angle - deg (from newton)

        # missile pos wrt launch point
        SBTL = m.SBELC - m.SLEL   # missile pos (INS estimate) wrt launch

        # LOA frame: direction from aircraft to target
        POLAR_LOA = pol_from_cart(STALC)
        dtac  = POLAR_LOA[0]
        az_loa = POLAR_LOA[1]
        el_loa = POLAR_LOA[2]

        if thtflx == 0.0:
            TFL = mat2tr(az_loa, el_loa)
        else:
            TFL = mat2tr(az_loa, thtflx * RAD)

        # LOS frame: direction from missile to target
        POLAR_LOS = pol_from_cart(STBLC)
        dtbc  = POLAR_LOS[0]
        az_los = POLAR_LOS[1]
        el_los = POLAR_LOS[2]
        TOL = mat2tr(az_los, el_los)

        # velocity-frame DCM
        TVL = mat2tr(psivlx * RAD, thtvlx * RAD)
        TBV = TBL @ TVL.T

        # missile velocity in LOS and LOA frames
        VBEO = TOL @ VBELC
        VBEF = TFL @ VBELC

        # nonlinear gain
        nl_gain = nl_gain_fact * (1.0 - math.exp(-dtbc / max(decrement, SMALL)))

        # line guidance steering law (velocity axes) - g's
        algv1 =  grav * math.sin(thtvlx * RAD) / AGRAV
        algv2 =  line_gain * (-VBEO[1] + nl_gain * VBEF[1]) / AGRAV
        algv3 =  line_gain * ((-VBEO[2] + nl_gain * VBEF[2])
                               - grav * math.cos(thtvlx * RAD)) / AGRAV

        ACVX = np.array([algv1, algv2, algv3])

        # convert to body axes
        ACBX = TBV @ ACVX

        # diagnostic: missile position normal to LOS
        SBTO = TOL @ SBTL

        self.dtbc = dtbc
        m.dtbc = dtbc

        return ACBX


    # ── terminal compensated pro-nav ───────────────────────────────────────────

    def _pronav_term_comp(self):
        """Terminal compensated pro-nav (requires seeker data).

        Reads from missile:
            psipb, thtpb  - seeker body pointing angles - rad
            sigdpy, sigdpz - seeker LOS rates - rad/s
            FSPCB         - specific force in body axes - m/s^2
            TBL, VBEL, SBEL
            STEL, VTEL
        Returns:
            ACBX: acceleration command in body axes - g's
        """
        m    = self.missile
        TBL  = m.TBLC        # INS-estimated DCM
        VBEL = m.VBELC       # INS-estimated velocity
        SBEL = m.SBELC       # INS-estimated position
        gnav = m.gnav

        STEL = m.STEL
        VTEL = m.VTEL

        psipb  = m.psipb    # seeker yaw body angle - rad
        thtpb  = m.thtpb    # seeker pitch body angle - rad
        sigdpy = m.sigdpy   # LOS rate y - rad/s
        sigdpz = m.sigdpz   # LOS rate z - rad/s
        FSPCB  = m.FSPCB

        # closing velocity
        SBTL = SBEL - STEL
        dbt  = float(np.linalg.norm(SBTL))
        if dbt < SMALL:
            return np.zeros(3)
        dum   = float(np.dot(SBTL, VBEL - VTEL))
        dcvel = abs(dum / dbt)

        # missile longitudinal accel correction
        fspcb1 = float(FSPCB[0])
        adely = fspcb1 * math.tan(psipb) / AGRAV
        adelz = fspcb1 * math.tan(thtpb) / (math.cos(psipb) * AGRAV)

        # gravity bias in body frame
        GRAVL = np.array([0.0, 0.0, 1.0])
        GRAVB = TBL @ GRAVL

        # acceleration commands
        gn    = gnav * dcvel
        apny  = gn * sigdpz / (math.cos(psipb) * AGRAV)
        apnz  = gn * (sigdpz * math.tan(thtpb) * math.tan(psipb)
                      + sigdpy / math.cos(thtpb)) / AGRAV

        all_acc = apny + adely - GRAVB[1]
        ann_acc = apnz + adelz + GRAVB[2]

        ACBX = np.array([0.0, all_acc, -ann_acc])
        return ACBX
