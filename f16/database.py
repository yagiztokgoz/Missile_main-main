import re
import numpy as np
from scipy.interpolate import interp1d, RegularGridInterpolator

LBF_TO_N = 4.44822   # pound-force → Newton
FT_TO_M  = 0.3048    # feet → metre


class F16_Database:
    """
    Reads F-16 aerodynamic and propulsion deck files and builds scipy
    interpolators for each table.

    Aero deck tables (all vs alpha in deg, unless otherwise noted):
      1DIM : cxq, cyr, cyp, cz, czq, clr, clp, cmq, cnr, cnp   (alpha → coeff)
      2DIM : cx_vs_elev_alpha, cm_vs_elev_alpha                  (elev, alpha → coeff)
             cl_vs_beta_alpha, cn_vs_beta_alpha                  (beta 0-30, alpha → coeff)
             clda_vs_beta_alpha, cldr_vs_beta_alpha              (beta ±30, alpha → coeff)
             cnda_vs_beta_alpha, cndr_vs_beta_alpha

    Prop deck tables:
      2DIM : idle_vs_mach_alt, mil_vs_mach_alt, max_vs_mach_alt  (Mach, alt_m → thrust_N)
      3DIM : ff_vs_thrust_alt_mach                               (thrust_N, alt_m, Mach → kg/s)

    Unit note: prop 2DIM tables are stored in English units (lbf, ft) and
    converted to SI (N, m) on load. The 3DIM fuel-flow table is already SI.
    """

    def __init__(self):
        self.aero_db = {}
        self.prop_db = {}

    # ── public loaders ─────────────────────────────────────────────────────────

    def load_aero_deck(self, filepath):
        with open(filepath) as f:
            lines = f.readlines()
        self._parse(lines, self.aero_db, prop=False)

    def load_prop_deck(self, filepath):
        with open(filepath) as f:
            lines = f.readlines()
        self._parse(lines, self.prop_db, prop=True)

    # ── top-level parser ───────────────────────────────────────────────────────

    def _parse(self, lines, db, prop):
        """Scan lines for 1DIM/2DIM/3DIM headers and dispatch to _build."""
        i, n = 0, len(lines)
        while i < n:
            clean = lines[i].split('//')[0].strip()
            m = re.match(r'^(1DIM|2DIM|3DIM)\s+(\w+)', clean)
            if not m:
                i += 1
                continue

            dim, name = m.group(1), m.group(2)
            i += 1

            # NX line: extract dimension sizes — match "NX1 5", "NX2 12", etc.
            # (plain r'\d+' would also grab digits inside "NX1", "NX2" keywords)
            nx = [int(v) for v in re.findall(r'NX\d+\s+(\d+)', lines[i].split('//')[0])]
            i += 1

            # Collect body lines until the next table header or EOF
            body = []
            while i < n:
                peek = lines[i].split('//')[0].strip()
                if re.match(r'^(1DIM|2DIM|3DIM|TITLE)\b', peek):
                    break
                body.append(peek)
                i += 1

            self._build(name, dim, nx, body, db, prop)

    # ── table builder ──────────────────────────────────────────────────────────

    def _build(self, name, dim, nx, body, db, prop):
        if dim == '1DIM':
            self._build_1d(name, nx, body, db)
        elif dim == '2DIM':
            self._build_2d(name, nx, body, db, prop)
        elif dim == '3DIM':
            self._build_3d(name, nx, body, db)

    def _build_1d(self, name, nx, body, db):
        """1DIM: each data line is (x1_i, value_i)."""
        nx1 = nx[0]
        x1, vals = [], []
        for line in body:
            ns = self._nums(line)
            if len(ns) >= 2:
                x1.append(ns[0])
                vals.append(ns[1])
        x1   = np.array(x1[:nx1])
        vals = np.array(vals[:nx1])
        if len(x1) < 2:
            return
        db[name] = interp1d(x1, vals, bounds_error=False, fill_value='extrapolate')

    def _build_2d(self, name, nx, body, db, prop):
        """
        2DIM: NX1 data rows + up to (NX2-NX1) thin axis-label rows.

        Each data row: [X1_i, X2_i, val_0 … val_{NX2-1}]
        Thin rows:     [X2_j]   (j = NX1 … NX2-1)

        So the flat number list has structure:
            For i in range(NX1): X1[i], X2[i], data[i][0..NX2-1]
            Then: X2[NX1..NX2-1]
        """
        nx1, nx2 = nx[0], nx[1]
        all_nums = []
        for line in body:
            all_nums.extend(self._nums(line))

        x1, x2, data = self._parse2d(all_nums, nx1, nx2)
        x1   = np.array(x1)
        x2   = np.array(x2)
        data = np.array(data)          # shape (nx1, nx2)

        # Prop 2D tables are in English units: X1=Mach (ND), X2=alt ft, data=lbf
        if prop and 'vs_mach_alt' in name:
            x2   *= FT_TO_M
            data *= LBF_TO_N

        db[name] = RegularGridInterpolator(
            (x1, x2), data, bounds_error=False, fill_value=None)

    def _build_3d(self, name, nx, body, db):
        """
        3DIM: NX1 data rows, each line i has:
            X1[i] always
            X2[i] if i < NX2
            X3[i] if i < NX3
            NX2*NX3 data values

        The fuel-flow table is already in SI (N, m, –, kg/s).
        """
        nx1, nx2, nx3 = nx[0], nx[1], nx[2]
        x1, x2, x3, data = self._parse3d(body, nx1, nx2, nx3)
        if not x1:
            return
        db[name] = RegularGridInterpolator(
            (np.array(x1), np.array(x2), np.array(x3)),
            np.array(data), bounds_error=False, fill_value=None)

    # ── parsing helpers ────────────────────────────────────────────────────────

    @staticmethod
    def _nums(line):
        """Extract all floats from a line (comments already stripped)."""
        return [float(t) for t in
                re.findall(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?', line)]

    @staticmethod
    def _parse2d(nums, nx1, nx2):
        """Split flat number list into x1 grid, x2 grid, and (nx1 x nx2) data."""
        step = 1 + 1 + nx2      # x1_label + x2_label + nx2 data values per row
        x1, x2_part, data = [], [], []
        for i in range(nx1):
            b = i * step
            x1.append(nums[b])
            x2_part.append(nums[b + 1])
            data.append(nums[b + 2: b + 2 + nx2])
        # Thin rows provide the remaining x2 axis values
        x2_rest = nums[nx1 * step: nx1 * step + (nx2 - nx1)]
        x2 = x2_part + list(x2_rest)
        return x1, x2, data

    @staticmethod
    def _parse3d(body, nx1, nx2, nx3):
        """
        Parse a 3D table line by line.
        Returns x1, x2, x3 grids and data array of shape (nx1, nx2, nx3).
        """
        _re = re.compile(r'[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?')
        x1, x2, x3, data = [], [], [], []
        row = 0
        for line in body:
            ns = [float(t) for t in _re.findall(line)]
            if not ns or row >= nx1:
                continue
            n_data  = nx2 * nx3
            has_x2  = row < nx2
            has_x3  = row < nx3
            pfx_len = 1 + has_x2 + has_x3
            if len(ns) < pfx_len + n_data:
                continue
            idx = 0
            x1.append(ns[idx]); idx += 1
            if has_x2:
                x2.append(ns[idx]); idx += 1
            if has_x3:
                x3.append(ns[idx]); idx += 1
            data.append(np.array(ns[idx: idx + n_data]).reshape(nx2, nx3))
            row += 1
        return x1, x2, x3, np.array(data) if data else np.array([])


# ── self-test ──────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import os
    base = os.path.dirname(os.path.abspath(__file__))
    db = F16_Database()
    db.load_aero_deck(os.path.join(base, 'data', 'f16_aero_deck.asc'))
    db.load_prop_deck(os.path.join(base, 'data', 'f16_prop_deck.asc'))

    print('F-16 Database loaded')
    print(f'  Aero tables : {sorted(db.aero_db)}')
    print(f'  Prop tables : {sorted(db.prop_db)}')

    alpha = 10.0   # deg
    elev  = 0.0    # deg
    beta  = 5.0    # deg
    mach  = 0.6
    alt_m = 3000.0

    print('\n--- Aero spot-checks (alpha=10°, beta=5°, elev=0°) ---')
    print(f'  CX            : {db.aero_db["cx_vs_elev_alpha"]([elev, alpha])[0]:.4f}')
    print(f'  CZ            : {db.aero_db["cz_vs_alpha"](alpha):.4f}')
    print(f'  CM            : {db.aero_db["cm_vs_elev_alpha"]([elev, alpha])[0]:.4f}')
    print(f'  CL(beta,alpha): {db.aero_db["cl_vs_beta_alpha"]([beta, alpha])[0]:.4f}')
    print(f'  CN(beta,alpha): {db.aero_db["cn_vs_beta_alpha"]([beta, alpha])[0]:.4f}')
    print(f'  Clp           : {db.aero_db["clp_vs_alpha"](alpha):.4f}')
    print(f'  Cmq           : {db.aero_db["cmq_vs_alpha"](alpha):.4f}')
    print(f'  Cnr           : {db.aero_db["cnr_vs_alpha"](alpha):.4f}')

    print(f'\n--- Prop spot-checks (Mach={mach}, alt={alt_m} m) ---')
    idle = db.prop_db['idle_vs_mach_alt']([mach, alt_m])[0]
    mil  = db.prop_db['mil_vs_mach_alt'] ([mach, alt_m])[0]
    maxt = db.prop_db['max_vs_mach_alt'] ([mach, alt_m])[0]
    print(f'  Idle thrust : {idle:8.1f} N')
    print(f'  Mil  thrust : {mil:8.1f} N')
    print(f'  Max  thrust : {maxt:8.1f} N')
    ff = db.prop_db['ff_vs_thrust_alt_mach']([mil, alt_m, mach])[0]
    print(f'  Fuel flow   : {ff:.4f} kg/s  (at mil thrust)')
