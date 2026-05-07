#!/usr/bin/env python3
"""Export SRAAM6 aerodynamic deck tables to a MATLAB .mat file.

The Simulink model should use the exported breakpoint vectors and table
arrays directly in 1-D and n-D Lookup Table blocks.
"""

from pathlib import Path
import sys

from scipy.io import savemat

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import config
from missile.database import SRAAM6_Database


def main():
    out_path = Path(sys.argv[1]) if len(sys.argv) > 1 else ROOT / "simulink" / "aero_tables.mat"
    out_path.parent.mkdir(parents=True, exist_ok=True)

    db = SRAAM6_Database()
    db.load_aero_deck(config.AERO_DECK)

    mat = {
        "mach_bp": db.mach_grid,
        "alpha_bp": db.alpha_grid,
        "beta_bp": db.beta_grid,
    }

    for name, interp in db.aero_db.items():
        if hasattr(interp, "values"):
            # RegularGridInterpolator: table shape is (Mach, Alpha, Beta).
            mat[name] = interp.values
        else:
            # interp1d: x is Mach breakpoints, y is table values.
            mat[name] = interp.y

    savemat(out_path, mat)
    print(f"Wrote {out_path}")
    print(f"1-D/3-D aero tables: {len(db.aero_db)}")


if __name__ == "__main__":
    main()
