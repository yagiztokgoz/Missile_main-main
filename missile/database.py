import numpy as np
from scipy.interpolate import interp1d, RegularGridInterpolator
import re

class SRAAM6_Database:
    def __init__(self):
        self.aero_db = {}
        self.prop_db = {}
        # Regex pattern to extract only numbers (ignores text/headers)
        self.num_pattern = re.compile(r'[-+]?\d*\.\d+|\d+')
        
        # Static axes (grids) for Aerodynamic Tables based on the deck header
        self.mach_grid = np.array([0.5, 0.8, 0.95, 1.05, 1.2, 1.6, 2.0, 3.0, 4.5, 5.5])
        self.alpha_grid = np.arange(-48, 52, 4.0)
        self.beta_grid = np.arange(-48, 52, 4.0)

    def load_aero_deck(self, filepath):
        """Reads aerodynamic coefficients and builds interpolators."""
        with open(filepath, 'r') as f:
            lines = f.readlines()

        current_table_name, current_table_type = None, None
        current_numbers = []

        for line in lines:
            line = line.strip()
            if '//' in line: line = line.split('//')[0] # Remove comments
            
            match = re.match(r'(1DIM|3DIM)\s+(\w+)', line)
            if match:
                self._save_aero_table(current_table_name, current_table_type, current_numbers)
                current_table_type, current_table_name = match.group(1), match.group(2)
                current_numbers = []
                continue
            
            # Skip headers, grid definitions, or empty lines
            if current_table_name is None or line.startswith('NX1') or line.startswith('Alpha') or line.startswith('Beta') or line.startswith('Mach'):
                continue
                
            nums = self.num_pattern.findall(line)
            if nums:
                current_numbers.extend([float(n) for n in nums])


        self._save_aero_table(current_table_name, current_table_type, current_numbers)

    def _save_aero_table(self, name, t_type, nums):
        """Converts the raw number list into SciPy interpolation objects."""
        if name is None or not nums: return
        
        if t_type == '1DIM':
            # 1D tables are in (Mach, Value) pairs. We slice starting from index 1 with step 2 to get values.
            vals = nums[1::2]
            self.aero_db[name] = interp1d(self.mach_grid, vals, bounds_error=False, fill_value="extrapolate")
            
        elif t_type == '3DIM':
            data = []
            idx = 0
            for _ in range(10): # 10 Mach pages
                idx += 3 # Skip the [Mach, Alpha, Beta] coordinate tags at the beginning of each block
                data.extend(nums[idx : idx + 625]) # Extract the 25x25 block
                idx += 625
                
            matrix = np.array(data).reshape((10, 25, 25))
            self.aero_db[name] = RegularGridInterpolator(
                (self.mach_grid, self.alpha_grid, self.beta_grid), 
                matrix, bounds_error=False, fill_value=None 
            )

    def load_propulsion_deck(self, filepath):
        """Reads engine and mass data and builds interpolators."""
        with open(filepath, 'r') as f:
            lines = f.readlines()

        current_table_name = None
        times, values = [], []

        for line in lines:
            line = line.strip()
            if '//' in line: line = line.split('//')[0]
            
            match = re.match(r'1DIM\s+(\w+)', line)
            if match:
                if current_table_name is not None:

                    self.prop_db[current_table_name] = interp1d(times, values, bounds_error=False, fill_value=(values[0], values[-1]))
                current_table_name = match.group(1)
                times, values = [], []
                continue
            
            if current_table_name is None or line.startswith('NX1') or not line:
                continue
                
            nums = self.num_pattern.findall(line)
            if len(nums) == 2:
                times.append(float(nums[0]))
                values.append(float(nums[1]))

        if current_table_name is not None:
            self.prop_db[current_table_name] = interp1d(times, values, bounds_error=False, fill_value=(values[0], values[-1]))


if __name__ == "__main__":

    db = SRAAM6_Database()
    db.load_aero_deck(r'/home/ygz/Desktop/Missile/data/sraam6_aero_deck.asc')
    db.load_propulsion_deck(r'/home/ygz/Desktop/Missile/data/sraam6_prop_deck.asc')
    
    print("SRAAM6 Database Loaded Successfully!\n" + "="*45)
    
    # 2. Flight Simulation Current State (State Variables)
    t = 2.80          # seconds (Engine is off - Coast Phase)
    mach = 1.60       # Mach number
    alpha = -48.0       # degrees (Angle of Attack)
    beta = -48.0       # degrees (Sideslip Angle)
    delta_q = 2.0     # degrees (Elevator deflection - pitch command)
    
    print(f"FLIGHT STATE:\nTime: {t}s | Mach: {mach} | Alpha: {alpha}° | Beta: {beta}° | Fin Deflection: {delta_q}°\n" + "-"*45)

    # 3. PROPULSION AND MASS READINGS (Time-dependent)
    mass = db.prop_db['mass_vs_time'](t)
    thrust = db.prop_db['thrust_vs_time'](t)
    cg = db.prop_db['cg_vs_time'](t)
    moi_pitch = db.prop_db['moipitch_vs_time'](t)
    
    print("PROPULSION DATA:")
    print(f"  Engine Thrust : {thrust:.2f} N")
    print(f"  Mass          : {mass:.2f} kg")
    print(f"  CG Location   : {cg:.4f} m (from nose)")
    print(f"  Inertia (Iyy) : {moi_pitch:.2f} kg*m^2\n")

    # 4. AERODYNAMIC READINGS AND MATH
    query = [mach, alpha, beta]
    
    # Axial Force (Drag) Calculation
    ca0 = db.aero_db['ca0_vs_mach'](mach)
    caa = db.aero_db['caa_vs_mach_alpha_beta'](query)[0]
    cad = db.aero_db['cad_vs_mach'](mach) * (delta_q**2) # Fin drag is proportional to deflection squared
    
    # Base Drag Logic: Is the engine producing thrust?
    if thrust > 0:
        # If engine is active, account for base drag reduction (added as a negative value)
        ca_base = db.aero_db['ca0b_vs_mach'](mach) 
    else:
        ca_base = 0.0 # Engine is off, no reduction, full base drag is applied
        
    total_ca = ca0 + caa + ca_base + cad
    
    print("AERODYNAMIC DATA:")
    print(f"  Basic C_A (C_A0)    : {ca0:.4f}")
    print(f"  Alpha C_A (C_Aa)    : {caa:.4f}")
    print(f"  Fin C_A (C_Ad)      : {cad:.4f}")
    print(f"  Base Drag Reduction : {ca_base:.4f} (Engine Status: {'Active' if thrust>0 else 'Off'})")
    print(f"  TOTAL AXIAL DRAG C_A: {total_ca:.4f}\n")
    
    # Pitch Moment Example
    cm0 = db.aero_db['clm0_vs_mach_alpha_beta'](query)[0]
    cmdq = db.aero_db['clmdq_vs_mach_alpha_beta'](query)[0]
    
    # Pitch moment generated by fin deflection
    total_cm = cm0 + (cmdq * delta_q)
    print(f"  Basic Pitch Moment  : {cm0:.4f}")
    print(f"  Fin Pitch Moment    : {(cmdq * delta_q):.4f}")
    print(f"  TOTAL C_m           : {total_cm:.4f}")