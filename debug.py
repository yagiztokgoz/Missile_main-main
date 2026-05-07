import simulate
import pandas as pd

log = simulate.run(scenario=1)
df = pd.read_csv('logs/sim_log_s1.csv')
t_check = [1.0, 2.0, 3.0, 4.0, 5.0]
for t in t_check:
    row = df.iloc[(df['t']-t).abs().argsort()[:1]]
    print(f"t={t}: dvbe={row['dvbe'].values[0]:.3f}, hbe={row['hbe'].values[0]:.3f}, "
          f"alphax={row['alphax'].values[0]:.4f}, dqx={row['dqx'].values[0]:.4f}, "
          f"ancomx={row['ancomx'].values[0]:.4f}")



import sys
sys.path.insert(0, '.')
import config
import math
from missile.missile import Missile
from missile.database import SRAAM6_Database

# Veritabanını yükle
db = SRAAM6_Database()
db.load_aero_deck(config.AERO_DECK)
db.load_propulsion_deck(config.PROP_DECK)

# t=1s'deki değerleri elle ver (Python log'dan)
mach    = 2.18   # dvbe=643/vsound_10km≈295
alphax  = 0.667
betax   = 0.0
pdynmc  = 84000.0
dvbe    = 643.0
mass    = 75.0   # yaklaşık t=1s
xcg     = 1.45
xcgref  = config.XCG_REF
ai33    = 50.0   # yaklaşık
refl    = config.REFL
refa    = config.REFA
DEG     = 180/math.pi

# clmdq LUT'tan oku
clmdq = float(db.aero_db['clmdq_vs_mach_alpha_beta']([mach, alphax, betax])[0])

# dmd hesapla
dmd = DEG * (pdynmc * refa * refl / ai33) * clmdq

print(f"clmdq = {clmdq:.6f} /deg")
print(f"dmd   = {dmd:.4f} /s²")


row = df.iloc[(df['t']-1.0).abs().argsort()[:1]]
print(f"qqx={row['qqx'].values[0]:.4f} deg/s")
print(f"fspb3={row['fspb3'].values[0]:.4f}")

ai33_t1 = float(db.prop_db['moipitch_vs_time'](1.0))
ai11_t1 = float(db.prop_db['moiroll_vs_time'](1.0))
print(f"ai33 at t=1s: {ai33_t1:.4f}")
print(f"ai11 at t=1s: {ai11_t1:.4f}")

# t=1s tahmin
pdynmc_1 = 0.5 * 0.414 * 643**2  # ~85000 Pa
qSr_1 = pdynmc_1 * 0.01824 * 0.1524
print(f"qSr ≈ {qSr_1:.1f}")


# Python'da t=1s Fm'i hesapla
# qqd ≈ (qqx(t=1.01) - qqx(t=0.99)) / 0.02
row_a = df.iloc[(df['t']-0.99).abs().argsort()[:1]]
row_b = df.iloc[(df['t']-1.01).abs().argsort()[:1]]
qqd_py = (row_b['qqx'].values[0] - row_a['qqx'].values[0]) / 0.02  # deg/s²
ai33_t1 = float(db.prop_db['moipitch_vs_time'](1.0))
Fm_py = qqd_py * (math.pi/180) * ai33_t1
print(f"qqd_py = {qqd_py:.4f} deg/s²")
print(f"Fm_py  = {Fm_py:.4f} N·m")


print("*"*80)

log = simulate.run(scenario=1)
df = pd.read_csv('logs/sim_log_s1.csv')
for t in [0.5, 1.0, 2.0]:
    row = df.iloc[(df['t']-t).abs().argsort()[:1]]
    print(f"t={t}: qqx={row['qqx'].values[0]:.4f} alphax={row['alphax'].values[0]:.4f} hbe={row['hbe'].values[0]:.3f}")

xcg_t1 = float(db.prop_db['cg_vs_time'](1.0))
print(f"xcg at t=1s: {xcg_t1:.4f} m")
print(f"xcg_ref: {config.XCG_REF:.4f} m")
print(f"xcg - xcg_ref = {xcg_t1 - config.XCG_REF:.4f} m")


mach, alpha, beta = 2.0, 0.0, 0.0
clldp = float(db.aero_db['clldp_vs_mach_alpha_beta']([mach, alpha, beta])[0])
print(f"clldp = {clldp:.6f}")  # pozitif mi negatif mi?


for t in [0.5, 1.0, 2.0]:
    row = df.iloc[(df['t']-t).abs().argsort()[:1]]
    print(f"t={t}: phiblx={row['phiblx'].values[0]:.3f}  dpx={row['dpx'].values[0]:.4f}")
