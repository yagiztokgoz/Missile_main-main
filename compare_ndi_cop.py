"""
================================================================================
 NDI  vs  NDI-CoP  —  Karşılaştırma ve Monte Carlo Analizi
================================================================================

KULLANIM
  python3 compare_ndi_cop.py              # deterministik, tüm senaryolar
  python3 compare_ndi_cop.py mc           # MC tüm senaryolar, varsayılan n_runs
  python3 compare_ndi_cop.py mc 50        # MC tüm senaryolar, 50 run
  python3 compare_ndi_cop.py mc 3         # MC sadece senaryo 3
  python3 compare_ndi_cop.py mc 3 50      # MC senaryo 3, 50 run

SENARYOLAR  (tümü mseek=0, mins=0/1 koşullara göre)
  NDI     — maut=5 : standart Nonlinear Dynamic Inversion (CG referansı)
  NDI-CoP — maut=7 : NDI, Center of Percussion referansı (NMP sıfırı giderilmiş)

MONTE CARLO
  Her run'da:
    • Aerodinamik türevler bağımsız N(1, σ²) faktörle bozulur
      (fiziksel kuvvetler bozulmaz — sadece kontrolcünün model kopyası)
    • INS gürültüsü eklenir (mins=1)
  Aynı bozulma vektörü her iki varianta eşit uygulanır → adil karşılaştırma.
================================================================================
"""

import sys
import math
import numpy as np
import config
import simulate
from target import Target

# ── Sabitler ──────────────────────────────────────────────────────────────────
SCENARIOS = list(range(1, 11))

SCENARIO_NAMES = {
    1:  "Straight & Level",
    2:  "Accel Escape",
    3:  "Sinusoidal Jink",
    4:  "Hard L-Turn",
    5:  "Beaming Notch",
    6:  "Head-On Break",
    7:  "Look-Down",
    8:  "Last-Ditch",
    9:  "Energy-Bleed Climb",
    10: "Coordinated Turn",
}

VARIANTS = {
    'NDI':     {'maut': 5, 'mseek': 0, 'mins': 0},
    'NDI-CoP': {'maut': 7, 'mseek': 0, 'mins': 0},
}

# Aero bozulma parametreleri
AERO_SIGMA  = config.MC_AERO_SIGMA    # varsayılan %15
AERO_PARAMS = config.MC_AERO_PARAMS
N_RUNS      = config.MC_N_RUNS


# ── Yardımcı ──────────────────────────────────────────────────────────────────

def _arrow(diff):
    if diff < -0.05:
        return "↓ iyi"
    if diff >  0.05:
        return "↑ kötü"
    return "≈"


def _pct(new, old):
    if old > 0:
        return f"{(new - old) / old * 100:+.1f}%"
    return "—"


# ══════════════════════════════════════════════════════════════════════════════
#  1. DETERMİNİSTİK KARŞILAŞTIRMA
# ══════════════════════════════════════════════════════════════════════════════

def run_deterministic(scenarios=None, aero_sigma=0.0):
    """
    Her senaryo için NDI ve NDI-CoP'u birer kez çalıştırır.
    aero_sigma > 0 ise sabit seed'li tek bir aero bozulması eklenir
    (deterministik ama stresli koşul).
    """
    if scenarios is None:
        scenarios = SCENARIOS

    rng = np.random.default_rng(config.MC_BASE_SEED or 0)

    # sabit aero bozulması (sigma > 0 ise)
    if aero_sigma > 0:
        aero_pert = {p: 1.0 + aero_sigma * float(rng.standard_normal())
                     for p in AERO_PARAMS}
    else:
        aero_pert = {}

    results = {lbl: {} for lbl in VARIANTS}

    for s in scenarios:
        for label, base_cfg in VARIANTS.items():
            cfg = dict(base_cfg)
            cfg['hit_r'] = 0.0
            if aero_pert:
                cfg['aero_pert'] = aero_pert
            try:
                _, _, cpa = simulate.run(scenario=s, cfg=cfg, verbose=False)
                results[label][s] = cpa['min_dist']
            except Exception as e:
                print(f"  [HATA] S{s} {label}: {e}")
                results[label][s] = float('nan')

    _print_deterministic_table(results, scenarios, aero_sigma)
    return results


def _print_deterministic_table(results, scenarios, aero_sigma):
    lbl_a, lbl_b = 'NDI', 'NDI-CoP'
    col = 12
    sigma_str = f"σ={aero_sigma:.0%}" if aero_sigma > 0 else "nominal (σ=0)"
    title = f"DETERMİNİSTİK KARŞILAŞTIRMA  [{sigma_str}]"

    header = (f"  {'Senaryo':<22}"
              f"{lbl_a:>{col}}"
              f"{lbl_b:>{col}}"
              f"{'Fark [m]':>{col}}"
              f"{'Değişim':>{col}}"
              f"   Sonuç")
    sep = "─" * len(header)

    print(f"\n{'═'*len(header)}")
    print(f"  {title}")
    print(f"{'═'*len(header)}")
    print(header)
    print(sep)

    wins_cop = 0
    valid_a, valid_b = [], []

    for s in scenarios:
        a = results[lbl_a].get(s, float('nan'))
        b = results[lbl_b].get(s, float('nan'))
        diff = b - a
        name = f"S{s}: {SCENARIO_NAMES[s]}"
        arrow = _arrow(diff)
        if diff < -0.05:
            wins_cop += 1
        if not math.isnan(a):
            valid_a.append(a)
        if not math.isnan(b):
            valid_b.append(b)

        a_str   = f"{a:.3f}" if not math.isnan(a) else "ERR"
        b_str   = f"{b:.3f}" if not math.isnan(b) else "ERR"
        d_str   = f"{diff:+.3f}" if not math.isnan(diff) else "—"
        pct_str = _pct(b, a)

        print(f"  {name:<22}"
              f"{a_str:>{col}}"
              f"{b_str:>{col}}"
              f"{d_str:>{col}}"
              f"{pct_str:>{col}}"
              f"   {arrow}")

    print(sep)
    mean_a = sum(valid_a) / len(valid_a) if valid_a else float('nan')
    mean_b = sum(valid_b) / len(valid_b) if valid_b else float('nan')
    mean_d = mean_b - mean_a
    print(f"  {'ORTALAMA':<22}"
          f"{mean_a:>{col}.3f}"
          f"{mean_b:>{col}.3f}"
          f"{mean_d:>{col}+.3f}"
          f"{_pct(mean_b, mean_a):>{col}}")
    print(f"{'═'*len(header)}")
    print(f"  NDI-CoP daha iyi: {wins_cop}/{len(scenarios)} senaryo\n")


# ══════════════════════════════════════════════════════════════════════════════
#  2. MONTE CARLO KARŞILAŞTIRMA
# ══════════════════════════════════════════════════════════════════════════════

def run_mc_scenario(scenario, n_runs=None, aero_sigma=None):
    """Tek senaryo için NDI vs NDI-CoP Monte Carlo."""
    if n_runs    is None: n_runs    = N_RUNS
    if aero_sigma is None: aero_sigma = AERO_SIGMA

    mc_variants = {
        'NDI':     {'maut': 5, 'mseek': 0},
        'NDI-CoP': {'maut': 7, 'mseek': 0},
    }

    res = simulate.monte_carlo(
        scenario=scenario,
        variants=mc_variants,
        n_runs=n_runs,
        aero_sigma=aero_sigma,
        aero_params=AERO_PARAMS,
        ins_noise=True,
        base_seed=config.MC_BASE_SEED,
        save=True,
        show=False,
    )
    return res


def run_mc_all(scenarios=None, n_runs=None, aero_sigma=None):
    """Tüm senaryolar için NDI vs NDI-CoP Monte Carlo + özet CEP tablosu."""
    if scenarios  is None: scenarios  = SCENARIOS
    if n_runs     is None: n_runs     = N_RUNS
    if aero_sigma is None: aero_sigma = AERO_SIGMA

    all_res = []
    for s in scenarios:
        print(f"\n{'─'*60}")
        print(f"  Senaryo {s}: {SCENARIO_NAMES[s]}")
        print(f"{'─'*60}")
        res = run_mc_scenario(s, n_runs=n_runs, aero_sigma=aero_sigma)
        all_res.append((s, res))

    _print_mc_summary(all_res, n_runs, aero_sigma)
    return all_res


def _print_mc_summary(all_res, n_runs, aero_sigma):
    labels = ['NDI', 'NDI-CoP']
    col = 10

    print(f"\n{'═'*82}")
    print(f"  MONTE CARLO ÖZET  —  CEP [m]  "
          f"n={n_runs}  σ_aero={aero_sigma:.0%}  INS=açık")
    print(f"{'═'*82}")

    # Başlık satırı
    hdr = (f"  {'S':<4}  {'Senaryo':<22}"
           + "".join(f"{'CEP':>{col}} {'P90':>{col}}" for _ in labels)
           + f"  {'Δ CEP':>{col}}  Kazanan")
    print(hdr)
    print("  " + "─" * (len(hdr) - 2))

    ndi_ceps, cop_ceps = [], []
    wins_cop = 0

    for s, res in all_res:
        ndi_cep = res['NDI']['cep']
        cop_cep = res['NDI-CoP']['cep']
        ndi_p90 = res['NDI']['p90']
        cop_p90 = res['NDI-CoP']['p90']
        delta   = cop_cep - ndi_cep
        winner  = "NDI-CoP" if cop_cep < ndi_cep else ("NDI" if ndi_cep < cop_cep else "eşit")
        if cop_cep < ndi_cep:
            wins_cop += 1
        ndi_ceps.append(ndi_cep)
        cop_ceps.append(cop_cep)

        print(f"  {s:<4}  {SCENARIO_NAMES[s]:<22}"
              f"  {ndi_cep:>{col}.2f} {ndi_p90:>{col}.2f}"
              f"  {cop_cep:>{col}.2f} {cop_p90:>{col}.2f}"
              f"  {delta:>{col}+.2f}  {winner}")

    print("  " + "─" * (len(hdr) - 2))
    mean_ndi = sum(ndi_ceps) / len(ndi_ceps) if ndi_ceps else float('nan')
    mean_cop = sum(cop_ceps) / len(cop_ceps) if cop_ceps else float('nan')
    print(f"  {'—':<4}  {'ORTALAMA':<22}"
          f"  {mean_ndi:>{col}.2f} {'':>{col}}"
          f"  {mean_cop:>{col}.2f} {'':>{col}}"
          f"  {mean_cop - mean_ndi:>{col}+.2f}")
    print(f"{'═'*82}")
    print(f"  NDI-CoP kazandı: {wins_cop}/{len(all_res)} senaryo  "
          f"({wins_cop/len(all_res)*100:.0f}%)")
    print(f"{'═'*82}\n")

    _print_mc_detail_table(all_res)


def _print_mc_detail_table(all_res):
    """Her senaryo için tam istatistik tablosu (Mean / Std / CEP / P90 / P95 / Pk<5m)."""
    labels = ['NDI', 'NDI-CoP']
    stats  = ['mean', 'std', 'cep', 'p90', 'p95', 'pk5']

    print(f"\n{'═'*90}")
    print(f"  DETAYLI İSTATİSTİKLER")
    print(f"{'═'*90}")
    print(f"  {'':>4}  {'Variant':<10}"
          f"  {'Mean':>7}  {'Std':>6}  {'CEP':>7}  "
          f"{'P90':>7}  {'P95':>7}  {'Pk<5m':>7}")
    print("  " + "─" * 86)

    for s, res in all_res:
        first = True
        for lbl in labels:
            d = res[lbl]
            prefix = f"  {s:<4}" if first else f"  {'':>4}"
            first  = False
            print(f"{prefix}  {lbl:<10}"
                  f"  {d['mean']:>7.2f}  {d['std']:>6.2f}  {d['cep']:>7.2f}  "
                  f"{d['p90']:>7.2f}  {d['p95']:>7.2f}  {d['pk5']:>6.1f}%")
        print("  " + "─" * 86)

    print(f"{'═'*90}\n")


# ══════════════════════════════════════════════════════════════════════════════
#  3. CLI ENTRY POINT
# ══════════════════════════════════════════════════════════════════════════════

def _parse_args():
    """
    Kabul edilen formlar:
      (yok)            → deterministik, tüm senaryolar
      mc               → MC tüm senaryolar, varsayılan n_runs
      mc <n>           → MC tüm senaryolar, n run (n > 10 ise run sayısı)
                         ya da MC senaryo n (n ≤ 10 ise senaryo)
      mc <s> <n>       → MC senaryo s, n run
    """
    argv = sys.argv[1:]

    if not argv:
        return 'det', None, None

    if argv[0] != 'mc':
        print(__doc__)
        sys.exit(0)

    ints = [int(t) for t in argv[1:] if t.lstrip('-').isdigit()]

    if len(ints) == 0:
        return 'mc_all', None, None
    if len(ints) == 1:
        # senaryo (1-10) mi, run sayısı mı?
        if 1 <= ints[0] <= 10:
            return 'mc_one', ints[0], None
        else:
            return 'mc_all', None, ints[0]
    # iki integer: senaryo + run sayısı
    return 'mc_one', ints[0], ints[1]


def main():
    mode, scenario, n_runs = _parse_args()

    if mode == 'det':
        print("\n── Nominal (σ=0) ────────────────────────────────────────────────────────")
        run_deterministic(aero_sigma=0.0)
        print(f"\n── Bozulmuş aero (σ={AERO_SIGMA:.0%}, sabit seed) ─────────────────────────────")
        run_deterministic(aero_sigma=AERO_SIGMA)

    elif mode == 'mc_one':
        run_mc_scenario(scenario=scenario, n_runs=n_runs)

    elif mode == 'mc_all':
        run_mc_all(n_runs=n_runs)


if __name__ == '__main__':
    main()
