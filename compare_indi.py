"""
INDI LPF karşılaştırması: eski (1-aşamalı) vs yeni (2-aşamalı) faz eşleştirme.
Her 10 senaryo için miss distance tablosu üretir.
"""
import sys
import importlib
import config
import simulate

SCENARIOS = list(range(1, 11))

SCENARIO_NAMES = {
    1:  "Straight Level",
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

def run_all_scenarios(two_stage: bool) -> dict:
    config.INDI_TWO_STAGE = two_stage
    # control module reads config.INDI_TWO_STAGE at runtime, no reload needed
    results = {}
    for s in SCENARIOS:
        try:
            _, result, cpa = simulate.run(scenario=s, cfg={'maut': 6, 'hit_r': 0}, verbose=False)
            results[s] = cpa['min_dist']
        except Exception as e:
            results[s] = float('nan')
            print(f"  S{s} HATA: {e}")
    return results

def main():
    print("Eski INDI (1-aşamalı LPF) çalışıyor...")
    old = run_all_scenarios(two_stage=False)

    print("Yeni INDI (2-aşamalı LPF) çalışıyor...")
    new = run_all_scenarios(two_stage=True)

    # Tabloyu yazdır
    col_w = 14
    header = (
        f"{'Senaryo':<25}"
        f"{'Eski [m]':>{col_w}}"
        f"{'Yeni [m]':>{col_w}}"
        f"{'Fark [m]':>{col_w}}"
        f"{'Değişim':>{col_w}}"
    )
    sep = "─" * len(header)

    print()
    print(sep)
    print(header)
    print(sep)

    improvements = 0
    for s in SCENARIOS:
        o = old[s]
        n = new[s]
        diff = n - o
        if o > 0:
            pct = (n - o) / o * 100.0
            pct_str = f"{pct:+.1f}%"
        else:
            pct_str = "—"

        arrow = "↓ DAHA İYİ" if diff < -0.1 else ("↑ daha kötü" if diff > 0.1 else "≈ eşit")
        if diff < -0.1:
            improvements += 1

        name = f"S{s}: {SCENARIO_NAMES[s]}"
        print(
            f"{name:<25}"
            f"{o:>{col_w}.2f}"
            f"{n:>{col_w}.2f}"
            f"{diff:>{col_w}.2f}"
            f"{pct_str:>{col_w}}  {arrow}"
        )

    print(sep)
    total_old = sum(v for v in old.values() if v == v)
    total_new = sum(v for v in new.values() if v == v)
    print(
        f"{'TOPLAM / ORTALAMA':<25}"
        f"{total_old/len(SCENARIOS):>{col_w}.2f}"
        f"{total_new/len(SCENARIOS):>{col_w}.2f}"
        f"{(total_new-total_old)/len(SCENARIOS):>{col_w}.2f}"
    )
    print(sep)
    print(f"\n{improvements}/10 senaryoda yeni INDI daha iyi miss distance verdi.")

    # Restore default
    config.INDI_TWO_STAGE = True

if __name__ == '__main__':
    main()
