"""
fig_f_observer_gain.py — FIGURE F (optional): observer gain vs wall distance.

Source (read-only, no re-simulation): outputs/e42_observer_fidelity.csv, FIRST block
(d_mm,d_eff_mm,est_rad_s2,true_rad_s2,gain,ydrift_mm).

NOTE: the CSV's second block (bw_base_rad_s2,bw_final_rad_s2,rise_10_90_ms) is a
SCALAR triple (base level, final level, one 10-90% rise-time number), not a
time-series trace. There is no step-response trace (roll_dist vs t) committed
anywhere in outputs/ for e42, so the step-response panel the task asked for CANNOT
be plotted from committed data without re-running e42_observer_fidelity.py's
bandwidth() function. Per instructions, this script therefore produces ONLY the
gain-vs-distance panel; the scalar rise-time is printed and annotated as text, not
fabricated as a curve.

Run: python experiments/report_figs/fig_f_observer_gain.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SRC_CSV = "outputs/e42_observer_fidelity.csv"
OUT_PNG = "outputs/report/fig_f_observer_gain.png"


def load_blocks(path):
    with open(path) as fh:  # universal-newline mode: this CSV is CRLF
        text = fh.read()
    blocks = [b for b in text.split("\n\n") if b.strip()]
    assert len(blocks) == 2, f"expected 2 blocks in {path}, found {len(blocks)}"
    fid_rows = list(csv.DictReader(blocks[0].strip().splitlines()))
    bw_row = list(csv.DictReader(blocks[1].strip().splitlines()))[0]
    return fid_rows, bw_row


def main():
    fid_rows, bw_row = load_blocks(SRC_CSV)
    print(f"Source CSV: {SRC_CSV}")
    print("\nblock 1 (gain vs distance):")
    print("d_mm, d_eff_mm, est_rad_s2, true_rad_s2, gain, ydrift_mm")
    d = np.array([float(r["d_mm"]) for r in fid_rows])
    gain = np.array([float(r["gain"]) for r in fid_rows])
    for r in fid_rows:
        print(f"  {float(r['d_mm']):5.1f} {float(r['d_eff_mm']):7.3f} "
              f"{float(r['est_rad_s2']):10.3f} {float(r['true_rad_s2']):10.3f} "
              f"{float(r['gain']):.4f} {float(r['ydrift_mm']):+.3f}")

    print("\nblock 2 (scalar bandwidth/rise-time triple -- NOT a time trace):")
    print(f"  bw_base_rad_s2={float(bw_row['bw_base_rad_s2']):.4f}  "
          f"bw_final_rad_s2={float(bw_row['bw_final_rad_s2']):.4f}  "
          f"rise_10_90_ms={float(bw_row['rise_10_90_ms']):.6f}")
    print("\n*** No step-response TIME TRACE (roll_dist vs t) is committed in outputs/. ***")
    print("*** Only this gain-vs-distance panel is plotted; the step-response panel is skipped. ***")

    fig, ax = plt.subplots(figsize=(7, 5.5))
    fig.patch.set_facecolor("white")

    flat_mask = (d >= 15) & (d <= 30)
    ax.plot(d, gain, "o-", color="#1d6fb8", lw=1.6, ms=6, label="observer gain (est/true)")
    ax.axhspan(gain[flat_mask].min(), gain[flat_mask].max(), color="#a9d4a0", alpha=0.35,
              label=f"flat band 15-30mm (gain {gain[flat_mask].min():.3f}-{gain[flat_mask].max():.3f})")
    for xi, yi in zip(d, gain):
        ax.annotate(f"{yi:.3f}", (xi, yi), textcoords="offset points", xytext=(0, 7),
                    ha="center", fontsize=8)

    ax.set_xlabel("wall distance d (mm)")
    ax.set_ylabel("gain = estimated / true roll disturbance")
    ax.set_title("Disturbance-observer fidelity: gain vs wall distance\n"
                 f"(rise time 10-90% = {float(bw_row['rise_10_90_ms']):.1f} ms, "
                 "step response trace not in committed CSV)")
    ax.set_ylim(0, max(gain) * 1.2)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8.5, loc="lower right")
    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=300)
    print(f"\nsaved -> {OUT_PNG}")


if __name__ == "__main__":
    main()
