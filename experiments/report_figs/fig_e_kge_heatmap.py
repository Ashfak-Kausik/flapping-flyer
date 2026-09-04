"""
fig_e_kge_heatmap.py — FIGURE E: closed-loop K_GE x control-rate completion grid.

Source (read-only, no re-simulation): outputs/e44_kge_sensitivity.csv, SECOND
(closed-loop) block: columns K_GE,rate_hz,reached,n_seeds,crashed,mean_clear_mm.

3x3 grid, rows K_GE in {0.5,1.0,1.5}, cols rate in {1000,500,250} Hz. Cell value =
reached/n_seeds (all n_seeds=3 in this CSV). Annotated with the "reached/n_seeds"
count and, where all 3 seeds completed, the mean clearance (mm). The single 0/3 cell
(K_GE=1.5, 500Hz, where all 3 seeds crashed) gets a distinct color from the rest.

Run: python experiments/report_figs/fig_e_kge_heatmap.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors
import numpy as np

SRC_CSV = "outputs/e44_kge_sensitivity.csv"
OUT_PNG = "outputs/report/fig_e_kge_heatmap.png"

KGE_ORDER = ["0.5", "1.0", "1.5"]
RATE_ORDER = ["1000", "500", "250"]


def load_closedloop_block(path):
    with open(path) as fh:  # universal-newline mode: this CSV is CRLF
        text = fh.read()
    blocks = [b for b in text.split("\n\n") if b.strip()]
    assert len(blocks) == 2, f"expected 2 blocks in {path}, found {len(blocks)}"
    rows = list(csv.DictReader(blocks[1].strip().splitlines()))
    grid = {}
    for r in rows:
        kge = f"{float(r['K_GE']):.1f}"
        rate = r["rate_hz"]
        grid[(kge, rate)] = r
    return grid


def main():
    grid = load_closedloop_block(SRC_CSV)
    print(f"Source CSV: {SRC_CSV} (closed-loop block)")
    print("K_GE, rate_hz, reached, n_seeds, crashed, mean_clear_mm")
    for kge in KGE_ORDER:
        for rate in RATE_ORDER:
            r = grid[(kge, rate)]
            mc = r["mean_clear_mm"]
            mc_s = f"{float(mc):.3f}" if mc not in ("", None) else "n/a (0 completed)"
            print(f"  {kge:>4s} {rate:>6s} {r['reached']:>3s} {r['n_seeds']:>3s} "
                  f"{r['crashed']:>3s}  {mc_s}")

    n_rows, n_cols = len(KGE_ORDER), len(RATE_ORDER)
    completed = np.zeros((n_rows, n_cols), dtype=int)
    for i, kge in enumerate(KGE_ORDER):
        for j, rate in enumerate(RATE_ORDER):
            completed[i, j] = int(grid[(kge, rate)]["reached"])

    fig, ax = plt.subplots(figsize=(7, 6))
    fig.patch.set_facecolor("white")

    # colormap: green shades for 3/3 and 2/3-ish, distinct red for the 0/3 cell
    cmap = mcolors.ListedColormap(["#c0392b", "#f4a582", "#a9d4a0", "#2e9e4f"])
    bounds = [-0.5, 0.5, 1.5, 2.5, 3.5]
    norm = mcolors.BoundaryNorm(bounds, cmap.N)

    im = ax.imshow(completed, cmap=cmap, norm=norm, aspect="equal")

    for i, kge in enumerate(KGE_ORDER):
        for j, rate in enumerate(RATE_ORDER):
            r = grid[(kge, rate)]
            n = int(r["reached"]); ns = int(r["n_seeds"])
            mc = r["mean_clear_mm"]
            label = f"{n}/{ns}"
            if mc not in ("", None):
                label += f"\n{float(mc):.1f}mm"
            else:
                label += "\n(crashed)"
            color = "white" if n == 0 else "black"
            ax.text(j, i, label, ha="center", va="center", fontsize=10.5, color=color,
                    fontweight="bold" if n == 0 else "normal")

    ax.set_xticks(range(n_cols)); ax.set_xticklabels([f"{r} Hz" for r in RATE_ORDER])
    ax.set_yticks(range(n_rows)); ax.set_yticklabels([f"K_GE={k}" for k in KGE_ORDER])
    ax.set_xlabel("control rate")
    ax.set_ylabel("ground-effect coefficient")
    ax.set_title("K_GE x control-rate closed-loop grid: completion (reached/n_seeds)\n"
                 "cell label: reached/3, mean clearance where all 3 completed")
    for edge in ("top", "right", "bottom", "left"):
        ax.spines[edge].set_visible(False)
    ax.set_xticks(np.arange(-0.5, n_cols, 1), minor=True)
    ax.set_yticks(np.arange(-0.5, n_rows, 1), minor=True)
    ax.grid(which="minor", color="white", linewidth=2)
    ax.tick_params(which="minor", length=0)

    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=300)
    print(f"\nsaved -> {OUT_PNG}")


if __name__ == "__main__":
    main()
