"""
fig_kge_grid_truewall.py — K_GE x control-rate completion grid, TRUE-WALL re-run (e55).

Source (read-only, NO re-simulation): outputs/e44_kge_sensitivity_truewall.csv, first (summary) block,
columns K_GE,rate_hz,reached,strike_free,n_seeds,crashed,mean_clear_mm_reached,level — the e38 crafted
course flown after the true-wall fix at nominal noise (level 1.0), five noise seeds per cell.
Same layout and monochrome style as fig_design_b_kge_grid.py (which stays on the pre-fix CSV);
cell = seeds reaching the finish / n_seeds, shaded by the fraction that did not.

Run: python experiments/report_figs/fig_kge_grid_truewall.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib.pyplot as plt
from fig_design_common import apply_style, save, K
from fig_e_kge_heatmap import KGE_ORDER, RATE_ORDER

SRC_CSV = "outputs/e44_kge_sensitivity_truewall.csv"


def load():
    with open(SRC_CSV) as fh:
        block = [b for b in fh.read().split("\n\n") if b.strip()][0]
    grid = {(f"{float(r['K_GE']):.1f}", r["rate_hz"]): r for r in csv.DictReader(block.strip().splitlines())}
    print(f"Source CSV: {SRC_CSV} (summary block)")
    print("K_GE, rate_hz, reached, strike_free, n_seeds")
    for kge in KGE_ORDER:
        for rate in RATE_ORDER:
            r = grid[(kge, rate)]
            print(f"  {kge:>4s} {rate:>6s}  {r['reached']}/{r['n_seeds']}  strike-free {r['strike_free']}/{r['n_seeds']}")
    assert len({v["n_seeds"] for v in grid.values()}) == 1
    return grid


def draw(ax, grid):
    ns = int(next(iter(grid.values()))["n_seeds"])
    for i, kge in enumerate(KGE_ORDER):
        for j, rate in enumerate(RATE_ORDER):
            n = int(grid[(kge, rate)]["reached"]); miss = 1 - n / ns
            dark = miss > 0.5
            ax.add_patch(plt.Rectangle((j, i), 1, 1, fc=str(1 - 0.77 * miss), ec=K, lw=0.8,
                                       hatch="////" if dark else None, zorder=1))
            if dark:
                ax.add_patch(plt.Rectangle((j + 0.2, i + 0.28), 0.6, 0.44, fc=str(1 - 0.77 * miss), ec="none", zorder=2))
            ax.text(j + 0.5, i + 0.5, f"{n}/{ns}", ha="center", va="center", fontsize=10, zorder=3,
                    color="white" if dark else K, fontweight="bold" if miss > 0 else "normal")
    ax.set_xlim(0, 3); ax.set_ylim(3, 0); ax.set_aspect("equal")
    ax.set_xticks([0.5, 1.5, 2.5]); ax.set_xticklabels(RATE_ORDER)
    ax.set_yticks([0.5, 1.5, 2.5]); ax.set_yticklabels(KGE_ORDER)
    ax.tick_params(length=0); ax.xaxis.tick_top(); ax.xaxis.set_label_position("top")
    ax.set_xlabel("control rate (Hz)"); ax.set_ylabel(r"$K_{\mathrm{GE}}$")
    for s in ax.spines.values(): s.set_visible(False)
    ax.text(1.5, 3.12, f"cell = seeds reaching the finish / {ns}", ha="center", va="top", fontsize=7.5)


if __name__ == "__main__":
    apply_style()
    grid = load()
    fig = plt.figure(figsize=(2.9, 3.0)); ax = fig.add_axes([0.2, 0.1, 0.74, 0.72])
    draw(ax, grid)
    save(fig, "design_b_kge_grid_truewall")
