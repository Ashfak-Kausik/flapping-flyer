"""
fig_design_b_kge_grid.py — PANEL B: K_GE x control-rate closed-loop completion grid.

Source (read-only, NO re-simulation): outputs/e44_kge_sensitivity.csv, second (closed-loop) block,
columns K_GE,rate_hz,reached,n_seeds,crashed,mean_clear_mm — the same rows as fig_e_kge_heatmap.py,
loaded with that script's own loader. Same data as fig_e; restyled monochrome to match the other
panels of the design figure (cell = reached/n_seeds, i.e. noise seeds that reached the finish
on the e38 course at that K_GE and control rate; the 0/3 cell is dark + hatched).

Run: python experiments/report_figs/fig_design_b_kge_grid.py
"""
import sys; sys.path.insert(0, '.')
import matplotlib.pyplot as plt
from fig_design_common import apply_style, save, K
from fig_e_kge_heatmap import load_closedloop_block, SRC_CSV, KGE_ORDER, RATE_ORDER


def load():
    grid = load_closedloop_block(SRC_CSV)
    print(f"Source CSV: {SRC_CSV} (closed-loop block)")
    print("K_GE, rate_hz, reached, n_seeds, crashed")
    for kge in KGE_ORDER:
        for rate in RATE_ORDER:
            r = grid[(kge, rate)]
            print(f"  {kge:>4s} {rate:>6s}  {r['reached']}/{r['n_seeds']}  crashed={r['crashed']}")
    zero = [(k, r) for (k, r), v in grid.items() if int(v["reached"]) == 0]
    assert zero == [("1.5", "500")], f"expected exactly one 0/3 cell at K_GE=1.5, 500 Hz; found {zero}"
    assert all(int(v["n_seeds"]) == 3 for v in grid.values())
    return grid


def draw(ax, grid):
    for i, kge in enumerate(KGE_ORDER):
        for j, rate in enumerate(RATE_ORDER):
            r = grid[(kge, rate)]; n, ns = int(r["reached"]), int(r["n_seeds"])
            fail = n == 0
            ax.add_patch(plt.Rectangle((j, i), 1, 1, fc="#3a3a3a" if fail else "white", ec=K, lw=0.8,
                                       hatch="////" if fail else None, zorder=1))
            if fail:
                ax.add_patch(plt.Rectangle((j + 0.2, i + 0.28), 0.6, 0.44, fc="#3a3a3a", ec="none", zorder=2))
            ax.text(j + 0.5, i + 0.5, f"{n}/{ns}", ha="center", va="center", fontsize=10, zorder=3,
                    color="white" if fail else K, fontweight="bold" if fail else "normal")
    ax.set_xlim(0, 3); ax.set_ylim(3, 0); ax.set_aspect("equal")
    ax.set_xticks([0.5, 1.5, 2.5]); ax.set_xticklabels(RATE_ORDER)
    ax.set_yticks([0.5, 1.5, 2.5]); ax.set_yticklabels(KGE_ORDER)
    ax.tick_params(length=0); ax.xaxis.tick_top(); ax.xaxis.set_label_position("top")
    ax.set_xlabel("control rate (Hz)"); ax.set_ylabel(r"$K_{\mathrm{GE}}$")
    for s in ax.spines.values(): s.set_visible(False)
    ax.text(1.5, 3.12, "cell = seeds reaching the finish / 3", ha="center", va="top", fontsize=7.5)


if __name__ == "__main__":
    apply_style()
    grid = load()
    fig = plt.figure(figsize=(2.9, 3.0)); ax = fig.add_axes([0.2, 0.1, 0.74, 0.72])
    draw(ax, grid)
    save(fig, "design_b_kge_grid")
