"""
fig_cd_e52_sweeps.py — FIGURES C and D: e52 width-sweep and leg-count-sweep deltas.

Source (read-only, no re-simulation): outputs/e52_wingwash_scaling.csv, which contains
three comma-blocks:
  block1 (per-run)   : factor,cell,variant,course_id,...            [not used here]
  block2 (per-cell)  : factor,cell,variant,runs,completed,crashed,clear_mean_mm,...
  block3 (delta)     : factor,cell,comp_delta_FULL_minus_NOWW,crash_delta,...

FIGURE C -- width sweep (5 cells, legs fixed at 3): top panel = completed count,
FULL vs NO_WINGWASH, per width band; bottom panel = comp_delta (FULL - NO_WINGWASH)
with a zero line, to show the delta sign-flips and is non-monotonic.

FIGURE D -- leg-count sweep (5 cells, width fixed 58-64mm): same structure, x-axis
is leg count.

Run: python experiments/report_figs/fig_cd_e52_sweeps.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SRC_CSV = "outputs/e52_wingwash_scaling.csv"
OUT_PNG_C = "outputs/report/fig_c_e52_width_sweep.png"
OUT_PNG_D = "outputs/report/fig_d_e52_legs_sweep.png"

COLORS = {"FULL": "#1d6fb8", "NO_WINGWASH": "#f59e0b"}
WIDTH_CELL_ORDER = ["70-76", "64-70", "58-64", "52-58", "46-52"]
LEGS_CELL_ORDER = ["2", "3", "4", "5", "6"]


def load_blocks(path):
    with open(path) as fh:  # universal-newline mode: this CSV is CRLF
        text = fh.read()
    blocks = [b for b in text.split("\n\n") if b.strip()]
    assert len(blocks) == 3, f"expected 3 blocks in {path}, found {len(blocks)}"
    per_cell = list(csv.DictReader(blocks[1].strip().splitlines()))
    delta = list(csv.DictReader(blocks[2].strip().splitlines()))
    return per_cell, delta


def make_panel(factor, cell_order, xlabel, out_pdf, title, per_cell, delta):
    cells = [r for r in per_cell if r["factor"] == factor]
    deltas = [r for r in delta if r["factor"] == factor]
    by_cell_variant = {(r["cell"], r["variant"]): r for r in cells}
    delta_by_cell = {r["cell"]: r for r in deltas}

    print(f"\n--- {factor} sweep ({out_pdf}) ---")
    print("cell, variant, runs, completed, crashed, clear_mean_mm, clear_min_mm")
    for c in cell_order:
        for v in ("FULL", "NO_WINGWASH"):
            r = by_cell_variant[(c, v)]
            print(f"  {c:>6s} {v:12s} {r['runs']:>3s} {r['completed']:>3s} {r['crashed']:>3s} "
                  f"{float(r['clear_mean_mm']):7.3f} {float(r['clear_min_mm']):7.3f}")
    print("cell, comp_delta_FULL_minus_NOWW, crash_delta, clear_mean_delta_mm, clear_min_delta_mm")
    for c in cell_order:
        d = delta_by_cell[c]
        print(f"  {c:>6s} {d['comp_delta_FULL_minus_NOWW']:>4s} {d['crash_delta']:>4s} "
              f"{float(d['clear_mean_delta_mm']):7.3f} {float(d['clear_min_delta_mm']):7.3f}")

    x = np.arange(len(cell_order))
    width = 0.35

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8), sharex=True,
                                    gridspec_kw=dict(height_ratios=[1.3, 1]))
    fig.patch.set_facecolor("white")

    for i, v in enumerate(("FULL", "NO_WINGWASH")):
        comps = [int(by_cell_variant[(c, v)]["completed"]) for c in cell_order]
        offs = x + (i - 0.5) * width
        bars = ax1.bar(offs, comps, width=width, color=COLORS[v], label=v,
                       edgecolor="white", linewidth=0.6)
        for bar, val in zip(bars, comps):
            ax1.annotate(f"{val}", (bar.get_x() + bar.get_width()/2, bar.get_height()),
                        textcoords="offset points", xytext=(0, 2), ha="center", fontsize=8)
    ax1.set_ylabel("courses completed (out of 20)")
    ax1.set_title(title)
    ax1.legend(fontsize=9)
    ax1.grid(axis="y", alpha=0.25)
    ax1.set_ylim(0, 21)

    comp_deltas = [int(delta_by_cell[c]["comp_delta_FULL_minus_NOWW"]) for c in cell_order]
    bar_colors = ["#1d6fb8" if d >= 0 else "#f59e0b" for d in comp_deltas]
    bars2 = ax2.bar(x, comp_deltas, width=0.5, color=bar_colors, edgecolor="white", linewidth=0.6)
    for bar, val in zip(bars2, comp_deltas):
        ax2.annotate(f"{val:+d}", (bar.get_x() + bar.get_width()/2, bar.get_height()),
                    textcoords="offset points", xytext=(0, 3 if val >= 0 else -11),
                    ha="center", fontsize=8.5)
    ax2.axhline(0, color="#333333", lw=1.0)
    ax2.set_ylabel("Δ completion\n(FULL − NO_WINGWASH)")
    ax2.set_xlabel(xlabel)
    ax2.set_xticks(x)
    ax2.set_xticklabels(cell_order)
    ax2.grid(axis="y", alpha=0.25)
    ymax = max(abs(d) for d in comp_deltas) + 1
    ax2.set_ylim(-ymax, ymax)

    fig.tight_layout()
    fig.savefig(out_pdf, dpi=300)
    print(f"saved -> {out_pdf}")


def main():
    per_cell, delta = load_blocks(SRC_CSV)
    print(f"Source CSV: {SRC_CSV} (per-cell block + delta block)")

    make_panel("width", WIDTH_CELL_ORDER, "corridor width band (mm), legs fixed at 3",
               OUT_PNG_C, "e52 width sweep: completion FULL vs NO_WINGWASH, and the delta\n"
                         "(non-monotonic, sign-flipping)",
               per_cell, delta)
    make_panel("legs", LEGS_CELL_ORDER, "leg count, width fixed 58-64mm",
               OUT_PNG_D, "e52 leg-count sweep: completion FULL vs NO_WINGWASH, and the delta\n"
                         "(non-monotonic, sign-flipping)",
               per_cell, delta)


if __name__ == "__main__":
    main()
