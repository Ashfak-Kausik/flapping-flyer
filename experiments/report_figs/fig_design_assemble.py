"""
fig_design_assemble.py — assemble panels (a), (b), (c) into one two-column (figure*) figure.

Pure composition: calls the load()/draw() functions of the three panel scripts, which read
  (a) the course generator (e51.generate_courses, default_rng(1234)); geometry only
  (b) outputs/e44_kge_sensitivity.csv (closed-loop block)
  (c) outputs/degraded_feeler.csv.partial.jsonl + outputs/stats_pass1.csv (seed_idx 1)
No simulation is run and no result is recomputed.

Run: python experiments/report_figs/fig_design_assemble.py
"""
import sys; sys.path.insert(0, '.')
import matplotlib.pyplot as plt
from fig_design_common import apply_style, save
import fig_design_a_courses as A
import fig_design_b_kge_grid as B
import fig_design_c_feeler_range as C

if __name__ == "__main__":
    apply_style()
    print("===== panel (a) ====="); sel = A.select(A.load_courses())
    print("\n===== panel (b) ====="); grid = B.load()
    print("\n===== panel (c) ====="); D = C.load()

    fig = plt.figure(figsize=(7.1, 4.75))
    axa = [fig.add_axes([0.012 + i * 0.1645, 0.735, 0.158, 0.19]) for i in range(6)]
    A.draw(axa, sel)
    fig.legend(handles=A.legend_handles(), loc="center", bbox_to_anchor=(0.5, 0.695), ncol=5, frameon=False,
               handlelength=1.6, columnspacing=1.4)
    axb = fig.add_axes([0.075, 0.085, 0.27, 0.4036]); B.draw(axb, grid)
    axc = fig.add_axes([0.475, 0.105, 0.51, 0.48]); C.draw(axc, D)
    for s, x, y in (("(a)", 0.008, 0.99), ("(b)", 0.008, 0.635), ("(c)", 0.405, 0.635)):
        fig.text(x, y, s, fontsize=10, fontweight="bold", ha="left", va="top")
    save(fig, "design_fig")
