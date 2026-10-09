"""
fig_design_common.py — shared style + save helper for the design/parameter-space figure
(fig_design_a_courses.py, fig_design_b_kge_grid.py, fig_design_c_feeler_range.py,
fig_design_assemble.py). No data, no simulation: presentation only.
"""
import os
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

OUT_DIR = "outputs/report/design_fig"
K = "#111111"; G = "#666666"; LG = "#e6e6e6"

# same family as the other report figures (matplotlib default, DejaVu Sans); sized for a
# two-column (figure*) width of ~7.1 in
STYLE = {"font.family": "DejaVu Sans", "font.size": 8, "axes.labelsize": 8, "axes.titlesize": 8,
         "xtick.labelsize": 7.5, "ytick.labelsize": 7.5, "legend.fontsize": 7.5,
         "axes.linewidth": 0.8, "pdf.fonttype": 42, "mathtext.fontset": "dejavusans"}


def apply_style():
    plt.rcParams.update(STYLE)


def save(fig, stem):
    os.makedirs(OUT_DIR, exist_ok=True)
    for ext in ("png", "pdf"):
        fig.savefig(f"{OUT_DIR}/{stem}.{ext}", dpi=400, facecolor="white")
    plt.close(fig)
    print(f"saved -> {OUT_DIR}/{stem}.png, .pdf")
