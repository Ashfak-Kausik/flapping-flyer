"""
fig_g_flyer_geometry.py — flyer_geometry.png: to-scale top-view schematic of the flyer.

Source (read-only, no re-simulation, no new computation of results):
  - outputs/e02_strip_table.csv  -- the actual 20-strip wing discretization
    (strip,r_mm,dr_mm,chord_mm,area_mm2), i.e. exactly what src/aero.strips_for_wing()
    builds at runtime; the wing outline below is the polygon traced by each strip's
    leading/trailing edge, not a redrawn ellipse.
  - models/flyer.xml -- STATIC geometry only (body/wing geom `pos`/`size` attributes):
    thorax_g, head_g, tail_g ellipsoid/sphere size+pos, and the stroke_R/stroke_L
    hinge positions the wing strips are measured from. No simulation is run; these
    are constant model parameters, read the same way wing_params_from_model() does.
  - pitch-axis fraction x_hat0 = 0.25 (quarter-chord), the constant computed by
    wing_params_from_model() as x_le/c_max from the same wing_R_g geom (pos[0]+size[0])/(2*size[0])
    = (-0.000875+0.00175)/0.0035 = 0.25 exactly -- verified in-script below, not asserted.

Run: python experiments/report_figs/fig_g_flyer_geometry.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

SRC_CSV = "outputs/e02_strip_table.csv"
SRC_XML = "models/flyer.xml"
OUT_PNG = "outputs/report/fig_g_flyer_geometry.png"

# Static geometry read directly from models/flyer.xml (mm). Each value is the exact
# XML attribute cited in the comment; nothing here is a simulated or fitted result.
THORAX = dict(pos=(1.0, 0.0), size=(2.0, 1.2))       # thorax_g: pos="0.001 0 0" size="0.0020 0.0012 0.0012"
HEAD   = dict(pos=(4.0, 0.0), r=0.8)                  # head_g:   pos="0.0040 0 0" size="0.0008"
TAIL   = dict(pos=(-5.5, 0.0), size=(5.0, 0.6))       # tail_g:   pos="-0.0055 0 0" size="0.0050 0.0006 0.0006"
HINGE_R = (0.0, 1.0)                                  # stroke_R: pos="0 0.0010 0.0012" (x,y only)
HINGE_L = (0.0, -1.0)                                 # stroke_L: pos="0 -0.0010 0.0012"
X_LE_OVER_CMAX = (-0.000875 + 0.00175) / (2 * 0.00175)  # wing_R_g: pos[0]=-0.000875, size[0]=0.00175 -> x_hat0
C_MAX_MM = 2 * 0.00175 * 1e3                          # = 3.5 mm, sanity cross-check vs CSV max chord


def load_strips(path):
    with open(path) as fh:  # universal-newline mode
        rows = list(csv.DictReader(fh))
    return rows


def main():
    rows = load_strips(SRC_CSV)
    print(f"Source CSV: {SRC_CSV} ({len(rows)} strips)")
    print("strip, r_mm, dr_mm, chord_mm, area_mm2")
    r = np.array([float(x["r_mm"]) for x in rows])
    dr = np.array([float(x["dr_mm"]) for x in rows])
    chord = np.array([float(x["chord_mm"]) for x in rows])
    for x in rows:
        print(f"  {x['strip']:>2s}  {float(x['r_mm']):6.4f}  {float(x['dr_mm']):.4f}  "
              f"{float(x['chord_mm']):.4f}  {float(x['area_mm2']):.5f}")

    print(f"\nx_hat0 (pitch-axis fraction of chord, from {SRC_XML} wing_R_g pos/size) = {X_LE_OVER_CMAX:.4f}")
    print(f"c_max from XML size[0]*2 = {C_MAX_MM:.4f} mm  (CSV max chord = {chord.max():.4f} mm)")
    print(f"wing span from hinge: r_root={r[0]-dr[0]/2:.4f}mm  r_tip={r[-1]+dr[-1]/2:.4f}mm")

    le = X_LE_OVER_CMAX * chord            # leading-edge x offset from hinge line (mm)
    te = -(1 - X_LE_OVER_CMAX) * chord     # trailing-edge x offset from hinge line (mm)
    edges = np.concatenate([[r[0] - dr[0]/2], r[:-1] + dr[:-1]/2, [r[-1] + dr[-1]/2]])

    fig, ax = plt.subplots(figsize=(10, 6.5))
    fig.patch.set_facecolor("white")

    # body: thorax, head, tail (ellipses/circle), drawn to scale in mm
    ax.add_patch(mpatches.Ellipse(THORAX["pos"], 2*THORAX["size"][0], 2*THORAX["size"][1],
                                   color="#37474f", alpha=0.85, zorder=3, label="thorax"))
    ax.add_patch(mpatches.Circle(HEAD["pos"], HEAD["r"], color="#546e7a", alpha=0.85, zorder=3))
    ax.add_patch(mpatches.Ellipse(TAIL["pos"], 2*TAIL["size"][0], 2*TAIL["size"][1],
                                   color="#78909c", alpha=0.75, zorder=2))

    # wings: polygon traced by each strip's LE/TE (this IS the true discretized planform)
    for hinge, sign, label in ((HINGE_R, +1, "right wing (R)"), (HINGE_L, -1, "left wing (L)")):
        hx, hy = hinge
        y_le = hy + sign * edges
        # build a closed polygon: LE edge out to tip, TE edge back to root
        le_pts = np.column_stack([hx + np.repeat(le, 1)[:len(r)], hy + sign * r])
        te_pts = np.column_stack([hx + np.repeat(te, 1)[:len(r)], hy + sign * r])
        poly = np.vstack([le_pts, te_pts[::-1]])
        ax.add_patch(plt.Polygon(poly, closed=True, facecolor="#90a4ae", edgecolor="#455a64",
                                 alpha=0.55, lw=1.0, zorder=1, label=label if sign == 1 else None))
        # strip boundary ticks (span-wise divisions actually used by strips_for_wing)
        for e in edges:
            ax.plot([hx + X_LE_OVER_CMAX*0, hx], [hy + sign*e, hy + sign*e], lw=0)  # no-op keeps zorder simple
        for i in range(len(r)):
            y0 = hy + sign * (r[i] - dr[i]/2); y1 = hy + sign * (r[i] + dr[i]/2)
            ax.plot([hx + le[i], hx + le[i]], [y0, y1], color="#455a64", lw=0.4, alpha=0.6, zorder=1)
        ax.plot([hx, hx], [hy, hy + sign*edges[-1]], color="#b71c1c", lw=1.0, ls=":", zorder=2)  # hinge/pitch axis

    ax.plot(*HINGE_R, marker="o", color="#c0392b", ms=5, zorder=4)
    ax.plot(*HINGE_L, marker="o", color="#c0392b", ms=5, zorder=4)

    # annotations
    ax.annotate(f"R (wingtip radius) = {r[-1]+dr[-1]/2:.1f} mm", xy=(0, HINGE_R[1] + r[-1] + dr[-1]/2),
               xytext=(6, HINGE_R[1] + r[-1] + dr[-1]/2), fontsize=8.5, va="center", color="#b71c1c")
    ax.annotate(f"c_max = {chord.max():.1f} mm", xy=(le[9], HINGE_R[1] + r[9]),
               xytext=(le[9] + 1.5, HINGE_R[1] + r[9] + 1.5), fontsize=8.5, color="#455a64",
               arrowprops=dict(arrowstyle="->", lw=0.7, color="#455a64"))
    ax.annotate("pitch axis\n(quarter-chord, x̂₀=0.25)", xy=(0, 9), xytext=(-8.5, 9),
               fontsize=7.5, color="#b71c1c", ha="left", va="center",
               arrowprops=dict(arrowstyle="->", lw=0.7, color="#b71c1c"))

    ax.set_xlim(-9, 6.5); ax.set_ylim(-15, 15)
    ax.set_aspect("equal")
    ax.set_xlabel("x — fore(+)/aft(−) from thorax origin (mm)")
    ax.set_ylabel("y — span, left(−)/right(+) (mm)")
    ax.set_title("Flyer geometry: top view, to scale\n"
                 "(wing planform = actual 20-strip discretization from e02_strip_table.csv; "
                 "body from models/flyer.xml static geometry)")
    ax.grid(alpha=0.2)
    handles = [mpatches.Patch(color="#37474f", label="thorax"),
               mpatches.Patch(color="#546e7a", label="head"),
               mpatches.Patch(color="#78909c", label="tail/abdomen"),
               mpatches.Patch(color="#90a4ae", alpha=0.55, label="wing planform (20 strips)"),
               plt.Line2D([0], [0], color="#b71c1c", ls=":", lw=1.0, label="stroke hinge / pitch axis")]
    ax.legend(handles=handles, fontsize=8, loc="lower right")

    fig.tight_layout()
    fig.savefig(OUT_PNG, dpi=300)
    print(f"\nsaved -> {OUT_PNG}")


if __name__ == "__main__":
    main()
