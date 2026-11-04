"""
fig_design_a_courses.py — PANEL A: course diversity (primary ensemble), true sampled layouts.

Source (read-only, NO simulation): the randomized-course generator itself.
  experiments/e51_baselines.generate_courses(N=40, hard=False) is the factored-out copy of the
  e50.batch() sampling loop: np.random.default_rng(1234), e50.sample_course(), e47.build_geometry(),
  rejection on e50.valid() and >=1 opening. It only builds geometry; nothing is flown here.
  Regenerated courses are cross-checked against the committed outputs/e51_baselines_truewall.csv
  (n_legs, n_openings for all 40 course ids) before anything is drawn.

Selection rule (deterministic, not hand-picked): the primary ensemble contains exactly six distinct
turn sequences (E-N, E-S, E-N-E, E-N-W, E-S-E, E-S-W); the LOWEST course id of each is drawn.

Drawn per course, all from geo = e47.build_geometry(legs, breaches): left/right wall polylines
geo['Lv'], geo['Rv']; openings as the gaps [f0, f1] along wall segment V[leg] -> V[leg+1] (same
construction as e47.layout_figure); start geo['P'][0]; finish pad of radius e36.FIN_PAD at
geo['finish']. All six are drawn at one common scale.

Run: python experiments/report_figs/fig_design_a_courses.py
"""
import sys; sys.path.insert(0, '.')
import csv
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mp
import experiments.e51_baselines as e51
import experiments.e36_reactive_course as e36
from fig_design_common import apply_style, save, K, G, LG

GEN_SEED = 1234                       # e50.batch / e51.generate_courses: default_rng(1234 + (1 if hard else 0))
CHECK_CSV = "outputs/e51_baselines_truewall.csv"
MM = 1e3


def load_courses():
    courses = e51.generate_courses(N=40, hard=False)
    with open(CHECK_CSV) as fh:
        rows = [r for r in csv.DictReader(fh.read().split("\n\n")[0].splitlines()) if r["variant"] == "FULL"]
    ref = {int(r["course_id"]): (int(r["n_legs"]), int(r["n_openings"])) for r in rows}
    got = {k: (len(legs), len(br)) for k, legs, br, _ in courses}
    assert got == ref, "regenerated courses do not match the committed per-course CSV"
    print(f"generator: e51.generate_courses(N=40, hard=False), default_rng({GEN_SEED}); "
          f"{len(courses)} courses; n_legs/n_openings match {CHECK_CSV} for all {len(ref)} ids")
    return courses


def select(courses):
    first = {}
    for k, legs, br, geo in courses:
        first.setdefault("-".join(d for d, _, _ in legs), (k, legs, br, geo))
    counts = {}
    for _, legs, _, _ in courses:
        s = "-".join(d for d, _, _ in legs); counts[s] = counts.get(s, 0) + 1
    print("turn sequences in the 40-course primary ensemble (count): "
          + ", ".join(f"{s} ({n})" for s, n in sorted(counts.items(), key=lambda kv: (len(kv[0]), kv[0]))))
    sel = sorted(first.values(), key=lambda c: (len(c[1]), c[0]))
    for k, legs, br, geo in sel:
        print(f"\ncourse_id {k}  (turn sequence {'-'.join(d for d, _, _ in legs)}, {len(legs)} legs, "
              f"{len(legs) - 1} turn(s), {len(br)} opening(s))")
        for i, (d, L, w) in enumerate(legs):
            print(f"   leg {i}: dir {d}  length {L * MM:6.1f} mm  width {w * MM:5.1f} mm")
        for (s, li, f0, f1) in br:
            print(f"   opening: {'left' if s == 1 else 'right'} wall, leg {li}, frac [{f0:.3f}, {f1:.3f}] "
                  f"-> {(f1 - f0) * legs[li][1] * MM:.1f} mm of leg length")
        print(f"   finish at ({geo['finish'][0] * MM:.1f}, {geo['finish'][1] * MM:.1f}) mm")
    return sel


def _bbox(geo):
    pts = np.array(list(geo["Lv"]) + list(geo["Rv"])); f = np.array(geo["finish"]); r = e36.FIN_PAD
    lo = np.minimum(pts.min(0), f - r); hi = np.maximum(pts.max(0), f + r)
    return lo * MM, hi * MM


def draw_course(ax, course, half):
    k, legs, br, geo = course
    Lv = np.array(geo["Lv"]) * MM; Rv = np.array(geo["Rv"]) * MM; P = np.array(geo["P"]) * MM
    ax.add_patch(plt.Polygon(np.vstack([Lv, Rv[::-1]]), closed=True, fc=LG, ec="none", zorder=1))
    ax.plot(P[:, 0], P[:, 1], color=G, lw=0.5, ls=(0, (2, 2)), zorder=2)
    for side, V in ((+1, Lv), (-1, Rv)):
        for i in range(len(legs)):
            a, b = V[i], V[i + 1]
            gaps = sorted((f0, f1) for (s, li, f0, f1) in br if s == side and li == i)
            cur = 0.0
            for f0, f1 in gaps + [(1.0, 1.0)]:
                if f0 > cur:
                    p, q = a + (b - a) * cur, a + (b - a) * f0
                    ax.plot([p[0], q[0]], [p[1], q[1]], color=K, lw=1.5, solid_capstyle="butt", zorder=4)
                cur = max(cur, f1)
            for f0, f1 in gaps:                               # opening: white break + outward arrow
                p, q = a + (b - a) * f0, a + (b - a) * f1; mid = (p + q) / 2
                t = (b - a) / np.linalg.norm(b - a); n_out = np.array([-t[1], t[0]])
                if np.dot(n_out, mid - (P[i] + P[i + 1]) / 2) < 0: n_out = -n_out
                ax.plot([p[0], q[0]], [p[1], q[1]], color="white", lw=2.6, solid_capstyle="butt", zorder=3)
                for e_ in (p, q):
                    ax.plot([e_[0] - 9 * n_out[0], e_[0] + 9 * n_out[0]], [e_[1] - 9 * n_out[1], e_[1] + 9 * n_out[1]],
                            color=K, lw=0.9, zorder=5)
                ax.annotate("", xy=mid + 34 * n_out, xytext=mid - 4 * n_out, zorder=6,
                            arrowprops=dict(arrowstyle="-|>", lw=0.9, color=K, shrinkA=0, shrinkB=0, mutation_scale=6))
    ax.plot(*P[0], "o", ms=3.6, color=K, zorder=7)
    ax.add_patch(mp.Circle(P[-1], e36.FIN_PAD * MM, fc="white", ec=K, lw=0.9, zorder=6))
    ax.plot(*P[-1], "x", ms=3.2, mew=0.9, color=K, zorder=7)
    lo, hi = _bbox(geo); c = (lo + hi) / 2
    ax.set_xlim(c[0] - half[0], c[0] + half[0]); ax.set_ylim(c[1] - half[1], c[1] + half[1])
    ax.set_aspect("equal"); ax.axis("off")
    ax.set_title(f"course {k}\n{len(legs)} legs, {len(br)} opening{'s' if len(br) != 1 else ''}", fontsize=7.5, pad=2,
                 linespacing=1.25)


def draw(axes, sel):
    ext = [np.subtract(*_bbox(c[3])[::-1]) for c in sel]
    half = np.max(ext, axis=0) / 2 + np.array([10.0, 40.0])    # common scale; margin for the opening arrows
    for ax, c in zip(axes, sel):
        draw_course(ax, c, half)
    ax0 = axes[0]; x0, y0 = ax0.get_xlim()[0] + 8, ax0.get_ylim()[0] + 10
    ax0.plot([x0, x0 + 100], [y0, y0], color=K, lw=1.6, solid_capstyle="butt")
    ax0.text(x0 + 50, y0 + 9, "100 mm", ha="center", va="bottom", fontsize=7)
    print(f"\ncommon view box per course: {2 * half[0]:.0f} x {2 * half[1]:.0f} mm; finish pad radius "
          f"{e36.FIN_PAD * MM:.0f} mm (e36.FIN_PAD); scale bar 100 mm")


def legend_handles():
    L = plt.Line2D
    return [L([], [], color=K, lw=1.5, label="wall"),
            L([], [], color=K, lw=0, marker=r"$\rightarrow$", ms=9, label="wall opening (gap in wall)"),
            L([], [], color=G, lw=0.6, ls=(0, (2, 2)), label="centreline"),
            L([], [], color=K, lw=0, marker="o", ms=3.6, label="start"),
            L([], [], color=K, lw=0, marker="o", mfc="white", ms=7, label="finish pad")]


if __name__ == "__main__":
    apply_style()
    sel = select(load_courses())
    fig, axs = plt.subplots(2, 3, figsize=(7.1, 4.6))
    fig.subplots_adjust(left=0.01, right=0.99, top=0.93, bottom=0.08, wspace=0.03, hspace=0.22)
    draw(list(axs.ravel()), sel)
    fig.legend(handles=legend_handles(), loc="lower center", ncol=5, frameon=False, handlelength=1.6, columnspacing=1.4)
    save(fig, "design_a_courses")
