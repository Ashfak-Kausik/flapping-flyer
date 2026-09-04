"""
fig_b_clearance_boxplot.py — FIGURE B: min-clearance distribution, FULL vs NO_WINGWASH.

Source (read-only, no re-simulation): outputs/e51b_ablation_followup.csv, rows with
part=="part1" -- the 40 primary courses re-run under both variants with per-course
min-clearance instrumentation (e51b_followup.py, part1_clearance()).

WINGREACH (wingtip-strike radius) = 13.3 mm (src/safety.py: WINGREACH = 0.0133 m).
NEAR_STRIKE_MM = WINGREACH + 5.0 = 18.3 mm (e51b_followup.py:26-27) -- the "within
5mm of the strike radius" band shaded here is [13.3, 18.3) mm.

Run: python experiments/report_figs/fig_b_clearance_boxplot.py
"""
import sys; sys.path.insert(0, '.')
import csv
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

SRC_CSV = "outputs/e51b_ablation_followup.csv"
OUT_PDF = "outputs/report/fig_b_clearance_boxplot.pdf"

WINGREACH_MM = 13.3
NEAR_STRIKE_MM = 18.3

VARIANT_ORDER = ["FULL", "NO_WINGWASH"]
COLORS = {"FULL": "#1d6fb8", "NO_WINGWASH": "#f59e0b"}


def load_part1(path):
    with open(path) as fh:  # universal-newline mode: this CSV is CRLF
        rows = list(csv.DictReader(fh))
    part1 = [r for r in rows if r["part"] == "part1"]
    by_variant = {v: [] for v in VARIANT_ORDER}
    for r in part1:
        by_variant[r["variant"]].append(r)
    return by_variant


def main():
    by_variant = load_part1(SRC_CSV)

    print(f"Source CSV: {SRC_CSV}, rows with part=='part1' (40 primary courses x 2 variants)")
    stats = {}
    for v in VARIANT_ORDER:
        rows = by_variant[v]
        clear = np.array([float(r["min_clear_mm"]) for r in rows])
        crashed = sum(int(r["crashed"]) for r in rows)
        completed = sum(int(r["completed"]) for r in rows)
        near = int(np.sum(clear < NEAR_STRIKE_MM))
        stats[v] = dict(clear=clear, crashed=crashed, completed=completed, near=near)
        print(f"\n{v} (n={len(rows)}):")
        print("  course_id, completed, crashed, min_clear_mm")
        for r in sorted(rows, key=lambda r: int(r["course_id"])):
            print(f"    {r['course_id']:>2s}  {r['completed']}  {r['crashed']}  {float(r['min_clear_mm']):.4f}")
        print(f"  -> mean={clear.mean():.3f}  std={clear.std():.3f}  min={clear.min():.3f}  "
              f"crashed={crashed}/{len(rows)}  within {NEAR_STRIKE_MM}mm of strike={near}/{len(rows)}")

    fig, ax = plt.subplots(figsize=(8, 6.2))
    fig.patch.set_facecolor("white")

    # shaded near-strike band and strike-radius line
    ax.axhspan(WINGREACH_MM, NEAR_STRIKE_MM, color="#f4b6ab", alpha=0.45, zorder=0,
               label=f"within 5mm of strike ({WINGREACH_MM}–{NEAR_STRIKE_MM} mm)")
    ax.axhline(WINGREACH_MM, color="#c0392b", ls="--", lw=1.6, zorder=1,
               label=f"wingtip-strike radius ({WINGREACH_MM} mm)")

    data = [stats[v]["clear"] for v in VARIANT_ORDER]
    bp = ax.boxplot(data, positions=[1, 2], widths=0.5, patch_artist=True,
                     showmeans=True, meanline=True,
                     medianprops=dict(color="black", lw=1.4),
                     meanprops=dict(color="#333333", ls=":", lw=1.4),
                     flierprops=dict(marker="o", markersize=4, markerfacecolor="none"))
    for patch, v in zip(bp["boxes"], VARIANT_ORDER):
        patch.set_facecolor(COLORS[v]); patch.set_alpha(0.55)

    rng = np.random.default_rng(0)
    for i, v in enumerate(VARIANT_ORDER, start=1):
        jitter = rng.uniform(-0.12, 0.12, size=len(stats[v]["clear"]))
        ax.scatter(np.full_like(stats[v]["clear"], i) + jitter, stats[v]["clear"],
                   s=14, color=COLORS[v], edgecolor="white", linewidth=0.4, zorder=3)

    ax.set_xticks([1, 2])
    ax.set_xticklabels([f"{v}\n(min={stats[v]['clear'].min():.1f}mm, "
                         f"crashed={stats[v]['crashed']}/{len(stats[v]['clear'])}, "
                         f"within-5mm={stats[v]['near']}/{len(stats[v]['clear'])})"
                         for v in VARIANT_ORDER], fontsize=8.5)
    ax.set_ylabel("min clearance over the course (mm)")
    ax.set_title("Per-course minimum clearance: FULL vs NO_WINGWASH\n"
                 "(e51b part1, 40 primary courses, level=1.0 noise, rate=1000 Hz)")
    ax.grid(axis="y", alpha=0.25)
    ax.legend(fontsize=8, loc="upper right")
    fig.tight_layout()
    fig.savefig(OUT_PDF, dpi=300)
    print(f"\nsaved -> {OUT_PDF}")


if __name__ == "__main__":
    main()
