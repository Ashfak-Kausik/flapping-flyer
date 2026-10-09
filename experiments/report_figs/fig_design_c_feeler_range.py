"""
fig_design_c_feeler_range.py — PANEL C: strike-free completion vs feeler range limit.

Sources (read-only, NO re-simulation, no statistics recomputed beyond counting rows):
  * outputs/degraded_feeler.csv.partial.jsonl — committed checkpoint of e54_degraded_feeler.py
    (306 flights, range axis only; no final outputs/degraded_feeler.csv exists). Rows with
    axis == "range", levels 0.12 / 0.08 / 0.05 / 0.03 m.
  * outputs/stats_pass1.csv, seed_idx == 1 — the undegraded reference (antenna reach 0.16 m), the
    same slice e54_analysis.py uses as its "idealized" row (same courses, same noise seeds).

Counting follows e54_analysis.py / docs/truewall_results.md Table T4: paired by course — a course
counts at a level only if BOTH variants have a record there. The counts are asserted against T4.
The 0.03 m level is incomplete (32 of 40 courses paired) and is drawn with open markers and a
dotted connector; every point is labelled with its own n/N.

Run: python experiments/report_figs/fig_design_c_feeler_range.py
"""
import sys; sys.path.insert(0, '.')
import csv, json
import matplotlib.pyplot as plt
from fig_design_common import apply_style, save, K, G

SRC_JSONL = "outputs/degraded_feeler.csv.partial.jsonl"
SRC_BASE = "outputs/stats_pass1.csv"
LEVELS = [0.16, 0.12, 0.08, 0.05, 0.03]
VARIANTS = ["FULL", "NO_WINGWASH"]
# docs/truewall_results.md Table T4: level -> (paired courses, FULL strike-free, NO_WINGWASH strike-free)
T4 = {0.16: (40, 32, 32), 0.12: (40, 30, 31), 0.08: (40, 31, 31), 0.05: (40, 25, 18), 0.03: (32, 0, 1)}


def load():
    P = {l: {} for l in LEVELS}
    for r in csv.DictReader(open(SRC_BASE)):
        if r["seed_idx"] == "1" and r["variant"] in VARIANTS:
            P[0.16].setdefault(int(r["course_id"]), {})[r["variant"]] = int(r["strike_free"])
    nrec = 0
    for line in open(SRC_JSONL):
        r = json.loads(line); nrec += 1
        if r["axis"] == "range":
            P[float(r["level"])].setdefault(int(r["course_id"]), {})[r["variant"]] = int(r["strike_free"])
    print(f"Sources: {SRC_JSONL} ({nrec} records), {SRC_BASE} (seed_idx 1 slice for 0.16 m)")
    out = {}
    print("range_m | paired courses | FULL strike-free | NO_WINGWASH strike-free | excluded (unpaired) course ids")
    for l in LEVELS:
        paired = {k: v for k, v in P[l].items() if len(v) == 2}
        miss = sorted(set(range(40)) - set(paired))
        n = len(paired); f = sum(v["FULL"] for v in paired.values()); w = sum(v["NO_WINGWASH"] for v in paired.values())
        assert (n, f, w) == T4[l], f"level {l}: counted {(n, f, w)} but Table T4 says {T4[l]}"
        out[l] = (n, f, w)
        print(f"  {l:.2f}  |  {n:2d}  |  {f:2d}/{n}  |  {w:2d}/{n}  |  {miss if miss else '-'}")
    print("all counts match docs/truewall_results.md Table T4")
    return out


def draw(ax, D):
    full = [l for l in LEVELS if D[l][0] == 40]; part = [l for l in LEVELS if D[l][0] != 40]
    pct = lambda l, i: 100.0 * D[l][i] / D[l][0]
    spec = {"FULL": dict(i=1, color=K, ls="-", marker="o", ms=5),
            "NO_WINGWASH": dict(i=2, color=G, ls=(0, (5, 2)), marker="s", ms=4.6)}
    for name, s in spec.items():
        ax.plot(full, [pct(l, s["i"]) for l in full], color=s["color"], ls=s["ls"], lw=1.3, marker=s["marker"],
                ms=s["ms"], mfc=s["color"], mec=s["color"], label=name.replace("NO_WINGWASH", "NO_WINGWASH (feelers only)")
                .replace("FULL", "FULL (wing-wash + feelers)") if name == "FULL" else "NO_WINGWASH (feelers only)", zorder=4)
        for l in part:
            ax.plot([full[-1], l], [pct(full[-1], s["i"]), pct(l, s["i"])], color=s["color"], ls=(0, (1, 2)), lw=1.1, zorder=3)
            ax.plot(l, pct(l, s["i"]), marker=s["marker"], ms=s["ms"], mfc="white", mec=s["color"], mew=1.2, ls="none", zorder=5)
    for l in LEVELS:                                           # n/N labels: higher value above, lower below
        n, f, w = D[l]
        if f == w:                                             # tie: one label for both
            ax.annotate(f"{f}/{n} (both)", (l, 100.0 * f / n), xytext=(0, 5.5), textcoords="offset points",
                        ha="center", va="bottom", fontsize=7, color=K, zorder=6)
            continue
        top, bot = (("FULL", f), ("NO_WINGWASH", w)) if f > w else (("NO_WINGWASH", w), ("FULL", f))
        if n != 40:                                            # partial level: labels to the right of the markers
            place = ((top, (6, 4), "left", "bottom"), (bot, (6, -4), "left", "top"))
        else:
            place = ((top, (0, 5.5), "center", "bottom"), (bot, (-4, -5), "right", "top"))
        for (name, v), off, ha, va in place:
            ax.annotate(f"{v}/{n}", (l, 100.0 * v / n), xytext=off, textcoords="offset points", ha=ha, va=va,
                        fontsize=7, color=spec[name]["color"], zorder=6)
    ax.plot([], [], marker="o", ms=5, mfc="white", mec=K, mew=1.2, ls=(0, (1, 2)), color=K,
            label=f"partial level: {D[part[0]][0]}/40 courses flown" if part else None)
    ax.set_xlim(0.182, 0.010); ax.set_xticks(LEVELS); ax.set_xticklabels([f"{l:.2f}" for l in LEVELS])
    ax.set_ylim(-14, 104); ax.set_yticks([0, 25, 50, 75, 100])
    ax.set_xlabel("feeler range limit (m)   (0.16 = undegraded antenna reach)")
    ax.set_ylabel("strike-free completion (%)")
    ax.grid(axis="y", color="#dddddd", lw=0.5, zorder=0)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    ax.legend(loc="lower left", frameon=False, handlelength=2.6, borderaxespad=0.3)


if __name__ == "__main__":
    apply_style()
    D = load()
    fig = plt.figure(figsize=(4.2, 3.0)); ax = fig.add_axes([0.14, 0.16, 0.83, 0.80])
    draw(ax, D)
    save(fig, "design_c_feeler_range")
