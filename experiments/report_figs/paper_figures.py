"""
paper_figures.py — every figure for the paper draft, in one consistent style, into
outputs/paper_figures/ (PNG 400 dpi + vector PDF).

Read-only on committed data. Sources per figure are printed as it is drawn. Three figures use
traces that were never persisted to a committed CSV and are regenerated (existing experiment code,
unchanged) by paper_regen_data.py into outputs/paper_figures/data/ :
    fig03 open-loop hover trace, fig04 wing-wash distance sweep, fig09 example mission.
Figures 01 and 08 are copies of outputs/report/model_fig and outputs/report/design_fig.

Run: python experiments/report_figs/paper_figures.py            (all)
     python experiments/report_figs/paper_figures.py fig11 fig12 (some)
"""
import sys; sys.path.insert(0, '.'); sys.path.insert(0, 'experiments')
import csv, json, os, shutil
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mp
from fig_design_common import apply_style, K, G, LG
from e53_analysis import mcnemar_exact, B, RNG_SEED, MARGIN

OUT = "outputs/paper_figures"
DATA = f"{OUT}/data"
G2 = "#9a9a9a"
WINGREACH_MM = 13.3            # src/safety.py WINGREACH = 0.0133 m (as in fig_b_clearance_boxplot.py)
FULL_LBL, NOWW_LBL = "FULL (wing-wash + feelers)", "NO_WINGWASH (feelers only)"


def save(fig, stem):
    for ext in ("png", "pdf"):
        fig.savefig(f"{OUT}/{stem}.{ext}", dpi=400, facecolor="white")
    plt.close(fig); print(f"  saved -> {OUT}/{stem}.png, .pdf\n")


def blocks(path):
    """split a multi-block CSV (blank-line separated) into lists of DictReader rows."""
    return [list(csv.DictReader(b.strip().splitlines())) for b in open(path).read().split("\n\n") if b.strip()]


def clean(ax, grid="y"):
    for s in ("top", "right"): ax.spines[s].set_visible(False)
    if grid: ax.grid(axis=grid, color="#dddddd", lw=0.5, zorder=0)


def boot(M, stat):
    """cluster bootstrap over rows (courses), as e54_analysis.boot: B resamples, percentile, RNG_SEED."""
    rng = np.random.default_rng(RNG_SEED); n = len(M)
    out = np.array([stat(M[rng.integers(0, n, n)]) for _ in range(B)])
    return np.percentile(out, 2.5), np.percentile(out, 97.5)


# ------------------------------------------------------------------------------------------
def fig01():
    print("fig01 vehicle model: copies of outputs/report/model_fig/ (make_model_fig.py)")
    for src, dst in (("model_iso", "fig01a_model_iso"), ("model_wing_detail", "fig01b_model_wing_detail"),
                     ("model_sensing_geometry", "fig01c_model_sensing_geometry")):
        for ext in ("png", "pdf"):
            shutil.copyfile(f"outputs/report/model_fig/{src}.{ext}", f"{OUT}/{dst}.{ext}")
        print(f"  copied -> {OUT}/{dst}.png, .pdf")
    print()


def fig08():
    print("fig08 experimental design: copies of outputs/report/design_fig/ (fig_design_*.py)")
    for src, dst in (("design_fig", "fig08_design"), ("design_a_courses", "fig08a_design_courses"),
                     ("design_b_kge_grid", "fig08b_design_kge_grid"), ("design_c_feeler_range", "fig08c_design_feeler_range")):
        for ext in ("png", "pdf"):
            shutil.copyfile(f"outputs/report/design_fig/{src}.{ext}", f"{OUT}/{dst}.{ext}")
        print(f"  copied -> {OUT}/{dst}.png, .pdf")
    print()


# ------------------------------------------------------------------------------------------
def fig02():
    from fig_h_force_decomposition import load_decomposition, load_weight_uN, SRC_CSV, WEIGHT_CSV
    rows = load_decomposition(SRC_CSV); W = load_weight_uN(WEIGHT_CSV)
    t = np.array([float(r["t_ms"]) for r in rows])
    cols = {"trans_uN": ("translational", G, "-", 1.2), "rot_uN": ("rotational", K, (0, (4, 2)), 1.0),
            "added_uN": ("added mass", G2, (0, (1, 1.5)), 1.2), "total_uN": ("total", K, "-", 1.8)}
    print(f"fig02 force decomposition: {SRC_CSV} ({len(rows)} rows, t {t[0]:.1f}-{t[-1]:.1f} ms); weight line "
          f"{W:.1f} uN = trim Fz from {WEIGHT_CSV}")
    fig = plt.figure(figsize=(3.5, 2.9)); ax = fig.add_axes([0.17, 0.155, 0.80, 0.68])
    for c, (lab, col, ls, lw) in cols.items():
        y = np.array([float(r[c]) for r in rows]); ax.plot(t, y, color=col, ls=ls, lw=lw, label=lab)
        print(f"  {c}: mean {y.mean():.1f} uN, min {y.min():.1f}, max {y.max():.1f}")
    # e07 trace is the last two wingbeats of e07.run(F_REF=40 Hz), stroke amplitude 60 deg (NOT the 80 Hz hover point): mean total is
    # a small fraction of the weight, so the weight line is reported in the printout but not drawn.
    tot = np.array([float(r["total_uN"]) for r in rows])
    print(f"  mean total {tot.mean():.1f} uN = {100 * tot.mean() / W:.1f}% of weight ({W:.1f} uN): 40 Hz / 60 deg operating point, two wingbeats")
    ax.axhline(0, color=K, lw=0.5)
    ax.set_xlabel("time (ms)   (two wingbeats at 40 Hz, Φ = 60°)"); ax.set_ylabel("vertical force (µN)"); ax.set_xlim(t[0], t[-1])
    ax.set_ylim(-70, 280)
    ax.legend(frameon=False, ncol=2, loc="lower center", bbox_to_anchor=(0.5, 1.0), handlelength=2.4, columnspacing=1.2); clean(ax)
    save(fig, "fig02_force_decomposition")


def fig03():
    cl = list(csv.DictReader(open("outputs/e11_hover_control.csv")))
    ol = list(csv.DictReader(open(f"{DATA}/e11_open_loop.csv")))
    print(f"fig03 hover control: closed loop outputs/e11_hover_control.csv ({len(cl)} rows); open loop "
          f"{DATA}/e11_open_loop.csv (regenerated, {len(ol)} rows); gusts at 400 / 800 ms (e11 GUSTS)")
    g = lambda R, c: np.array([float(r[c]) for r in R])
    tc, to = g(cl, "t_ms"), g(ol, "t_ms")
    oP, oR, oH = g(ol, "pitch_deg"), g(ol, "roll_deg"), g(ol, "height_mm")
    bad = np.where((np.abs(oP) > 60) | (np.abs(oR) > 60))[0]          # same blanking rule as e11
    lost = to[np.where((np.abs(oP) > 45) | (np.abs(oR) > 45))[0][0]]
    if len(bad): oP[bad[0]:] = np.nan; oR[bad[0]:] = np.nan; oH[bad[0]:] = np.nan
    print(f"  open loop exceeds 45 deg at {lost:.0f} ms; closed loop final 200 ms: |pitch| < "
          f"{np.abs(g(cl, 'pitch_deg')[-2000:]).max():.1f} deg, |roll| < {np.abs(g(cl, 'roll_deg')[-2000:]).max():.1f} deg, "
          f"height {g(cl, 'height_mm')[-2000:].mean():.1f} mm")
    fig, axs = plt.subplots(3, 1, figsize=(3.5, 3.9), sharex=True)
    fig.subplots_adjust(left=0.17, right=0.95, top=0.98, bottom=0.11, hspace=0.12)
    for ax, c, o, yl, lim in ((axs[0], "pitch_deg", oP, "pitch (deg)", (-25, 45)), (axs[1], "roll_deg", oR, "roll (deg)", (-45, 25)),
                              (axs[2], "height_mm", oH, "height (mm)", (30, 60))):
        ax.plot(to, o, color=G, ls=(0, (4, 2)), lw=1.1, label="open loop (no control)")
        ax.plot(tc, g(cl, c), color=K, lw=1.1, label="closed loop (LQG)")
        for tg in (400, 800): ax.axvline(tg, color=G2, lw=0.6, ls=(0, (1, 2)))
        ax.set_ylabel(yl); ax.set_ylim(*lim); clean(ax)
    axs[2].legend(frameon=False, loc="lower right", handlelength=2.2)
    axs[0].text(410, 41, "roll gust", fontsize=7, color=G, va="top"); axs[0].text(810, 41, "pitch gust", fontsize=7, color=G, va="top")
    axs[2].set_xlabel("time (ms)"); axs[2].set_xlim(0, 1200)
    save(fig, "fig03_hover_control")


def fig04():
    rows = list(csv.DictReader(open(f"{DATA}/e41_sweep.csv")))
    d = np.array([float(r["d_mm"]) for r in rows]); T = np.abs([float(r["T_wall_nNm"]) for r in rows])
    a = np.abs([float(r["roll_acc_rad_s2"]) for r in rows])
    fits = {r["regime"]: r for r in csv.DictReader(open("outputs/e41_power_law_fits.csv"))}
    far = fits["far 20-40"]; nf = float(blocks("outputs/e41_resolution.csv")[0][0]["noise_floor_rad_s2"])
    print(f"fig04 wing-wash signal: {DATA}/e41_sweep.csv (regenerated e41.sweep(), {len(d)} distances "
          f"{d[0]:.0f}-{d[-1]:.0f} mm); far-field exponent {float(far['exponent']):.2f} (r2 {float(far['r2']):.3f}) from "
          f"outputs/e41_power_law_fits.csv; noise floor {nf:.1f} rad/s^2 from outputs/e41_resolution.csv")
    for di, Ti, ai in zip(d, T, a): print(f"  d {di:5.1f} mm  |T| {Ti:8.1f} nN.m  |roll acc| {ai:8.1f} rad/s^2")
    fig, axs = plt.subplots(1, 2, figsize=(7.1, 2.6)); fig.subplots_adjust(left=0.085, right=0.985, top=0.93, bottom=0.17, wspace=0.27)
    axs[0].plot(d, T / 1e3, "o-", color=K, ms=3.2, lw=1.1); axs[0].set_xlabel("wall distance $d$ (mm)")
    axs[0].set_ylabel("wall-induced roll torque (µN·m)"); axs[0].set_xlim(0, 42); axs[0].set_ylim(0, None); clean(axs[0])
    axs[1].loglog(d, a, "o-", color=K, ms=3.2, lw=1.1, label="cycle-averaged roll acceleration")
    m = d >= float(far["d_min_mm"]); p = float(far["exponent"]); ref = a[m][0] * (d[m] / d[m][0]) ** p * 1.9
    axs[1].loglog(d[m], ref, color=G, lw=1.0, ls=(0, (4, 2)), label=rf"far-field fit, $\propto d^{{\,{p:.2f}}}$ (offset)")
    axs[1].axhline(nf, color=G2, lw=0.9, ls=(0, (1, 1.5)), label=f"observer noise floor ({nf:.1f} rad/s²)")
    axs[1].set_xlabel("wall distance $d$ (mm)"); axs[1].set_ylabel("roll acceleration (rad/s²)")
    axs[1].set_ylim(0.3, 9e4); axs[1].legend(frameon=False, loc="lower left"); clean(axs[1], grid=None)
    axs[1].set_xticks([2, 5, 10, 20, 40]); axs[1].set_xticklabels(["2", "5", "10", "20", "40"]); axs[1].minorticks_off()
    axs[1].grid(which="both", color="#e4e4e4", lw=0.4)
    for ax, s in zip(axs, "ab"): ax.text(-0.13, 1.02, f"({s})", transform=ax.transAxes, fontsize=10, fontweight="bold", va="bottom")
    save(fig, "fig04_wingwash_signal")


def fig05():
    bl = blocks("outputs/e43_offset_sweep.csv"); rows = bl[0]; slope = float(bl[1][0]["near_center_slope_rad_s2_per_mm"])
    sup = [r for r in csv.DictReader(open("outputs/e43_superposition.csv"))]
    x = np.array([float(r["offset_mm"]) for r in rows]); y = np.array([float(r["roll_acc_rad_s2"]) for r in rows])
    print(f"fig05 two-wall centring: outputs/e43_offset_sweep.csv ({len(x)} offsets), near-centre slope {slope:.1f} rad/s^2/mm; "
          f"superposition points from outputs/e43_superposition.csv")
    for xi, yi in zip(x, y): print(f"  offset {xi:+6.2f} mm  roll acc {yi:+9.1f} rad/s^2")
    for r in sup: print(f"  superposition offset {r['offset_mm']} mm: two-wall {float(r['two_wall_rad_s2']):.1f}, "
                        f"single-wall difference {float(r['single_diff_rad_s2']):.1f}, ratio {r['ratio']}")
    fig = plt.figure(figsize=(3.5, 2.6)); ax = fig.add_axes([0.2, 0.17, 0.77, 0.80])
    ax.plot(x, y / 1e3, "o-", color=K, ms=3.2, lw=1.1, label="two walls (corridor)")
    ax.plot([float(r["offset_mm"]) for r in sup], [float(r["single_diff_rad_s2"]) / 1e3 for r in sup], "s", ms=6.5, mfc="none",
            mec=G, mew=1.0, ls="none", label="sum of two single-wall terms")
    xx = np.array([-6, 6]); ax.plot(xx, slope * xx / 1e3, color=G, lw=0.9, ls=(0, (4, 2)), label=f"slope at centre, {slope:.0f} rad/s² per mm")
    ax.axhline(0, color=K, lw=0.5); ax.axvline(0, color=K, lw=0.5)
    ax.set_xlabel("lateral offset from corridor centre (mm)"); ax.set_ylabel("net roll acceleration (10³ rad/s²)")
    ax.legend(frameon=False, loc="upper left"); clean(ax)
    save(fig, "fig05_two_wall_centering")


def fig06():
    bl = blocks("outputs/e42_observer_fidelity.csv"); rows = bl[0]; rise = float(bl[1][0]["rise_10_90_ms"])
    d = np.array([float(r["d_mm"]) for r in rows]); gn = np.array([float(r["gain"]) for r in rows])
    band = gn[d >= 15]
    print(f"fig06 observer fidelity: outputs/e42_observer_fidelity.csv; rise time 10-90% = {rise:.1f} ms; "
          f"gain band for d >= 15 mm: {band.min():.3f}-{band.max():.3f}")
    for di, gi in zip(d, gn): print(f"  d {di:4.1f} mm  gain {gi:.3f}")
    fig = plt.figure(figsize=(3.5, 2.5)); ax = fig.add_axes([0.16, 0.18, 0.81, 0.79])
    ax.axhspan(band.min(), band.max(), color=LG, zorder=0, label=f"15–30 mm band ({band.min():.2f}–{band.max():.2f})")
    ax.plot(d, gn, "o-", color=K, ms=3.6, lw=1.1, label="estimated / true roll disturbance")
    for di, gi in zip(d, gn): ax.annotate(f"{gi:.2f}", (di, gi), xytext=(0, 5), textcoords="offset points", ha="center", fontsize=7)
    ax.set_xlabel("wall distance $d$ (mm)"); ax.set_ylabel("observer gain"); ax.set_ylim(0, 0.85); ax.set_xlim(10.5, 31.5)
    ax.legend(frameon=False, loc="lower right"); clean(ax)
    save(fig, "fig06_observer_gain")


def fig07():
    bl = blocks("outputs/e49_breach_floor.csv"); rows = bl[0]
    ca, cb = float(bl[1][0]["calibration_a"]), float(bl[1][0]["calibration_b_mm"])
    L = np.array([float(r["true_length_mm"]) for r in rows]); det = np.array([float(r["detect_rate"]) for r in rows])
    raw = np.array([float(r["size_raw_mm"]) for r in rows]); cal = (raw - cb) / ca
    print(f"fig07 detection floor / sizing: outputs/e49_breach_floor.csv; calibration raw = a*true + b with a = {ca:.3f}, "
          f"b = {cb:.2f} mm, so calibrated = (raw - b) / a")
    for Li, di, ri, ci in zip(L, det, raw, cal): print(f"  true {Li:4.0f} mm  detect {di:.0%}  raw {ri:6.2f}  calibrated {ci:6.2f}")
    fig, axs = plt.subplots(1, 2, figsize=(7.1, 2.6)); fig.subplots_adjust(left=0.075, right=0.985, top=0.93, bottom=0.17, wspace=0.24)
    axs[0].plot(L, 100 * det, "o-", color=K, ms=3.4, lw=1.1); axs[0].set_xlabel("true opening length (mm)")
    axs[0].set_ylabel("detection rate (%)"); axs[0].set_ylim(-5, 105); clean(axs[0])
    axs[1].plot([10, 82], [10, 82], color=G2, lw=0.8, ls=(0, (1, 1.5)), label="ideal")
    axs[1].plot(L, raw, "s-", color=G, ms=3.4, lw=1.0, mfc="white", label="raw detected length")
    axs[1].plot(L, cal, "o-", color=K, ms=3.4, lw=1.1, label="calibrated")
    axs[1].set_xlabel("true opening length (mm)"); axs[1].set_ylabel("estimated length (mm)")
    axs[1].legend(frameon=False, loc="upper left"); clean(axs[1])
    for ax, s in zip(axs, "ab"): ax.text(-0.12, 1.02, f"({s})", transform=ax.transAxes, fontsize=10, fontweight="bold", va="bottom")
    save(fig, "fig07_detection_floor")


def fig09():
    import experiments.e47_realistic_course as e47
    import fig_design_a_courses as A
    M = json.load(open(f"{DATA}/mission_truewall.json"))
    legs = [tuple(l) for l in M["legs"]]; br = [tuple(b) for b in M["breaches"]]
    assert legs == [tuple(l) for l in e47.LEGS] and br == [tuple(b) for b in e47.BREACHES]
    geo = e47.build_geometry(legs, br); tj = np.array(M["traj"]) * 1e3
    print(f"fig09 example mission: {DATA}/mission_truewall.json (regenerated: e48.run on e47.LEGS/BREACHES, level "
          f"{M['level']}, seed {M['seed']}, fuse={M['fuse']}, harness commit {M['harness_commit']})")
    print(f"  outcome: reached={M['reached']} crashed={M['crashed']} min clearance {M['min_clear_mm']:.1f} mm, "
          f"flight time {M['t']:.1f} s, {len(tj)} trajectory samples, {len(M['detected'])} openings logged of {len(br)} true")
    for s, a, b in M["detected"]:
        print(f"  detected: {'left' if s == 'L' else 'right'} wall, ({a[0]*1e3:.0f}, {a[1]*1e3:.0f}) -> ({b[0]*1e3:.0f}, {b[1]*1e3:.0f}) mm")
    fig = plt.figure(figsize=(3.5, 2.95)); ax = fig.add_axes([0.01, 0.17, 0.98, 0.82])
    lo, hi = A._bbox(geo); half = (hi - lo) / 2 + np.array([12.0, 42.0])
    A.draw_course(ax, ("ref", legs, br, geo), half); ax.set_title("")
    ax.plot(tj[:, 0], tj[:, 1], color=K, lw=0.9, zorder=8)
    for s, a, b in M["detected"]:
        a, b = np.array(a) * 1e3, np.array(b) * 1e3
        ax.plot([a[0], b[0]], [a[1], b[1]], color=G, lw=3.2, alpha=0.75, solid_capstyle="butt", zorder=7)
    x0, y0 = ax.get_xlim()[0] + 8, ax.get_ylim()[0] + 10
    ax.plot([x0, x0 + 100], [y0, y0], color=K, lw=1.6, solid_capstyle="butt"); ax.text(x0 + 50, y0 + 9, "100 mm", ha="center", fontsize=7)
    L = plt.Line2D
    fig.legend(handles=[L([], [], color=K, lw=0.9, label="flight path"), L([], [], color=K, lw=0, marker=r"$\rightarrow$", ms=9, label="true opening"),
                        L([], [], color=G, lw=3.2, alpha=0.75, label="logged opening"), L([], [], color=K, lw=0, marker="o", mfc="white", ms=7, label="finish")],
               loc="lower center", ncol=2, frameon=False, handlelength=1.5, columnspacing=1.6)
    save(fig, "fig09_example_mission")


def fig10():
    rows = blocks("outputs/e51_baselines_truewall.csv")[0]
    V = ["FULL", "NO_WINGWASH", "NO_FEELERS", "OPEN_LOOP"]; R = {v: [r for r in rows if r["variant"] == v] for v in V}
    nop = sum(int(r["n_openings"]) for r in R["FULL"])
    cnt = {v: (sum(int(r["completed"]) for r in R[v]), sum(int(r["completed"]) and not int(r["crashed"]) for r in R[v]),
               sum(int(r["n_detected"]) for r in R[v])) for v in V}
    T1 = {"FULL": (35, 30, 43), "NO_WINGWASH": (32, 29, 41), "NO_FEELERS": (16, 15, 0), "OPEN_LOOP": (0, 0, 0)}
    assert cnt == T1 and nop == 68 and all(len(R[v]) == 40 for v in V), cnt
    print("fig10 ablation (true-wall): outputs/e51_baselines_truewall.csv; 40 courses, 68 openings; matches Table T1")
    for v in V: print(f"  {v:12s} reached {cnt[v][0]}/40  strike-free {cnt[v][1]}/40  openings found (end-to-end) {cnt[v][2]}/{nop}")
    fig = plt.figure(figsize=(7.1, 2.7)); ax = fig.add_axes([0.07, 0.13, 0.92, 0.75])
    sty = [("reached finish (of 40)", "white", None), ("strike-free (of 40)", G2, None), (f"openings found, end-to-end (of {nop})", "#3a3a3a", None)]
    den = (40, 40, nop); w = 0.26
    for j, (lab, fc, _) in enumerate(sty):
        for i, v in enumerate(V):
            n = cnt[v][j]; h = 100 * n / den[j]
            ax.bar(i + (j - 1) * w, h, w * 0.92, fc=fc, ec=K, lw=0.8, label=lab if i == 0 else None, zorder=3)
            ax.text(i + (j - 1) * w, h + 1.5, f"{n}/{den[j]}", ha="center", va="bottom", fontsize=7)
    ax.set_xticks(range(4)); ax.set_xticklabels(["FULL\n(wing-wash + feelers)", "NO_WINGWASH\n(feelers only)", "NO_FEELERS\n(wing-wash only)", "OPEN_LOOP\n(no sensing)"])
    ax.set_ylabel("percent"); ax.set_ylim(0, 100); ax.tick_params(axis="x", length=0); clean(ax)
    ax.legend(frameon=False, ncol=3, loc="lower center", bbox_to_anchor=(0.5, 1.0), handlelength=1.4, columnspacing=1.6)
    save(fig, "fig10_ablation_truewall")


def _pairs(path, cell=None):
    P = {}
    for r in csv.DictReader(open(path)):
        if cell is None or r["cell"] == cell:
            P.setdefault((r["cell"], int(r["course_id"]), int(r["seed_idx"])), {})[r["variant"]] = r
    return P


def fig11():
    print("fig11 paired differences (FULL - NO_WINGWASH, strike-free completion):")
    rowsout = []
    # T2: primary ensemble, cluster bootstrap over courses with each course's 5 paired seeds
    P = _pairs("outputs/stats_pass1.csv"); courses = sorted({k[1] for k in P})
    PC = [np.array([[int(P[k]["FULL"]["strike_free"]), int(P[k]["NO_WINGWASH"]["strike_free"])] for k in sorted(P) if k[1] == c]) for c in courses]
    A = np.vstack(PC); rng = np.random.default_rng(RNG_SEED); bs = np.empty(B)
    for i in range(B): bs[i] = 100 * np.mean(np.vstack([PC[j] for j in rng.integers(0, len(PC), len(PC))]) @ [1, -1])
    lo, hi = np.percentile(bs, [2.5, 97.5]); b = int(((A[:, 0] == 1) & (A[:, 1] == 0)).sum()); c = int(((A[:, 0] == 0) & (A[:, 1] == 1)).sum())
    f, n_, N = int(A[:, 0].sum()), int(A[:, 1].sum()), len(A); d = 100 * (f - n_) / N
    assert (f, n_, N, b, c) == (152, 150, 200, 12, 10) and (round(d, 1), round(lo, 1), round(hi, 1)) == (1.0, -5.0, 6.5)
    rowsout.append(("Primary ensemble\n40 courses × 5 seeds", d, lo, hi, f"{f}/{N} vs {n_}/{N}", mcnemar_exact(b, c), True, "T2, outputs/stats_pass1.csv"))
    # T3: leg sweep, one flight per course -> bootstrap over the 100 paired courses (not in the committed table)
    P = _pairs("outputs/stats_legsweep.csv"); M = np.array([[int(v["FULL"]["strike_free"]), int(v["NO_WINGWASH"]["strike_free"])] for v in P.values()], float)
    f, n_, N = int(M[:, 0].sum()), int(M[:, 1].sum()), len(M); b = int(((M[:, 0] == 1) & (M[:, 1] == 0)).sum()); c = int(((M[:, 0] == 0) & (M[:, 1] == 1)).sum())
    assert (f, n_, N, b, c) == (48, 42, 100, 14, 8)
    lo, hi = boot(M, lambda X: 100 * np.mean(X[:, 0] - X[:, 1]))
    rowsout.append(("Leg-sweep replication\n100 courses, 2–6 legs", 100 * (f - n_) / N, lo, hi, f"{f}/{N} vs {n_}/{N}", mcnemar_exact(b, c), True,
                    "T3, outputs/stats_legsweep.csv (CI computed here: bootstrap over courses; not in the committed table)"))
    # T4: feeler range levels, paired by course
    T4 = {0.12: (-2.5, -12.5, 7.5), 0.08: (0.0, -12.5, 12.5), 0.05: (17.5, 0.0, 35.0), 0.03: (-3.1, -9.4, 0.0)}
    D = {}
    for line in open("outputs/degraded_feeler.csv.partial.jsonl"):
        r = json.loads(line)
        if r["axis"] == "range": D.setdefault(float(r["level"]), {}).setdefault(int(r["course_id"]), {})[r["variant"]] = int(r["strike_free"])
    for l in (0.12, 0.08, 0.05, 0.03):
        M = np.array([[v["FULL"], v["NO_WINGWASH"]] for v in D[l].values() if len(v) == 2], float); N = len(M)
        f, n_ = int(M[:, 0].sum()), int(M[:, 1].sum()); b = int(((M[:, 0] == 1) & (M[:, 1] == 0)).sum()); c = int(((M[:, 0] == 0) & (M[:, 1] == 1)).sum())
        d = 100 * (f - n_) / N; lo, hi = boot(M, lambda X: 100 * np.mean(X[:, 0] - X[:, 1]))
        assert (round(d, 1), round(lo, 1), round(hi, 1)) == T4[l], (l, d, lo, hi)
        rowsout.append((f"Feeler range {l:.2f} m" + ("\n(partial, 32 courses)" if N != 40 else "\n40 courses"), d, lo, hi, f"{f}/{N} vs {n_}/{N}",
                        mcnemar_exact(b, c), False, "T4, outputs/degraded_feeler.csv.partial.jsonl"))
    for lab, d, lo, hi, cts, p, pre, src in rowsout:
        print(f"  {lab.replace(chr(10), ' | '):48s} {d:+5.1f} pp  CI [{lo:+.1f}, {hi:+.1f}]  {cts}  McNemar p = {p:.3f}  "
              f"{'pre-registered' if pre else 'not pre-registered'}  [{src}]")
    print(f"  all values match docs/truewall_results.md T2/T3/T4 except the T3 interval (new); margin +/-{MARGIN:.0f} pp, "
          f"{B} resamples, default_rng({RNG_SEED})")
    fig = plt.figure(figsize=(7.1, 3.0)); ax = fig.add_axes([0.245, 0.16, 0.50, 0.76])
    ax.axvspan(-MARGIN, MARGIN, color=LG, zorder=0); ax.axvline(0, color=K, lw=0.7, zorder=1)
    for i, (lab, d, lo, hi, cts, p, pre, _) in enumerate(rowsout):
        ax.plot([lo, hi], [i, i], color=K, lw=1.2, zorder=3)
        for e in (lo, hi): ax.plot([e, e], [i - 0.13, i + 0.13], color=K, lw=1.2, zorder=3)
        ax.plot(d, i, "o", ms=6, mfc=K if pre else "white", mec=K, mew=1.2, zorder=4)
        ax.text(1.03, i, f"{cts}\n{d:+.1f} pp [{lo:+.1f}, {hi:+.1f}],  p = {p:.2f}", transform=ax.get_yaxis_transform(), va="center", ha="left",
                fontsize=7, linespacing=1.3)
    ax.set_yticks(range(len(rowsout))); ax.set_yticklabels([r[0] for r in rowsout], fontsize=7.5); ax.set_ylim(len(rowsout) - 0.45, -0.75)
    ax.axhline(1.5, color=G2, lw=0.6, ls=(0, (2, 2)))
    ax.set_xlim(-22, 40); ax.set_xlabel("strike-free completion, FULL − NO_WINGWASH (percentage points, 95% CI)")
    ax.text(0, -0.62, f"±{MARGIN:.0f} pp margin", ha="center", va="center", fontsize=7, color=G)
    ax.text(1.03, -0.62, "strike-free FULL vs NO_WINGWASH", transform=ax.get_yaxis_transform(), fontsize=7, color=G, va="center")
    for s in ("top", "right", "left"): ax.spines[s].set_visible(False)
    ax.tick_params(axis="y", length=0)
    L = plt.Line2D
    ax.legend(handles=[L([], [], marker="o", ms=6, mfc=K, mec=K, ls="none", label="pre-registered"),
                       L([], [], marker="o", ms=6, mfc="white", mec=K, mew=1.2, ls="none", label="not pre-registered, single seed")],
              frameon=False, loc="lower right", handletextpad=0.3)
    save(fig, "fig11_paired_differences")


def fig12():
    P = _pairs("outputs/stats_legsweep.csv"); legs = [2, 3, 4, 5, 6]
    new = {l: (sum(int(v["FULL"]["strike_free"]) for k, v in P.items() if k[0] == str(l)),
               sum(int(v["NO_WINGWASH"]["strike_free"]) for k, v in P.items() if k[0] == str(l))) for l in legs}
    old = {l: [0, 0] for l in legs}
    for r in blocks("outputs/e52_wingwash_scaling.csv")[0]:
        if r["factor"] == "legs":
            old[int(r["cell"])][0 if r["variant"] == "FULL" else 1] += int(r["completed"]) and not int(r["crashed"])
    T3new = {2: (13, 12), 3: (10, 9), 4: (6, 7), 5: (12, 10), 6: (7, 4)}; T3old = {2: (11, 8), 3: (11, 8), 4: (8, 9), 5: (13, 5), 6: (5, 4)}
    assert new == T3new and {l: tuple(v) for l, v in old.items()} == T3old, (new, old)
    print("fig12 leg-sweep replication: pre-fix outputs/e52_wingwash_scaling.csv (factor == legs) vs true-wall "
          "outputs/stats_legsweep.csv; strike-free of 20 per cell; matches Table T3")
    for l in legs: print(f"  legs {l}: pre-fix FULL {old[l][0]} / NO_WINGWASH {old[l][1]};  replication FULL {new[l][0]} / NO_WINGWASH {new[l][1]}")
    print(f"  pooled: pre-fix {sum(v[0] for v in old.values())} vs {sum(v[1] for v in old.values())}; "
          f"replication {sum(v[0] for v in new.values())} vs {sum(v[1] for v in new.values())} (of 100)")
    fig, axs = plt.subplots(1, 2, figsize=(7.1, 2.6), sharey=True); fig.subplots_adjust(left=0.07, right=0.99, top=0.84, bottom=0.17, wspace=0.06)
    for ax, D, ttl in ((axs[0], old, "original run (pre-fix walls)"), (axs[1], new, "replication (true walls, fresh noise seeds)")):
        for j, (lab, fc) in enumerate(((FULL_LBL, "#3a3a3a"), (NOWW_LBL, "white"))):
            xs = np.arange(5) + (j - 0.5) * 0.36; hs = [D[l][j] for l in legs]
            ax.bar(xs, hs, 0.33, fc=fc, ec=K, lw=0.8, label=lab, zorder=3)
            for x_, h in zip(xs, hs): ax.text(x_, h + 0.3, str(h), ha="center", va="bottom", fontsize=7)
        tot = (sum(D[l][0] for l in legs), sum(D[l][1] for l in legs))
        ax.set_title(f"{ttl}\npooled {tot[0]}/100 vs {tot[1]}/100", fontsize=8, pad=3)
        ax.set_xticks(range(5)); ax.set_xticklabels(legs); ax.set_xlabel("number of legs"); ax.set_ylim(0, 20); ax.set_yticks([0, 5, 10, 15, 20])
        ax.tick_params(axis="x", length=0); clean(ax)
    axs[0].set_ylabel("strike-free courses (of 20)"); axs[1].legend(frameon=False, loc="upper right")
    for ax, s in zip(axs, "ab"): ax.text(0.0, 1.03, f"({s})", transform=ax.transAxes, fontsize=10, fontweight="bold", va="bottom", ha="left")
    save(fig, "fig12_legsweep_replication")


def fig13():
    P = _pairs("outputs/stats_pass1.csv")
    F = np.array([float(v["FULL"]["min_clear_mm"]) for v in P.values()]); N = np.array([float(v["NO_WINGWASH"]["min_clear_mm"]) for v in P.values()])
    assert len(F) == 200 and (round(F.mean(), 2), round(N.mean(), 2)) == (17.98, 17.74)
    print(f"fig13 clearance (true-wall): outputs/stats_pass1.csv, min_clear_mm, 200 flights per variant; strike radius "
          f"{WINGREACH_MM} mm (src/safety.py WINGREACH)")
    for lab, X in (("FULL", F), ("NO_WINGWASH", N)):
        q = np.percentile(X, [25, 50, 75])
        print(f"  {lab:12s} mean {X.mean():.2f}  median {q[1]:.2f}  IQR [{q[0]:.2f}, {q[2]:.2f}]  min {X.min():.2f}  max {X.max():.2f}  "
              f"below strike radius {int((X < WINGREACH_MM).sum())}/200")
    fig = plt.figure(figsize=(3.5, 2.8)); ax = fig.add_axes([0.17, 0.13, 0.80, 0.84])
    rng = np.random.default_rng(0)                                   # jitter for display only
    for i, X in enumerate((F, N)):
        ax.plot(i + rng.uniform(-0.17, 0.17, len(X)), X, "o", ms=2.2, mfc="none", mec=G2, mew=0.5, zorder=2)
        ax.boxplot(X, positions=[i], widths=0.42, showfliers=False, zorder=3, medianprops=dict(color=K, lw=1.4),
                   boxprops=dict(color=K, lw=0.9), whiskerprops=dict(color=K, lw=0.9), capprops=dict(color=K, lw=0.9))
        ax.plot(i, X.mean(), "D", ms=4, mfc="white", mec=K, mew=1.0, zorder=4)
    ax.axhline(WINGREACH_MM, color=K, lw=0.9, ls=(0, (4, 2))); ax.text(1.48, WINGREACH_MM - 0.5, f"wingtip-strike radius {WINGREACH_MM} mm", ha="right", va="top", fontsize=7)
    ax.set_xticks([0, 1]); ax.set_xticklabels(["FULL", "NO_WINGWASH"]); ax.set_xlim(-0.5, 1.5); ax.set_ylim(0, None)
    ax.set_ylabel("minimum wall clearance per flight (mm)"); ax.tick_params(axis="x", length=0); clean(ax)
    save(fig, "fig13_clearance_truewall")


def _seed_sweep(path, key, stem, xlabel, order, note):
    rows = blocks(path)[0]; lv = sorted({float(r[key]) for r in rows}, key=order)
    print(f"{stem}: {path} (per-seed block). {note}")
    fig = plt.figure(figsize=(3.5, 2.6)); ax = fig.add_axes([0.17, 0.18, 0.80, 0.72])
    for i, l in enumerate(lv):
        R = [r for r in rows if float(r[key]) == l]; ok = [r for r in R if int(r["reached"])]
        c = [float(r["min_clear_mm"]) for r in R if r["min_clear_mm"] not in ("", "nan")]
        print(f"  {key} {l:g}: reached {len(ok)}/{len(R)}, crashed {sum(int(float(r['crashed'])) for r in R)}/{len(R)}, "
              f"min clearance per seed {[round(x, 1) for x in c]}")
        for r in R:
            if r["min_clear_mm"] in ("", "nan"): continue
            ax.plot(i, float(r["min_clear_mm"]), "o", ms=4.2, mfc=K if int(r["reached"]) else "white", mec=K, mew=1.0, zorder=3)
        ax.text(i, 1.02, f"{len(ok)}/{len(R)}", transform=ax.get_xaxis_transform(), ha="center", va="bottom", fontsize=7)
    ax.text(-0.02, 1.02, "reached:", transform=ax.transAxes, ha="right", va="bottom", fontsize=7)
    ax.axhline(WINGREACH_MM, color=K, lw=0.9, ls=(0, (4, 2))); ax.text(len(lv) - 0.55, WINGREACH_MM - 0.5, "strike radius", ha="right", va="top", fontsize=7)
    ax.set_xticks(range(len(lv))); ax.set_xticklabels([f"{l:g}" for l in lv]); ax.set_xlim(-0.5, len(lv) - 0.5); ax.set_ylim(0, 26)
    ax.set_xlabel(xlabel); ax.set_ylabel("minimum wall clearance (mm)"); clean(ax)
    L = plt.Line2D
    ax.legend(handles=[L([], [], marker="o", ms=4.2, mfc=K, mec=K, ls="none", label="reached finish"),
                       L([], [], marker="o", ms=4.2, mfc="white", mec=K, ls="none", label="did not reach")], frameon=False, loc="lower left", ncol=2,
              handletextpad=0.2, columnspacing=1.0)
    save(fig, stem)


def figS1():
    _seed_sweep("outputs/e37_noise_sweep.csv", "noise_level", "figS1_noise_sweep", "sensor-noise level (× baseline)", lambda v: v,
                "PRE-FIX harness (flown before the true-wall fix).")


def figS2():
    _seed_sweep("outputs/e38_rate_sweep.csv", "rate_hz", "figS2_control_rate", "control rate (Hz)", lambda v: -v,
                "PRE-FIX harness (flown before the true-wall fix).")


def figS3():
    rows = list(csv.DictReader(open("outputs/e40_side_gap.csv")))
    print("figS3 side gap: outputs/e40_side_gap.csv. PRE-FIX harness (flown before the true-wall fix).")
    fig = plt.figure(figsize=(3.5, 2.6)); ax = fig.add_axes([0.17, 0.18, 0.80, 0.79])
    for pol, lab, ls, mk in (("base", "wing-wash only (no veto)", (0, (4, 2)), "s"), ("fused", "feeler veto fusion", "-", "o")):
        R = [r for r in rows if r["policy"] == pol]; x = [float(r["gap_mm"]) for r in R]; y = [float(r["min_clear_mm"]) for r in R]
        ax.plot(x, y, color=K if pol == "fused" else G, ls=ls, lw=1.1, zorder=2)
        for r, xi, yi in zip(R, x, y):
            ax.plot(xi, yi, mk, ms=5, mfc="white" if int(r["crashed"]) else (K if pol == "fused" else G), mec=K if pol == "fused" else G, mew=1.1, zorder=3)
            print(f"  {pol:5s} gap {xi:5.0f} mm  outcome {r['outcome'] or '-':7s} min clearance {yi:5.1f} mm  crashed {r['crashed']}")
        ax.plot([], [], color=K if pol == "fused" else G, ls=ls, lw=1.1, marker=mk, ms=5, label=lab)
    ax.plot([], [], "o", ms=5, mfc="white", mec=K, ls="none", label="open marker: wall strike")
    ax.axhline(WINGREACH_MM, color=K, lw=0.9, ls=(0, (1, 1.5))); ax.text(196, WINGREACH_MM - 0.5, "strike radius", ha="right", va="top", fontsize=7)
    ax.set_xlabel("side-gap length (mm)"); ax.set_ylabel("minimum wall clearance (mm)"); ax.set_ylim(0, 41)
    ax.legend(frameon=False, loc="upper right"); clean(ax)
    save(fig, "figS3_side_gap")


ALL = dict(fig01=fig01, fig02=fig02, fig03=fig03, fig04=fig04, fig05=fig05, fig06=fig06, fig07=fig07, fig08=fig08, fig09=fig09,
           fig10=fig10, fig11=fig11, fig12=fig12, fig13=fig13, figS1=figS1, figS2=figS2, figS3=figS3)

if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True); apply_style()
    for name in (sys.argv[1:] or list(ALL)):
        ALL[name]()
