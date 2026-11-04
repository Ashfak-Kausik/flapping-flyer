"""
e55 — TRUE-WALL RE-RUNS of the two paper results that predate the true-wall fix (9fbef46):

  floor : e49 breach-length detection floor (one opening, 64mm corridor), nominal noise.
  kge   : e44 K_GE x control-rate grid on the e36/e38 crafted course. e44 flew it at noise
          level 0.0, where the seeds are identical flights; here it is flown at nominal noise
          (level 1.0) so the seeds are real noise realizations.

No flight logic is changed: each task calls e49.run_one / e38.run_one as they stand after the
fix. The only additions are a per-process model path (so parallel workers do not overwrite each
other's MJCF) and a JSONL checkpoint, one line per finished flight, so an interrupted run resumes.

Run:  python3 -u experiments/e55_truewall_reruns.py [floor] [kge]      (default: both)
Env:  E55_WORKERS (default 14), E55_SEEDS (default 5 -> seeds 1..5)
"""
import sys; sys.path.insert(0, '.')
import os, json, csv, time
os.environ.setdefault("OMP_NUM_THREADS", "1"); os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
import numpy as np
from multiprocessing import Pool

LEVEL = 1.0
SEEDS = list(range(1, int(os.environ.get("E55_SEEDS", "5")) + 1))
LENGTHS_MM = (10, 12, 15, 18, 20, 25, 30, 35, 40, 50, 60, 80)        # as e49.sweep
KGES = (0.5, 1.0, 1.5); RATES = (1000, 500, 250)                     # as e44.rate_robustness
CKPT = "outputs/e55_truewall_reruns.jsonl"
FLOOR_CSV = "outputs/e49_breach_floor_truewall.csv"
KGE_CSV = "outputs/e44_kge_sensitivity_truewall.csv"


def _num(v):
    v = float(v); return None if np.isnan(v) else v


def work(task):
    kind = task[0]; pid = os.getpid(); t0 = time.time()
    if kind == "floor":
        _, L, sd = task
        import experiments.e45_breach_inspection as e45
        import experiments.e49_breach_floor as e49
        if not hasattr(e45, "_e55_orig"):
            e45._e55_orig = e45.build_model
            e45.build_model = lambda: e45._e55_orig(path=f"models/_e55_floor_{pid}.xml")
        r = e49.run_one(L / 1e3, level=LEVEL, seed=sd)
        out = dict(kind=kind, length_mm=L, seed=sd, detected=bool(r["detected"]), loc_err_mm=_num(r["loc_err_mm"]),
                   size_raw_mm=_num(r["size_raw_mm"]), reached=r["reached"])
    else:
        _, k, rate, sd = task
        import src.ground_effect as ge
        import experiments.e36_reactive_course as e36
        import experiments.e38_control_rate as e38
        if not hasattr(e36, "_e55_orig"):
            e36._e55_orig = e36.build_model
            e36.build_model = lambda: e36._e55_orig(path=f"models/_e55_kge_{pid}.xml")
        ge.kappa_pts.__defaults__ = (float(k), ge.KAPPA_MAX)          # as e44.set_kge
        r = e38.run_one(rate_hz=rate, level=LEVEL, seed=sd)
        ge.kappa_pts.__defaults__ = (1.0, ge.KAPPA_MAX)
        out = dict(kind=kind, K_GE=k, rate_hz=rate, seed=sd, reached=bool(r["reached"]), crashed=bool(r["crashed"]),
                   min_clear_mm=_num(r["min_clear_mm"]))
    out["level"] = LEVEL; out["wall_s"] = round(time.time() - t0, 1)
    return out


def key(r):
    return ("floor", r["length_mm"], r["seed"]) if r["kind"] == "floor" else ("kge", r["K_GE"], r["rate_hz"], r["seed"])


def load():
    if not os.path.exists(CKPT): return []
    return [json.loads(l) for l in open(CKPT) if l.strip()]


def write_floor(recs):
    R = [r for r in recs if r["kind"] == "floor"]
    if not R: return
    rows = []
    for L in LENGTHS_MM:
        c = [r for r in R if r["length_mm"] == L]
        if not c: continue
        d = [r for r in c if r["detected"]]
        rows.append((L, len(d), len(c), np.mean([r["loc_err_mm"] for r in d]) if d else np.nan,
                     np.mean([r["size_raw_mm"] for r in d]) if d else np.nan))
    pts = [(r[0], r[4]) for r in rows if r[1] == r[2]]               # fit over fully detected lengths, as e49.calibrate
    a, b = np.polyfit([p[0] for p in pts], [p[1] for p in pts], 1) if len(pts) >= 2 else (np.nan, np.nan)
    with open(FLOOR_CSV, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["true_length_mm", "n_detected", "n_seeds", "detect_rate", "loc_err_mm", "size_raw_mm", "size_cal_mm", "size_err_cal_mm"])
        for L, nd, n, loc, sz in rows:
            cal = (sz - b) / a
            w.writerow([L, nd, n, nd / n, "" if np.isnan(loc) else loc, "" if np.isnan(sz) else sz,
                        "" if np.isnan(cal) else cal, "" if np.isnan(cal) else cal - L])
        w.writerow([]); w.writerow(["calibration_a", "calibration_b_mm", "level", "seeds"]); w.writerow([a, b, LEVEL, " ".join(map(str, SEEDS))])
        w.writerow([]); w.writerow(["true_length_mm", "seed", "detected", "loc_err_mm", "size_raw_mm", "reached"])
        for r in sorted(R, key=lambda r: (r["length_mm"], r["seed"])):
            w.writerow([r["length_mm"], r["seed"], int(r["detected"]), r["loc_err_mm"], r["size_raw_mm"], r["reached"]])
    print("saved ->", FLOOR_CSV)


def write_kge(recs):
    R = [r for r in recs if r["kind"] == "kge"]
    if not R: return
    with open(KGE_CSV, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["K_GE", "rate_hz", "reached", "strike_free", "n_seeds", "crashed", "mean_clear_mm_reached", "level"])
        for k in KGES:
            for rate in RATES:
                c = [r for r in R if r["K_GE"] == k and r["rate_hz"] == rate]
                if not c: continue
                cl = [r["min_clear_mm"] for r in c if r["reached"]]
                w.writerow([k, rate, sum(r["reached"] for r in c), sum(r["reached"] and not r["crashed"] for r in c), len(c),
                            sum(r["crashed"] for r in c), np.mean(cl) if cl else "", LEVEL])
        w.writerow([]); w.writerow(["K_GE", "rate_hz", "seed", "reached", "crashed", "min_clear_mm"])
        for r in sorted(R, key=lambda r: (r["K_GE"], -r["rate_hz"], r["seed"])):
            w.writerow([r["K_GE"], r["rate_hz"], r["seed"], int(r["reached"]), int(r["crashed"]), r["min_clear_mm"]])
    print("saved ->", KGE_CSV)


if __name__ == "__main__":
    parts = sys.argv[1:] or ["floor", "kge"]
    tasks = []
    if "floor" in parts: tasks += [("floor", L, sd) for sd in SEEDS for L in LENGTHS_MM]
    if "kge" in parts: tasks += [("kge", k, rate, sd) for sd in SEEDS for k in KGES for rate in RATES]
    recs = load(); done = {key(r) for r in recs}
    todo = [t for t in tasks if t not in done]
    print(f"e55: {len(tasks)} flights requested, {len(tasks) - len(todo)} already in {CKPT}, {len(todo)} to fly; "
          f"level {LEVEL}, seeds {SEEDS}", flush=True)
    if todo:
        t0 = time.time()
        with Pool(int(os.environ.get("E55_WORKERS", "14")), maxtasksperchild=1) as pool, open(CKPT, "a") as fh:
            for i, r in enumerate(pool.imap_unordered(work, todo), 1):
                fh.write(json.dumps(r) + "\n"); fh.flush(); recs.append(r)
                print(f"[{i}/{len(todo)}] {time.time() - t0:7.0f}s  {r}", flush=True)
    write_floor(recs); write_kge(recs)
    for f in os.listdir("models"):
        if f.startswith("_e55_"): os.remove(os.path.join("models", f))
