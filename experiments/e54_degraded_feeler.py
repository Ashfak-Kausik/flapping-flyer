"""
e54_degraded_feeler — does wing-wash earn its value when the FEELER is degraded toward realism?

FULL (wing-wash + degraded feeler) vs NO_WINGWASH (degraded feeler only) on the corrected
(true-wall) harness, primary 40-course ensemble (e51.generate_courses, seed 1234), one axis of
feeler degradation at a time:

  range    feeler_range       readings beyond X m return clear          (idealized: 0.16 m antenna reach)
  dropout  feeler_dropout     each ray drops w.p. p / tick and HOLDS its last reading (idealized: 0)
  noise    feeler_noise_mult  multiplier on the 2 mm feeler range sigma (idealized: 1)

The degradation acts ONLY in NoiseModel.feel(), i.e. on the readings navigation uses (steering,
safety veer, speed law, turn trigger, veto, breach logger). The true ray-cast that feeds the
wing-wash wall planes (e48: raw -> e.planes(psi_true, ..., raw[5], raw[6])) is untouched.

Pairing reuses e53 pass1 seed index j=1: noise_seed = 1000 + course_id, identical for FULL and
NO_WINGWASH and identical across every cell, so the undegraded reference for each cell is the
j=1 slice of outputs/stats_pass1.csv (not re-flown). Noise level 1.0, 1 kHz, no gains retuned,
generator and scoring unchanged.

Every finished flight is checkpointed to <OUT>.partial.jsonl; rerunning resumes. Refuses to
overwrite an existing output CSV.

Run:  python experiments/e54_degraded_feeler.py
      E54_CELLS=baseline:0,range:0.05 E54_COURSES=0 E54_OUT=/tmp/x.csv python experiments/e54_degraded_feeler.py   (smoke)
"""
import sys; sys.path.insert(0,'.'); import numpy as np, csv, os, time, glob, json
import concurrent.futures as cf
import experiments.e51_baselines as e51

VARIANTS = ("FULL", "NO_WINGWASH")
SEED0 = 1000                                   # = e53 pass1, seed index j=1
AXES = {"range":   ("feeler_range",      [0.12, 0.08, 0.05, 0.03]),
        "dropout": ("feeler_dropout",    [0.1, 0.3, 0.5, 0.8]),
        "noise":   ("feeler_noise_mult", [2.0, 4.0, 8.0, 16.0])}

def cells():
    sel = os.environ.get("E54_CELLS")
    if sel: return [(a, float(l)) for a, l in (c.split(":") for c in sel.split(","))]
    return [(a, l) for a, (_, levels) in AXES.items() for l in levels]

def build_jobs():
    only = os.environ.get("E54_COURSES"); only = {int(x) for x in only.split(",")} if only else None
    courses = [c for c in e51.generate_courses(N=40, hard=False) if only is None or c[0] in only]
    jobs = []
    for axis, level in cells():
        feeler = None if axis == "baseline" else {AXES[axis][0]: level}
        for (k, legs, br, geo) in courses:
            for v in VARIANTS:
                kwargs = dict(e51.VARIANTS[v], feeler=feeler); tag = f"df{axis}{level:g}{v}"
                meta = dict(axis=axis, level=level, variant=v, course_id=k, noise_seed=SEED0+k)
                jobs.append((f"{axis}|{level:g}|{k}|{v}", meta, (k, legs, br, geo, kwargs, 1.0, SEED0, tag, 1000)))
    return jobs

def _job(job):
    key, meta, args = job; t0 = time.time(); res = e51._run_one_course(args); row = res['row']
    for p in glob.glob(f"models/_e51_{args[7]}_{args[0]}_{os.getpid()}.xml"): os.remove(p)
    done = bool(row['completed']); hit = bool(row['crashed'])
    return dict(key=key, **meta, n_legs=row['n_legs'], n_openings=row['n_openings'], completed=int(done),
                crashed=int(hit), strike_free=int(done and not hit), min_clear_mm=float(row['min_clear_mm']),
                n_detected=int(row['n_detected']), n_false_positives=int(row['n_false_positives']),
                loc_errors_mm=";".join(f"{float(x):.3f}" for x in row['loc_errors_mm']), wall_s=round(time.time()-t0, 1))

COLS = ["axis","level","variant","course_id","noise_seed","n_legs","n_openings","completed","crashed",
        "strike_free","min_clear_mm","n_detected","n_false_positives","loc_errors_mm","wall_s"]

if __name__ == "__main__":
    OUT = os.environ.get("E54_OUT", "outputs/degraded_feeler.csv"); PART = OUT + ".partial.jsonl"
    if os.path.exists(OUT): sys.exit(f"refusing to overwrite existing {OUT}")
    W = int(os.environ.get("E54_WORKERS", str(os.cpu_count() or 4)))
    jobs = build_jobs(); done = {}
    if os.path.exists(PART):
        for line in open(PART):
            r = json.loads(line); done[r['key']] = r
        print(f"resumed {len(done)} finished flights from {PART}", flush=True)
    todo = [j for j in jobs if j[0] not in done]
    print(f"{len(jobs)} flights, {len(todo)} to run, {W} workers", flush=True); t0 = time.time()
    with cf.ProcessPoolExecutor(max_workers=W) as ex, open(PART, "a") as pf:
        for fut in cf.as_completed([ex.submit(_job, j) for j in todo]):
            r = fut.result(); done[r['key']] = r; pf.write(json.dumps(r) + "\n"); pf.flush()
            print(f"[{time.time()-t0:6.0f}s {len(done)}/{len(jobs)}] {r['key']:30s} reached={r['completed']} crashed={r['crashed']} "
                  f"clear={r['min_clear_mm']:.1f}mm det={r['n_detected']}/{r['n_openings']} ({r['wall_s']:.0f}s)", flush=True)
    with open(OUT, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(COLS)
        for key, _, _ in jobs: w.writerow([repr(done[key][c]) if c == "min_clear_mm" else done[key][c] for c in COLS])
    print(f"saved -> {OUT}   wall {time.time()-t0:.0f}s", flush=True)
