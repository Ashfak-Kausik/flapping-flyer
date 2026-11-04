"""
e57_width_sweep — corridor-width sweep on the corrected (true-wall) harness, FULL vs NO_WINGWASH.
Plan and decision rule were written first: docs/e57_analysis_plan.md.

  Five width bands (e52.WIDTH_CELLS), 3 legs, 40 courses per band from e52.generate_courses_param with
  generator seed 5200 + band index (the first 20 per band are the e52 courses), one flight per variant
  per course, matched noise_seed = 8000 + 100*band_index + course_id.   -> outputs/e57_width_sweep.csv

Every flight goes through e51._run_one_course -> e48.run + e50.score_quiet, noise level 1.0, 1 kHz.
Every finished flight is checkpointed to outputs/e57_runs.jsonl; rerunning resumes.

Run:  python3 -u experiments/e57_width_sweep.py
Env:  E57_WORKERS (default 15), E57_DRY=1 (list the jobs, fly nothing)
"""
import sys; sys.path.insert(0,'.'); import csv, os, time, glob, json, hashlib
import concurrent.futures as cf
import experiments.e51_baselines as e51
import experiments.e52_wingwash_scaling as e52

VARIANTS = ("FULL", "NO_WINGWASH"); N_PER_BAND = 40; SEED0 = 8000; SEED_STRIDE = 100
CKPT = "outputs/e57_runs.jsonl"; OUT = "outputs/e57_width_sweep.csv"

def build_jobs():
    """-> list of (key, meta, e51-args), band by band."""
    jobs = []
    for idx, (label, wlo, whi) in enumerate(e52.WIDTH_CELLS):
        courses, skipped = e52.generate_courses_param(N_PER_BAND, wlo, whi, e52.WIDTH_FIXED_LEGS, e52.WIDTH_SEED_BASE+idx)
        if skipped: print(f"  band {label}: {skipped} courses could not be generated", flush=True)
        seed0 = SEED0 + SEED_STRIDE*idx
        for (k, legs, br, geo) in courses:
            for v in VARIANTS:
                meta = dict(set="width", cell=label, variant=v, course_id=k, seed_idx=0, noise_seed=seed0+k)
                jobs.append((f"{label}|{k}|0|{v}", meta, (k, legs, br, geo, e51.VARIANTS[v], 1.0, seed0, f"e57w{idx}{v}", 1000)))
    return jobs

def _job(job):
    key, meta, args = job; t0 = time.time(); res = e51._run_one_course(args); row = res['row']
    for p in glob.glob(f"models/_e51_{args[7]}_{args[0]}_{os.getpid()}.xml"): os.remove(p)
    done = bool(row['completed']); hit = bool(row['crashed'])
    return dict(key=key, **meta, n_legs=row['n_legs'], n_openings=row['n_openings'], completed=int(done),
                crashed=int(hit), strike_free=int(done and not hit), min_clear_mm=float(row['min_clear_mm']),
                n_detected=int(row['n_detected']), n_false_positives=int(row['n_false_positives']),
                loc_errors_mm=";".join(f"{float(x):.3f}" for x in row['loc_errors_mm']), wall_s=round(time.time()-t0, 1))

COLS = ["set","cell","variant","course_id","seed_idx","noise_seed","n_legs","n_openings","completed","crashed",
        "strike_free","min_clear_mm","n_detected","n_false_positives","loc_errors_mm","wall_s"]

if __name__ == "__main__":
    plan = "docs/e57_analysis_plan.md"
    print(f"plan {plan} sha256 {hashlib.sha256(open(plan,'rb').read()).hexdigest()}", flush=True)
    jobs = build_jobs(); done = {}
    if os.path.exists(CKPT):
        for line in open(CKPT):
            r = json.loads(line); done[r['key']] = r
    todo = [j for j in jobs if j[0] not in done]
    for label, _, _ in e52.WIDTH_CELLS: print(f"  {label} mm: {sum(j[1]['cell']==label for j in jobs)} flights", flush=True)
    print(f"{len(jobs)} flights, {len(jobs)-len(todo)} already in {CKPT}, {len(todo)} to run", flush=True)
    if os.environ.get("E57_DRY"): sys.exit(0)
    W = int(os.environ.get("E57_WORKERS", "15")); t0 = time.time()
    if todo:
        with cf.ProcessPoolExecutor(max_workers=W) as ex, open(CKPT, "a") as pf:
            for fut in cf.as_completed([ex.submit(_job, j) for j in todo]):
                r = fut.result(); done[r['key']] = r; pf.write(json.dumps(r) + "\n"); pf.flush()
                print(f"[{time.time()-t0:6.0f}s {len(done)}/{len(jobs)}] {r['key']:26s} reached={r['completed']} crashed={r['crashed']} "
                      f"clear={r['min_clear_mm']:.1f}mm det={r['n_detected']}/{r['n_openings']} ({r['wall_s']:.0f}s)", flush=True)
    with open(OUT, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(COLS)
        for key, _, _ in jobs: w.writerow([repr(done[key][c]) if c == "min_clear_mm" else done[key][c] for c in COLS])
    print(f"saved -> {OUT}\ndone, wall {time.time()-t0:.0f}s", flush=True)
