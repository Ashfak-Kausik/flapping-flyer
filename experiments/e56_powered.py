"""
e56_powered — two follow-ups on the corrected (true-wall) harness. Plan and decision rule were written
first: docs/e56_analysis_plan.md.

  confirm  feeler range limited to 0.05 m, FULL vs NO_WINGWASH on 200 NEW primary-generator courses
           (generator seed 5600; none of the 40 e54 courses), matched noise_seed = 9000 + course_id.
           -> outputs/e56_confirm_range005.csv
  gain     centring gain Kc x {2, 4, 8}, FULL, the 40 primary courses, noise seeds 1000*j + course_id
           for j = 1..3 (= e53 pass1, so the 1x and 0x points are outputs/stats_pass1.csv, not re-flown).
           -> outputs/e56_gain_sweep.csv
  check    one repeat of a stored pass1 flight (course 16, j=1, FULL, 1x) through this runner.

Every flight goes through e51._run_one_course -> e48.run + e50.score_quiet, noise level 1.0, 1 kHz.
The only override is e48.Kc (module global read by e48.run), set per job in the worker.
Every finished flight is checkpointed to outputs/e56_runs.jsonl; rerunning resumes.

Run:  python3 -u experiments/e56_powered.py [check] [confirm] [gain]      (default: all three, in that order)
Env:  E56_WORKERS (default 15), E56_DRY=1 (list the jobs, fly nothing)
"""
import sys; sys.path.insert(0,'.'); import numpy as np, csv, os, time, glob, json, hashlib
import concurrent.futures as cf
import experiments.e36_reactive_course as e36
import experiments.e47_realistic_course as e47
import experiments.e48_mission as e48
import experiments.e50_batch as e50
import experiments.e51_baselines as e51

KC0 = e36.Kc                                   # deployed centring gain, 1.5e-5
CONFIRM_N = 200; CONFIRM_GEN_SEED = 5600; CONFIRM_SEED0 = 9000; CONFIRM_RANGE = 0.05
GAIN_MULTS = (2.0, 4.0, 8.0); GAIN_SEEDS = (1, 2, 3); SEED_STRIDE = 1000
CKPT = "outputs/e56_runs.jsonl"
OUTS = {"confirm": "outputs/e56_confirm_range005.csv", "gain": "outputs/e56_gain_sweep.csv", "check": "outputs/e56_check.csv"}

def generate_courses_seeded(N, seed):
    """e51.generate_courses loop (primary envelope, same rejection rule) with a caller-chosen generator seed."""
    rng = np.random.default_rng(seed); courses = []
    for k in range(N):
        for _ in range(40):
            legs, breaches = e50.sample_course(rng, False)
            geo = e47.build_geometry(legs, breaches)
            if e50.valid(geo) and len(breaches) > 0: break
        else:
            continue
        courses.append((k, legs, breaches, geo))
    return courses

def build_jobs(parts):
    """-> list of (key, meta, kc_mult, e51-args)."""
    jobs = []
    primary = e51.generate_courses(N=40, hard=False) if ("gain" in parts or "check" in parts) else []
    if "check" in parts:
        (k, legs, br, geo) = next(c for c in primary if c[0] == 16)
        meta = dict(set="check", cell="kc1", variant="FULL", course_id=k, seed_idx=1, noise_seed=SEED_STRIDE+k, kc_mult=1.0, feeler_range=None)
        jobs.append((f"check|{k}|1|FULL", meta, 1.0, (k, legs, br, geo, e51.VARIANTS["FULL"], 1.0, SEED_STRIDE, "e56chk", 1000)))
    if "confirm" in parts:
        for (k, legs, br, geo) in generate_courses_seeded(CONFIRM_N, CONFIRM_GEN_SEED):
            for v in ("FULL", "NO_WINGWASH"):
                kwargs = dict(e51.VARIANTS[v], feeler={"feeler_range": CONFIRM_RANGE})
                meta = dict(set="confirm", cell="range0.05", variant=v, course_id=k, seed_idx=0, noise_seed=CONFIRM_SEED0+k,
                            kc_mult=1.0, feeler_range=CONFIRM_RANGE)
                jobs.append((f"confirm|{k}|0|{v}", meta, 1.0, (k, legs, br, geo, kwargs, 1.0, CONFIRM_SEED0, f"e56c{v}", 1000)))
    if "gain" in parts:
        for j in GAIN_SEEDS:                       # seed index outermost: a partial run is still balanced across gains
            for (k, legs, br, geo) in primary:
                for m in GAIN_MULTS:
                    meta = dict(set="gain", cell=f"kc{m:g}", variant="FULL", course_id=k, seed_idx=j, noise_seed=SEED_STRIDE*j+k,
                                kc_mult=m, feeler_range=None)
                    jobs.append((f"gain|{k}|{j}|kc{m:g}", meta, m, (k, legs, br, geo, e51.VARIANTS["FULL"], 1.0, SEED_STRIDE*j, f"e56g{m:g}s{j}", 1000)))
    return jobs

def _job(job):
    key, meta, mult, args = job; t0 = time.time()
    e48.Kc = KC0*mult
    try:
        res = e51._run_one_course(args)
    finally:
        e48.Kc = KC0
    row = res['row']
    for p in glob.glob(f"models/_e51_{args[7]}_{args[0]}_{os.getpid()}.xml"): os.remove(p)
    done = bool(row['completed']); hit = bool(row['crashed'])
    return dict(key=key, **meta, n_legs=row['n_legs'], n_openings=row['n_openings'], completed=int(done),
                crashed=int(hit), strike_free=int(done and not hit), min_clear_mm=float(row['min_clear_mm']),
                n_detected=int(row['n_detected']), n_false_positives=int(row['n_false_positives']),
                loc_errors_mm=";".join(f"{float(x):.3f}" for x in row['loc_errors_mm']), wall_s=round(time.time()-t0, 1))

COLS = ["set","cell","variant","course_id","seed_idx","noise_seed","kc_mult","feeler_range","n_legs","n_openings","completed","crashed",
        "strike_free","min_clear_mm","n_detected","n_false_positives","loc_errors_mm","wall_s"]

def write_csvs(jobs, done):
    for part, out in OUTS.items():
        keys = [j[0] for j in jobs if j[1]["set"] == part]
        if not keys or any(k not in done for k in keys): continue
        with open(out, "w", newline="") as fh:
            w = csv.writer(fh); w.writerow(COLS)
            for k in keys: w.writerow([repr(done[k][c]) if c == "min_clear_mm" else done[k][c] for c in COLS])
        print(f"saved -> {out}", flush=True)

if __name__ == "__main__":
    parts = sys.argv[1:] or ["check", "confirm", "gain"]
    plan = "docs/e56_analysis_plan.md"
    print(f"plan {plan} sha256 {hashlib.sha256(open(plan,'rb').read()).hexdigest()}", flush=True)
    jobs = build_jobs(parts); done = {}
    if os.path.exists(CKPT):
        for line in open(CKPT):
            r = json.loads(line); done[r['key']] = r
    todo = [j for j in jobs if j[0] not in done]
    for part in OUTS: print(f"  {part}: {sum(j[1]['set']==part for j in jobs)} flights", flush=True)
    print(f"{len(jobs)} flights, {len(jobs)-len(todo)} already in {CKPT}, {len(todo)} to run", flush=True)
    if os.environ.get("E56_DRY"): sys.exit(0)
    W = int(os.environ.get("E56_WORKERS", "15")); t0 = time.time()
    if todo:
        with cf.ProcessPoolExecutor(max_workers=W) as ex, open(CKPT, "a") as pf:
            it = iter(todo); pending = set()
            def fill():                                                # keep submission in list order (check, confirm, gain)
                while len(pending) < W:
                    j = next(it, None)
                    if j is None: return
                    pending.add(ex.submit(_job, j))
            fill()
            while pending:
                fin, pending = cf.wait(pending, return_when=cf.FIRST_COMPLETED)
                for fut in fin:
                    r = fut.result(); done[r['key']] = r; pf.write(json.dumps(r) + "\n"); pf.flush()
                    print(f"[{time.time()-t0:6.0f}s {len(done)}/{len(jobs)}] {r['key']:26s} reached={r['completed']} crashed={r['crashed']} "
                          f"clear={r['min_clear_mm']:.1f}mm det={r['n_detected']}/{r['n_openings']} ({r['wall_s']:.0f}s)", flush=True)
                fill()
    write_csvs(jobs, done)
    print(f"done, wall {time.time()-t0:.0f}s", flush=True)
