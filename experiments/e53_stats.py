"""
e53_stats — statistics protocol on the corrected (true-wall) harness: FULL vs NO_WINGWASH only.

  pass1     primary 40-course ensemble (e51.generate_courses, seed 1234) x 5 noise seeds per course,
            noise seed MATCHED across the two variants: noise_seed = 1000*j + course_id, j=1..5.
            (The e51 runs used noise_seed = course_id, so all pass1 seeds are fresh.)
            -> outputs/stats_pass1.csv
  legsweep  e52 leg-count sweep (2..6 legs, 20 courses/cell, width 58-64mm, generator seeds
            5300+idx -- the SAME courses as e52) with FRESH noise seeds: noise_seed = 7000 + course_id
            (e52 used noise_seed = course_id).
            -> outputs/stats_legsweep.csv

Controller, generator, scoring and the fix are untouched: every flight goes through
e51._run_one_course -> e48.run + e50.score_quiet, noise level 1.0, 1 kHz.
Every finished flight is checkpointed to <OUT>.partial.jsonl; rerunning resumes. Refuses to
overwrite an existing output CSV.

Run:  python experiments/e53_stats.py pass1|legsweep
      E53_LIMIT=1 E53_OUT=/tmp/x.csv python experiments/e53_stats.py pass1     (smoke test)
"""
import sys; sys.path.insert(0,'.'); import numpy as np, csv, os, time, glob, json
import concurrent.futures as cf
import experiments.e51_baselines as e51
import experiments.e52_wingwash_scaling as e52

VARIANTS = ("FULL", "NO_WINGWASH")
PASS1_SEEDS = (1, 2, 3, 4, 5); PASS1_SEED_STRIDE = 1000
LEGSWEEP_SEED0 = 7000

def build_jobs(mode):
    """-> list of (key, meta, e51-args). key is unique per flight and stable across restarts."""
    jobs = []
    if mode == "pass1":
        for (k, legs, br, geo) in e51.generate_courses(N=40, hard=False):
            for j in PASS1_SEEDS:
                for v in VARIANTS:
                    seed0 = PASS1_SEED_STRIDE*j; tag = f"p1{v}s{j}"
                    meta = dict(set="pass1", cell="primary", variant=v, course_id=k, seed_idx=j, noise_seed=seed0+k)
                    jobs.append((f"primary|{k}|{j}|{v}", meta, (k, legs, br, geo, e51.VARIANTS[v], 1.0, seed0, tag, 1000)))
    elif mode == "legsweep":
        wlo, whi = e52.LEGS_FIXED_WIDTH
        for idx, n_legs in enumerate(e52.LEG_CELLS):
            courses, _ = e52.generate_courses_param(e52.N_PER_CELL, wlo, whi, n_legs, e52.LEGS_SEED_BASE+idx)
            for (k, legs, br, geo) in courses:
                for v in VARIANTS:
                    tag = f"ls{n_legs}{v}"
                    meta = dict(set="legsweep", cell=str(n_legs), variant=v, course_id=k, seed_idx=0, noise_seed=LEGSWEEP_SEED0+k)
                    jobs.append((f"{n_legs}|{k}|0|{v}", meta, (k, legs, br, geo, e51.VARIANTS[v], 1.0, LEGSWEEP_SEED0, tag, 1000)))
    else:
        sys.exit("mode must be pass1 or legsweep")
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
    mode = sys.argv[1] if len(sys.argv) > 1 else ""
    OUT = os.environ.get("E53_OUT", f"outputs/stats_{mode}.csv"); PART = OUT + ".partial.jsonl"
    if os.path.exists(OUT): sys.exit(f"refusing to overwrite existing {OUT}")
    W = int(os.environ.get("E53_WORKERS", str(os.cpu_count() or 4)))
    jobs = build_jobs(mode); lim = os.environ.get("E53_LIMIT")
    if lim: jobs = jobs[:int(lim)]
    done = {}
    if os.path.exists(PART):
        for line in open(PART):
            r = json.loads(line); done[r['key']] = r
        print(f"resumed {len(done)} finished flights from {PART}", flush=True)
    todo = [j for j in jobs if j[0] not in done]
    print(f"{mode}: {len(jobs)} flights, {len(todo)} to run, {W} workers", flush=True); t0 = time.time()
    with cf.ProcessPoolExecutor(max_workers=W) as ex, open(PART, "a") as pf:
        for fut in cf.as_completed([ex.submit(_job, j) for j in todo]):
            r = fut.result(); done[r['key']] = r; pf.write(json.dumps(r) + "\n"); pf.flush()
            print(f"[{time.time()-t0:6.0f}s {len(done)}/{len(jobs)}] {r['key']:28s} reached={r['completed']} crashed={r['crashed']} "
                  f"clear={r['min_clear_mm']:.1f}mm det={r['n_detected']}/{r['n_openings']} ({r['wall_s']:.0f}s)", flush=True)
    with open(OUT, "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(COLS)
        for key, _, _ in jobs: w.writerow([repr(done[key][c]) if c == "min_clear_mm" else done[key][c] for c in COLS])
    print(f"saved -> {OUT}   wall {time.time()-t0:.0f}s", flush=True)
