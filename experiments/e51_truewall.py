"""
e51_truewall — re-run of the e51 primary-ensemble ablation under the corrected (true-geometry)
aero wall construction (branch truewall-fix).

Identical to e51_baselines.py in every respect that matters -- same course generator and seed
(e51.generate_courses, seed 1234), same flight seeds (seed0+k, seed0=0), same noise level (1.0),
same control rate (1 kHz), same four VARIANTS, same scoring (e50.score_quiet via
e51._run_one_course) -- but:
  * writes to outputs/e51_baselines_truewall.csv (never touches outputs/e51_baselines.csv or the
    e51_partial_*.pkl files), and refuses to overwrite an existing output file;
  * also persists crashed / min_clear_mm per course (the original e51 CSV omitted them; e51b
    part1 re-flew FULL and NO_WINGWASH to get them);
  * runs all 4 x 40 flights in one process pool;
  * checkpoints every finished flight to <OUT>.partial.jsonl and resumes from it, so an
    interrupted run loses nothing (flights are deterministic given course + seed).

Run:  python experiments/e51_truewall.py
      E51_N=1 E51_ONLY=FULL E51_OUT=/tmp/x.csv python experiments/e51_truewall.py   (smoke test)
"""
import sys; sys.path.insert(0,'.'); import numpy as np, csv, os, time, glob, json
import concurrent.futures as cf
import experiments.e51_baselines as e51

OUT = os.environ.get("E51_OUT", "outputs/e51_baselines_truewall.csv")

def _job(args):
    t0 = time.time(); res = e51._run_one_course(args); res['wall_s'] = time.time()-t0; res['tag'] = args[7]
    for p in glob.glob(f"models/_e51_{args[7]}_{args[0]}_{os.getpid()}.xml"): os.remove(p)
    return res

if __name__ == "__main__":
    if os.path.exists(OUT): sys.exit(f"refusing to overwrite existing {OUT}")
    N = int(os.environ.get("E51_N", "40")); W = int(os.environ.get("E51_WORKERS", str(os.cpu_count() or 4)))
    only = os.environ.get("E51_ONLY"); tags = [only] if only else list(e51.VARIANTS)
    courses = e51.generate_courses(N=N, hard=False)
    print(f"{len(courses)} courses x {tags}, {W} workers", flush=True)
    jobs = [(k, legs, br, geo, e51.VARIANTS[tag], 1.0, 0, "tw"+tag, 1000)
            for tag in tags for (k, legs, br, geo) in courses]
    out = {tag: {} for tag in tags}; t0 = time.time(); PART = OUT + ".partial.jsonl"
    if os.path.exists(PART):
        for line in open(PART):
            r = json.loads(line)
            if r['tag'][2:] in out: out[r['tag'][2:]][r['course_id']] = r
        print(f"resumed {sum(len(v) for v in out.values())} finished flights from {PART}", flush=True)
    jobs = [j for j in jobs if j[0] not in out[j[7][2:]]]
    with cf.ProcessPoolExecutor(max_workers=W) as ex, open(PART, "a") as pf:
        for fut in cf.as_completed([ex.submit(_job, j) for j in jobs]):
            r = fut.result(); tag = r['tag'][2:]; out[tag][r['course_id']] = r
            pf.write(json.dumps(r, default=lambda o: o.item() if hasattr(o, 'item') else list(o)) + "\n"); pf.flush()
            print(f"[{time.time()-t0:6.0f}s] {tag:12s}{r['msg']}  crashed={int(r['row']['crashed'])} ({r['wall_s']:.0f}s)", flush=True)
    with open(OUT, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["variant","course_id","n_legs","n_openings","completed","crashed","min_clear_mm",
                    "n_detected","n_false_positives","loc_errors_mm"])
        for tag in tags:
            for k in sorted(out[tag]):
                row = out[tag][k]['row']
                w.writerow([tag, k, row['n_legs'], row['n_openings'], int(bool(row['completed'])),
                            int(bool(row['crashed'])), repr(float(row['min_clear_mm'])), row['n_detected'],
                            row['n_false_positives'], ";".join(f"{x:.3f}" for x in row['loc_errors_mm'])])
        w.writerow([])
        w.writerow(["variant","runs","completed","comp_rate","hits","tot_openings","detect_rate",
                    "courses_found","n_with_breach","per_passage_rate","loc_mean_mm","loc_std_mm",
                    "n_loc","fp_total","fp_per_course"])
        for tag in tags:
            R = [out[tag][k] for k in sorted(out[tag])]; C = [r for r in R if r['reached']]
            hits = sum(r['h'] for r in C); tot = sum(r['t'] for r in C); fp = sum(r['fp'] for r in C)
            found = sum(r['h'] >= 1 for r in C); locs = np.array([x for r in C for x in r['locs']])
            w.writerow([tag, len(R), len(C), len(C)/max(len(R),1), hits, tot, hits/max(tot,1), found, len(C),
                        found/max(len(C),1), locs.mean() if len(locs) else "", locs.std() if len(locs) else "",
                        len(locs), fp, fp/max(len(C),1)])
    print(f"saved -> {OUT}   total wall {time.time()-t0:.0f}s", flush=True)
