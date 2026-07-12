"""
e51 — BASELINE ABLATIONS: how much does each sensing channel actually contribute?

Runs the SAME 40 randomized primary-ensemble courses (identical generator, identical seed 1234,
identical rejection-sampling loop as e50.batch(N=40, hard=False)) through four control variants,
changing ONLY the sensing/control channel:

  FULL         wing-wash centring + feelers + veto fusion (= e50's existing primary result)
  NO_WINGWASH  feelers only (wing-wash centring term dropped)
  NO_FEELERS   wing-wash only (feeler steering/veto/detection dropped)
  OPEN_LOOP    dead-reckoning: constant speed, initial heading, no sensing at all

Same scoring (e50.score_quiet, 60mm match tolerance, completed-courses-only), same noise level
(1.0), same control rate (1kHz), same K_GE (module default 1.0, untouched by this file). No gains
are retuned for any variant -- the channel is removed, not compensated for.

The 40 courses are generated ONCE (generate_courses) and the identical list of (legs, breaches,
geo) objects is reused for every variant, so "same course set" is guaranteed by construction, not
just by seed reproducibility.

Run:  python experiments/e51_baselines.py            (all 4 variants, sequential)
      python experiments/e51_baselines.py FULL        (just one variant -- for parallel launch)
      python experiments/e51_baselines.py --combine   (merge the 4 partial pickles into the report)
"""
import sys; sys.path.insert(0,'.'); import numpy as np, csv, pickle, os
import experiments.e47_realistic_course as e47
import experiments.e48_mission as e48
import experiments.e50_batch as e50

def generate_courses(N=40, hard=False):
    """Identical course-generation loop to e50.batch(): same seed, same rejection sampling.
    Returns a list of (course_id, legs, breaches, geo) -- generated ONCE, reused by every variant."""
    rng = np.random.default_rng(1234 + (1 if hard else 0))
    courses = []
    for k in range(N):
        for _ in range(40):
            legs, breaches = e50.sample_course(rng, hard)
            geo = e47.build_geometry(legs, breaches)
            if e50.valid(geo) and len(breaches) > 0:
                break
        else:
            continue
        courses.append((k, legs, breaches, geo))
    return courses

VARIANTS = {
    "FULL":        dict(fuse=True,  use_wingwash=True,  use_feelers=True,  open_loop=False),
    "NO_WINGWASH": dict(fuse=True,  use_wingwash=False, use_feelers=True,  open_loop=False),
    "NO_FEELERS":  dict(fuse=False, use_wingwash=True,  use_feelers=False, open_loop=False),
    "OPEN_LOOP":   dict(fuse=False, use_wingwash=False, use_feelers=False, open_loop=True),
}

def run_variant(courses, kwargs, level=1.0, seed0=0, verbose=False, tag="", rate=1000):
    """Serial reference implementation (kept for the smoke test / small N)."""
    comp=0; runs=0; all_hits=all_tot=all_fp=0; all_locs=[]; n_with_breach=0; courses_found=0
    percourse=[]; all_clear=[]
    for (k, legs, breaches, geo) in courses:
        r = e48.run(geo, level=level, seed=seed0+k, rate=rate, tmax=None, **kwargs)
        runs += 1
        reached = r['reached'] == 'finish'
        comp += reached
        all_clear.append(r['min_clear_mm'])
        row = dict(course_id=k, n_legs=len(legs), n_openings=len(breaches),
                   completed=reached, crashed=r['crashed'], min_clear_mm=r['min_clear_mm'],
                   n_detected=0, n_false_positives=0, loc_errors_mm=[])
        if reached:
            h, t, fp, locs = e50.score_quiet(geo, breaches, r['detected'])
            all_hits += h; all_tot += t; all_fp += fp; all_locs += locs; n_with_breach += 1
            if h >= 1: courses_found += 1
            row.update(n_detected=h, n_false_positives=fp, loc_errors_mm=locs)
            if verbose: print(f"  [{tag} {k}] reached=Y det={h}/{t} fp={fp} clear={r['min_clear_mm']:.1f}mm", flush=True)
        elif verbose:
            print(f"  [{tag} {k}] reached=N ({r['reached']}) clear={r['min_clear_mm']:.1f}mm", flush=True)
        percourse.append(row)
    return dict(runs=runs, comp=comp, hits=all_hits, tot=all_tot, fp=all_fp, locs=all_locs,
                n_with_breach=n_with_breach, courses_found=courses_found, percourse=percourse, clear=all_clear)

def _run_one_course(args):
    """Top-level (picklable) worker: runs ONE course through e48.run() + scoring, for use with
    ProcessPoolExecutor. Each worker builds its own fresh Flyer/model -- no shared state."""
    (k, legs, breaches, geo, kwargs, level, seed0, tag, rate) = args
    model_path = f"models/_e51_{tag}_{k}_{os.getpid()}.xml"   # unique per (variant, course, process) -- avoids the build_model() shared-file race under parallel workers
    r = e48.run(geo, level=level, seed=seed0+k, rate=rate, tmax=None, model_path=model_path, **kwargs)
    reached = r['reached'] == 'finish'
    row = dict(course_id=k, n_legs=len(legs), n_openings=len(breaches),
               completed=reached, crashed=r['crashed'], min_clear_mm=r['min_clear_mm'],
               n_detected=0, n_false_positives=0, loc_errors_mm=[])
    h=t=fp=0; locs=[]
    if reached:
        h, t, fp, locs = e50.score_quiet(geo, breaches, r['detected'])
        row.update(n_detected=h, n_false_positives=fp, loc_errors_mm=locs)
    msg = (f"  [{tag} {k}] reached=Y det={h}/{t} fp={fp} clear={r['min_clear_mm']:.1f}mm" if reached
           else f"  [{tag} {k}] reached=N ({r['reached']}) clear={r['min_clear_mm']:.1f}mm")
    return dict(course_id=k, reached=reached, h=h, t=t, fp=fp, locs=locs, row=row, msg=msg,
                min_clear_mm=r['min_clear_mm'])

def run_variant_parallel(courses, kwargs, level=1.0, seed0=0, tag="", n_workers=12, rate=1000):
    """Same result as run_variant(), but runs the courses of ONE variant concurrently across
    n_workers OS processes (each course flight is ~90s-25min of single-core work, and courses
    are fully independent, so this parallelizes cleanly)."""
    import concurrent.futures as cf
    args = [(k, legs, breaches, geo, kwargs, level, seed0, tag, rate) for (k, legs, breaches, geo) in courses]
    results_by_k = {}
    with cf.ProcessPoolExecutor(max_workers=n_workers) as ex:
        futures = {ex.submit(_run_one_course, a): a[0] for a in args}
        for fut in cf.as_completed(futures):
            res = fut.result()
            print(res['msg'], flush=True)
            results_by_k[res['course_id']] = res
    comp=0; runs=0; all_hits=all_tot=all_fp=0; all_locs=[]; n_with_breach=0; courses_found=0; percourse=[]; all_clear=[]
    for k in sorted(results_by_k):
        res = results_by_k[k]
        runs += 1; comp += res['reached']; all_clear.append(res['min_clear_mm'])
        if res['reached']:
            all_hits += res['h']; all_tot += res['t']; all_fp += res['fp']; all_locs += res['locs']
            n_with_breach += 1
            if res['h'] >= 1: courses_found += 1
        percourse.append(res['row'])
    return dict(runs=runs, comp=comp, hits=all_hits, tot=all_tot, fp=all_fp, locs=all_locs,
                n_with_breach=n_with_breach, courses_found=courses_found, percourse=percourse, clear=all_clear)

def report(tag, res):
    locs = np.array(res['locs'])
    comp_rate  = res['comp'] / max(res['runs'], 1)
    det_rate   = res['hits'] / max(res['tot'], 1)
    pass_rate  = res['courses_found'] / max(res['n_with_breach'], 1)
    fp_rate    = res['fp'] / max(res['comp'], 1)
    print(f"\n=== {tag} ===")
    print(f"  completion          : {res['comp']}/{res['runs']} = {100*comp_rate:.0f}%")
    print(f"  per-passage detect  : {res['courses_found']}/{res['n_with_breach']} = {100*pass_rate:.0f}%")
    print(f"  per-opening detect  : {res['hits']}/{res['tot']} = {100*det_rate:.0f}%")
    if len(locs):
        print(f"  localization error  : {locs.mean():.1f} +/- {locs.std():.1f} mm (n={len(locs)})")
    else:
        print(f"  localization error  : n/a (no detections)")
    print(f"  false positives     : {res['fp']} over {res['comp']} completed courses ({fp_rate:.2f}/course)")

def save_csv(results, path="outputs/e51_baselines.csv"):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["variant","course_id","n_legs","n_openings","completed",
                    "n_detected","n_false_positives","loc_errors_mm"])
        for tag, res in results.items():
            for row in res['percourse']:
                w.writerow([tag, row['course_id'], row['n_legs'], row['n_openings'],
                            int(bool(row['completed'])), row['n_detected'], row['n_false_positives'],
                            ";".join(f"{x:.3f}" for x in row['loc_errors_mm'])])
        w.writerow([])
        w.writerow(["variant","runs","completed","comp_rate","hits","tot_openings","detect_rate",
                    "courses_found","n_with_breach","per_passage_rate","loc_mean_mm","loc_std_mm",
                    "n_loc","fp_total","fp_per_course"])
        for tag, res in results.items():
            locs = np.array(res['locs'])
            w.writerow([tag, res['runs'], res['comp'], res['comp']/max(res['runs'],1),
                        res['hits'], res['tot'], res['hits']/max(res['tot'],1),
                        res['courses_found'], res['n_with_breach'],
                        res['courses_found']/max(res['n_with_breach'],1),
                        locs.mean() if len(locs) else "", locs.std() if len(locs) else "",
                        len(locs), res['fp'], res['fp']/max(res['comp'],1)])
    print("saved ->", path)

def correctness_check(results):
    full = results["FULL"]
    print("\n##### correctness check: FULL vs known e50 Table 13 primary numbers #####")
    print(f"  completion   : {full['comp']}/{full['runs']}  (expect 34/40)")
    print(f"  per-opening  : {full['hits']}/{full['tot']}  (expect 38/56)")
    print(f"  per-passage  : {full['courses_found']}/{full['n_with_breach']}  (expect 28/34)")
    print(f"  false pos    : {full['fp']}  (expect 1)")
    ok = (full['comp']==34 and full['runs']==40 and full['hits']==38 and full['tot']==56
          and full['courses_found']==28 and full['n_with_breach']==34 and full['fp']==1)
    print(f"  => {'MATCH -- ablation harness is faithful to e50' if ok else 'MISMATCH -- investigate before trusting the other variants'}")

if __name__ == "__main__":
    os.makedirs("outputs", exist_ok=True)

    if len(sys.argv) > 1 and sys.argv[1] == "--combine":
        results = {}
        for tag in VARIANTS:
            with open(f"outputs/e51_partial_{tag}.pkl", "rb") as fh:
                results[tag] = pickle.load(fh)
        for tag, res in results.items(): report(tag, res)
        save_csv(results)
        correctness_check(results)
        sys.exit(0)

    N_COURSES = int(os.environ.get("E51_N", "40"))
    N_WORKERS = int(os.environ.get("E51_WORKERS", str(min(14, os.cpu_count() or 4))))

    print(f"generating primary-ensemble courses (N={N_COURSES}, seed=1234, identical to e50.batch)...")
    courses = generate_courses(N=N_COURSES, hard=False)
    print(f"  -> {len(courses)} courses generated (expect {N_COURSES})")

    only = sys.argv[1] if len(sys.argv) > 1 else None
    tags = [only] if only else list(VARIANTS.keys())

    results = {}
    for tag in tags:
        kwargs = VARIANTS[tag]
        print(f"\n##### running variant {tag}: {kwargs}  (parallel, {N_WORKERS} workers) #####")
        results[tag] = run_variant_parallel(courses, kwargs, level=1.0, seed0=0, tag=tag, n_workers=N_WORKERS)
        report(tag, results[tag])
        with open(f"outputs/e51_partial_{tag}.pkl", "wb") as fh:
            pickle.dump(results[tag], fh)
        print(f"saved -> outputs/e51_partial_{tag}.pkl")

    if not only:
        save_csv(results)
        correctness_check(results)
