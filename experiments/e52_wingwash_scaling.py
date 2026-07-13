"""
e52 — WING-WASH SCALING: does wing-wash's marginal benefit over feelers-only GROW as corridors
narrow, and/or as paths get longer/more complex?

Follow-up to e51/e51b, which found wing-wash contributes ~nothing on the primary randomized
ensemble (64-76mm, 2-3 legs) but helps modestly on the harder ensemble (56-64mm, 3-4 legs):
completion 45% vs 35%. Two candidate explanations, tested directly here:

  H1 (NARROWNESS)  wing-wash matters increasingly as corridors narrow (less room for tactile
                    course-correction, more reliance on a continuous restoring force).
  H2 (COMPLEXITY)   wing-wash's benefit accumulates over longer/more-turning paths, where small
                    lateral errors compound across legs.

Two factor sweeps, FULL vs NO_WINGWASH (identical ablation switch and courses to e51 -- same
generator pattern, same scoring, same gains, nothing retuned):

  FACTOR 1 (width): corridor width band swept {70-76,64-70,58-64,52-58,46-52}mm, legs FIXED at 3.
  FACTOR 2 (legs):  leg count swept {2,3,4,5,6}, width FIXED at the mid-range band (58-64mm).

N=20 courses per cell, IDENTICAL course set across FULL/NO_WINGWASH within a cell (same
generator seed), nominal noise (1.0), 1kHz, K_GE=1.0 (module default, untouched). Leg lengths
stay in the existing 160-260mm range throughout.

Reports per cell: completion (n/20), crash count (n/20, tracked SEPARATELY from completion --
a course can "complete" while having struck a wall mid-flight), mean/min clearance, per-passage
and per-opening detection, localization error. Then the FULL-minus-NO_WINGWASH delta as a
function of width and of leg count.

Run:  python experiments/e52_wingwash_scaling.py            (both sweeps)
      python experiments/e52_wingwash_scaling.py width       (just the width sweep)
      python experiments/e52_wingwash_scaling.py legs        (just the legs sweep)
      python experiments/e52_wingwash_scaling.py --combine   (merge partials into the report)
"""
import sys; sys.path.insert(0,'.'); import numpy as np, csv, pickle, os
import experiments.e47_realistic_course as e47
import experiments.e48_mission as e48
import experiments.e50_batch as e50
import experiments.e51_baselines as e51

LEFT={'E':'N','N':'W','W':'S','S':'E'}; RIGHT={'E':'S','S':'W','W':'N','N':'E'}

WIDTH_CELLS = [("70-76",0.070,0.076), ("64-70",0.064,0.070), ("58-64",0.058,0.064),
               ("52-58",0.052,0.058), ("46-52",0.046,0.052)]
WIDTH_FIXED_LEGS = 3
LEG_CELLS = [2,3,4,5,6]
LEGS_FIXED_WIDTH = (0.058, 0.064)   # mid-range band, = the "58-64" width cell

WIDTH_SEED_BASE = 5200
LEGS_SEED_BASE  = 5300
N_PER_CELL = 20

def sample_course_param(rng, wlo, whi, n_legs):
    """Generalized e50.sample_course: explicit width band + leg count; same breach-placement
    policy as e50's SAFE envelope (maxper=1 opening/leg, mid-leg placement, same accept window)."""
    dirs=['E']
    for _ in range(n_legs-1):
        dirs.append((LEFT if rng.random()<0.5 else RIGHT)[dirs[-1]])
    W=rng.uniform(wlo,whi,n_legs); L=rng.uniform(0.16,0.26,n_legs)
    legs=[(dirs[i],float(L[i]),float(W[i])) for i in range(n_legs)]
    breaches=[]
    for i in range(n_legs):
        cnt=int(rng.choice([0,1], p=[0.4,0.6]))
        for _ in range(cnt):
            side=int(rng.choice([+1,-1])); ln=float(rng.uniform(0.015,0.090))
            fc=rng.uniform(0.40,0.62)
            hf=(ln/L[i])/2; f0,f1=fc-hf,fc+hf
            lo,hi=(0.25,0.82)
            if f0>lo and f1<hi: breaches.append((side,i,float(f0),float(f1)))
    return legs, breaches

def generate_courses_param(N, wlo, whi, n_legs, seed):
    rng=np.random.default_rng(seed)
    courses=[]; skipped=0
    for k in range(N):
        for _ in range(40):
            legs,breaches=sample_course_param(rng, wlo, whi, n_legs)
            geo=e47.build_geometry(legs,breaches)
            if e50.valid(geo) and len(breaches)>0:
                break
        else:
            skipped+=1; continue
        courses.append((k, legs, breaches, geo))
    return courses, skipped

def clearance_stats(clear_list, crashed_list):
    c=np.array(clear_list)
    return dict(n=len(c), mean=float(c.mean()) if len(c) else float('nan'),
                min=float(c.min()) if len(c) else float('nan'),
                n_crashed=int(sum(crashed_list)))

def run_cell(tag_prefix, courses, n_workers):
    results={}
    for variant in ("FULL","NO_WINGWASH"):
        kwargs=e51.VARIANTS[variant]
        print(f"\n-- {tag_prefix} {variant} ({len(courses)} courses) --", flush=True)
        res=e51.run_variant_parallel(courses, kwargs, level=1.0, seed0=0,
                                      tag=f"{tag_prefix}_{variant}", n_workers=n_workers, rate=1000)
        results[variant]=res
        cst=clearance_stats(res['clear'], [r['crashed'] for r in res['percourse']])
        pass_rate = res['courses_found']/max(res['n_with_breach'],1)
        det_rate  = res['hits']/max(res['tot'],1)
        locs=np.array(res['locs'])
        print(f"   completion {res['comp']}/{res['runs']}  crashed {cst['n_crashed']}/{cst['n']}  "
              f"clearance mean={cst['mean']:.1f} min={cst['min']:.1f}mm  "
              f"per-passage {res['courses_found']}/{res['n_with_breach']}={100*pass_rate:.0f}%  "
              f"per-opening {res['hits']}/{res['tot']}={100*det_rate:.0f}%  "
              f"loc {locs.mean():.1f}+-{locs.std():.1f}mm (n={len(locs)})" if len(locs) else
              f"   completion {res['comp']}/{res['runs']}  crashed {cst['n_crashed']}/{cst['n']}  "
              f"clearance mean={cst['mean']:.1f} min={cst['min']:.1f}mm  no detections", flush=True)
    return results

def cell_row(cell_label, factor, results):
    rows=[]
    stat={}
    for variant in ("FULL","NO_WINGWASH"):
        res=results[variant]
        cst=clearance_stats(res['clear'], [r['crashed'] for r in res['percourse']])
        locs=np.array(res['locs'])
        stat[variant]=dict(comp=res['comp'], runs=res['runs'], crashed=cst['n_crashed'],
                            clear_mean=cst['mean'], clear_min=cst['min'],
                            hits=res['hits'], tot=res['tot'],
                            courses_found=res['courses_found'], n_with_breach=res['n_with_breach'],
                            loc_mean=float(locs.mean()) if len(locs) else float('nan'),
                            loc_std=float(locs.std()) if len(locs) else float('nan'),
                            n_loc=len(locs))
        for row in res['percourse']:
            rows.append(dict(factor=factor, cell=cell_label, variant=variant, course_id=row['course_id'],
                              n_legs=row['n_legs'], n_openings=row['n_openings'],
                              completed=row['completed'], crashed=row['crashed'],
                              min_clear_mm=row['min_clear_mm'], n_detected=row['n_detected'],
                              n_false_positives=row['n_false_positives'], loc_errors_mm=row['loc_errors_mm']))
    delta = dict(
        comp_delta = stat['FULL']['comp'] - stat['NO_WINGWASH']['comp'],
        crash_delta = stat['FULL']['crashed'] - stat['NO_WINGWASH']['crashed'],
        clear_min_delta = stat['FULL']['clear_min'] - stat['NO_WINGWASH']['clear_min'],
        clear_mean_delta = stat['FULL']['clear_mean'] - stat['NO_WINGWASH']['clear_mean'],
    )
    return rows, stat, delta

def save_csv(all_rows, cell_summaries, path="outputs/e52_wingwash_scaling.csv"):
    with open(path, "w", newline="") as fh:
        w=csv.writer(fh)
        w.writerow(["factor","cell","variant","course_id","n_legs","n_openings","completed",
                    "crashed","min_clear_mm","n_detected","n_false_positives","loc_errors_mm"])
        for row in all_rows:
            w.writerow([row['factor'],row['cell'],row['variant'],row['course_id'],row['n_legs'],
                        row['n_openings'],int(bool(row['completed'])),int(bool(row['crashed'])),
                        row['min_clear_mm'],row['n_detected'],row['n_false_positives'],
                        ";".join(f"{x:.3f}" for x in row['loc_errors_mm'])])
        w.writerow([])
        w.writerow(["factor","cell","variant","runs","completed","crashed","clear_mean_mm","clear_min_mm",
                    "hits","tot_openings","courses_found","n_with_breach","loc_mean_mm","loc_std_mm","n_loc"])
        for factor,cell,stat in cell_summaries:
            for variant in ("FULL","NO_WINGWASH"):
                s=stat[variant]
                w.writerow([factor,cell,variant,s['runs'],s['comp'],s['crashed'],s['clear_mean'],s['clear_min'],
                            s['hits'],s['tot'],s['courses_found'],s['n_with_breach'],s['loc_mean'],s['loc_std'],s['n_loc']])
        w.writerow([])
        w.writerow(["factor","cell","comp_delta_FULL_minus_NOWW","crash_delta","clear_mean_delta_mm","clear_min_delta_mm"])
        for factor,cell,stat in cell_summaries:
            f_,nw_ = stat['FULL'], stat['NO_WINGWASH']
            w.writerow([factor, cell, f_['comp']-nw_['comp'], f_['crashed']-nw_['crashed'],
                        f_['clear_mean']-nw_['clear_mean'], f_['clear_min']-nw_['clear_min']])
    print("saved ->", path)

def run_width_sweep(n_workers):
    print("\n########## FACTOR 1: WIDTH SWEEP (legs fixed at 3) ##########")
    all_rows=[]; cell_summaries=[]; deltas=[]
    for idx,(label,wlo,whi) in enumerate(WIDTH_CELLS):
        seed = WIDTH_SEED_BASE + idx
        courses, skipped = generate_courses_param(N_PER_CELL, wlo, whi, WIDTH_FIXED_LEGS, seed)
        print(f"\n=== width {label}mm (seed={seed}): {len(courses)} courses generated ({skipped} skipped) ===")
        results = run_cell(f"W{label}", courses, n_workers)
        rows, stat, delta = cell_row(label, "width", results)
        all_rows += rows; cell_summaries.append(("width", label, stat)); deltas.append((label, delta))
        with open(f"outputs/e52_width_{label}.pkl","wb") as fh: pickle.dump((rows,stat,delta), fh)
    print("\n--- WIDTH TREND: FULL minus NO_WINGWASH ---")
    print(" width(mm) | comp_delta | crash_delta | clear_mean_delta | clear_min_delta")
    for label,delta in deltas:
        print(f"  {label:>8} |    {delta['comp_delta']:+3d}     |    {delta['crash_delta']:+3d}     |"
              f"      {delta['clear_mean_delta']:+6.1f}      |     {delta['clear_min_delta']:+6.1f}")
    return all_rows, cell_summaries, deltas

def run_legs_sweep(n_workers):
    print("\n########## FACTOR 2: LEG-COUNT SWEEP (width fixed at 58-64mm) ##########")
    all_rows=[]; cell_summaries=[]; deltas=[]
    wlo,whi = LEGS_FIXED_WIDTH
    for idx,n_legs in enumerate(LEG_CELLS):
        seed = LEGS_SEED_BASE + idx
        courses, skipped = generate_courses_param(N_PER_CELL, wlo, whi, n_legs, seed)
        label=str(n_legs)
        print(f"\n=== legs={n_legs} (seed={seed}): {len(courses)} courses generated ({skipped} skipped) ===")
        results = run_cell(f"L{n_legs}", courses, n_workers)
        rows, stat, delta = cell_row(label, "legs", results)
        all_rows += rows; cell_summaries.append(("legs", label, stat)); deltas.append((label, delta))
        with open(f"outputs/e52_legs_{n_legs}.pkl","wb") as fh: pickle.dump((rows,stat,delta), fh)
    print("\n--- LEG-COUNT TREND: FULL minus NO_WINGWASH ---")
    print(" legs | comp_delta | crash_delta | clear_mean_delta | clear_min_delta")
    for label,delta in deltas:
        print(f"  {label:>4} |    {delta['comp_delta']:+3d}     |    {delta['crash_delta']:+3d}     |"
              f"      {delta['clear_mean_delta']:+6.1f}      |     {delta['clear_min_delta']:+6.1f}")
    return all_rows, cell_summaries, deltas

if __name__ == "__main__":
    os.makedirs("outputs", exist_ok=True)
    N_WORKERS = int(os.environ.get("E52_WORKERS", str(min(14, os.cpu_count() or 4))))
    which = sys.argv[1] if len(sys.argv) > 1 else None

    if which == "--combine":
        all_rows=[]; cell_summaries=[]
        for idx,(label,_,_) in enumerate(WIDTH_CELLS):
            with open(f"outputs/e52_width_{label}.pkl","rb") as fh: rows,stat,delta=pickle.load(fh)
            all_rows+=rows; cell_summaries.append(("width",label,stat))
        for n_legs in LEG_CELLS:
            with open(f"outputs/e52_legs_{n_legs}.pkl","rb") as fh: rows,stat,delta=pickle.load(fh)
            all_rows+=rows; cell_summaries.append(("legs",str(n_legs),stat))
        save_csv(all_rows, cell_summaries)
        sys.exit(0)

    all_rows=[]; cell_summaries=[]
    if which in (None, "width"):
        rows, summ, _ = run_width_sweep(N_WORKERS); all_rows+=rows; cell_summaries+=summ
    if which in (None, "legs"):
        rows, summ, _ = run_legs_sweep(N_WORKERS); all_rows+=rows; cell_summaries+=summ

    if which is None:
        save_csv(all_rows, cell_summaries)
