"""
e51b — FOLLOW-UP to e51: does wing-wash contribute on axes the primary-ensemble headline table
(completion / detection) did NOT measure?

Four checks, all on the SAME fairness rules as e51 (identical courses per part, no gain retuning,
same scoring, K_GE untouched):

  1. CLEARANCE/SAFETY MARGIN -- variants A (FULL) and B (NO_WINGWASH) on the identical 40 primary
     courses from e51. Reports min clearance per course and how many courses come within 5mm of
     the wingtip-strike radius (WINGREACH=13.3mm), i.e. min_clear_mm < 18.3mm.
  2. HARD ENSEMBLE -- all four variants on e50's 20-course harder ensemble (narrower corridors,
     3-4 legs, openings near turns), same 5 metrics as e51 plus clearance.
  3. CONTROL-RATE ROBUSTNESS -- variants A and B at 1000/500/250 Hz, on a representative N-course
     subset of the primary ensemble (subset is a deterministic PREFIX of the same course sequence,
     so it's the identical courses 0..N-1 that appear in the full 40-course set).
  4. E40 CRAFTED WORST CASE -- variant B (use_wingwash=False, fuse=True, i.e. feelers+veto only)
     on e40's side-gap sweep, the case that already stresses pure wing-wash centring.

Run:  python experiments/e51b_followup.py [part1|part2|part3|part4]   (default: all four)
"""
import sys; sys.path.insert(0,'.'); import numpy as np, csv, pickle, os
import experiments.e51_baselines as e51
import experiments.e40_side_gap as e40
from src.safety import WINGREACH

WINGREACH_MM = WINGREACH * 1e3
NEAR_STRIKE_MM = WINGREACH_MM + 5.0   # "within 5mm of the wingtip-strike radius"

def clearance_stats(clear_list, crashed_list=None):
    c = np.array(clear_list)
    near = int(np.sum(c < NEAR_STRIKE_MM))
    ncrash = int(sum(crashed_list)) if crashed_list is not None else None
    return dict(n=len(c), mean=float(c.mean()) if len(c) else float('nan'),
                std=float(c.std()) if len(c) else float('nan'),
                min=float(c.min()) if len(c) else float('nan'),
                n_near_strike=near, n_crashed=ncrash)

# ---------------------------------------------------------------- part 1: clearance, A vs B, primary
def part1_clearance(n_workers):
    print("\n########## PART 1: clearance/safety margin, A vs B, 40 primary courses ##########")
    courses = e51.generate_courses(N=40, hard=False)
    rows = []
    results = {}
    for tag in ("FULL", "NO_WINGWASH"):
        print(f"-- {tag} --")
        res = e51.run_variant_parallel(courses, e51.VARIANTS[tag], level=1.0, seed0=0, tag=tag, n_workers=n_workers)
        results[tag] = res
        for row in res['percourse']:
            rows.append(dict(part="part1", variant=tag, course_id=row['course_id'],
                              completed=row['completed'], crashed=row['crashed'],
                              min_clear_mm=row['min_clear_mm']))
        st = clearance_stats(res['clear'], [r['crashed'] for r in res['percourse']])
        print(f"  {tag}: clearance mean={st['mean']:.1f} std={st['std']:.1f} min={st['min']:.1f} mm "
              f"| within {NEAR_STRIKE_MM:.1f}mm of strike: {st['n_near_strike']}/{st['n']} "
              f"| crashed: {st['n_crashed']}/{st['n']}")
    return rows, results

# ---------------------------------------------------------------- part 2: hard ensemble, all 4 variants
def part2_hard(n_workers):
    print("\n########## PART 2: hard ensemble (20 courses), all 4 variants ##########")
    courses = e51.generate_courses(N=20, hard=True)
    print(f"  -> {len(courses)} hard courses generated (expect 20)")
    rows = []
    results = {}
    for tag in e51.VARIANTS:
        print(f"-- {tag} --")
        res = e51.run_variant_parallel(courses, e51.VARIANTS[tag], level=1.0, seed0=0, tag=f"H_{tag}", n_workers=n_workers)
        results[tag] = res
        for row in res['percourse']:
            rows.append(dict(part="part2", variant=tag, course_id=row['course_id'],
                              n_legs=row['n_legs'], n_openings=row['n_openings'],
                              completed=row['completed'], crashed=row['crashed'],
                              min_clear_mm=row['min_clear_mm'], n_detected=row['n_detected'],
                              n_false_positives=row['n_false_positives'],
                              loc_errors_mm=row['loc_errors_mm']))
        e51.report(tag, res)
        st = clearance_stats(res['clear'], [r['crashed'] for r in res['percourse']])
        print(f"  clearance mean={st['mean']:.1f} std={st['std']:.1f} min={st['min']:.1f} mm "
              f"| within {NEAR_STRIKE_MM:.1f}mm of strike: {st['n_near_strike']}/{st['n']} "
              f"| crashed: {st['n_crashed']}/{st['n']}")
    return rows, results

# ---------------------------------------------------------------- part 3: control-rate robustness
def part3_rate(n_workers, n_subset):
    print(f"\n########## PART 3: control-rate robustness, A vs B, {n_subset}-course subset ##########")
    courses = e51.generate_courses(N=n_subset, hard=False)   # deterministic prefix of the same 40-course sequence
    print(f"  -> {len(courses)} courses (courses 0..{n_subset-1} of the primary sequence)")
    rows = []
    results = {}
    for tag in ("FULL", "NO_WINGWASH"):
        for rate in (1000, 500, 250):
            key = f"{tag}@{rate}Hz"
            print(f"-- {key} --")
            res = e51.run_variant_parallel(courses, e51.VARIANTS[tag], level=1.0, seed0=0,
                                            tag=f"R{rate}_{tag}", n_workers=n_workers, rate=rate)
            results[key] = res
            for row in res['percourse']:
                rows.append(dict(part="part3", variant=tag, rate_hz=rate, course_id=row['course_id'],
                                  completed=row['completed'], crashed=row['crashed'],
                                  min_clear_mm=row['min_clear_mm']))
            st = clearance_stats(res['clear'], [r['crashed'] for r in res['percourse']])
            print(f"  {key}: completion {res['comp']}/{res['runs']} | clearance mean={st['mean']:.1f} "
                  f"std={st['std']:.1f} min={st['min']:.1f} mm | crashed {st['n_crashed']}/{st['n']}")
    return rows, results

# ---------------------------------------------------------------- part 4: e40 crafted worst case
def part4_e40():
    print("\n########## PART 4: e40 crafted worst case, variant B (feelers+veto, no wing-wash) ##########")
    gmults = (0.5, 1.0, 1.5, 2.0, 3.0)
    rows = []
    for gm in gmults:
        r = e40.run(gm * e40.e.Wd, level=0.0, seed=0, rec=False, fuse=True, use_wingwash=False)
        rows.append(dict(part="part4", variant="NO_WINGWASH", gap_x_Wd=gm, gap_mm=r['gap_mm'],
                          outcome=r['reached'], peak_y_mm=r['lurch_mm'], min_clear_mm=r['min_clear_mm'],
                          crashed=r['crashed']))
        print(f"  gap {gm:>3}xWd ({r['gap_mm']:3.0f}mm): {str(r['reached']):7}  peak +y {r['lurch_mm']:5.1f}mm  "
              f"minClear {r['min_clear_mm']:4.1f}mm  {'CRASH' if r['crashed'] else ''}")
    print("  reference (already verified earlier this session, e40 A/C variants, level=0.0):")
    print("    A (FULL/fused)   : 3.0 / 8.8 / 8.8 / 8.8 / 8.8 mm, all completed")
    print("    C (NO_FEELERS/base wing-wash-only): 3.1 / 15.0(CRASH) / 25.6(stall) / 62.0(CRASH,exit) / 62.0(exit) mm")
    return rows

def save_csv(all_rows, path="outputs/e51b_ablation_followup.csv"):
    fields = ["part","variant","rate_hz","course_id","n_legs","n_openings","completed","crashed",
              "min_clear_mm","n_detected","n_false_positives","loc_errors_mm",
              "gap_x_Wd","gap_mm","outcome","peak_y_mm"]
    with open(path, "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for row in all_rows:
            r = dict(row)
            if 'loc_errors_mm' in r and isinstance(r['loc_errors_mm'], list):
                r['loc_errors_mm'] = ";".join(f"{x:.3f}" for x in r['loc_errors_mm'])
            if 'completed' in r: r['completed'] = int(bool(r['completed']))
            if 'crashed' in r: r['crashed'] = int(bool(r['crashed']))
            w.writerow(r)
    print("saved ->", path)

if __name__ == "__main__":
    os.makedirs("outputs", exist_ok=True)
    N_WORKERS = int(os.environ.get("E51_WORKERS", str(min(14, os.cpu_count() or 4))))
    N_SUBSET = int(os.environ.get("E51B_RATE_N", "15"))

    which = sys.argv[1] if len(sys.argv) > 1 else None
    all_rows = []

    if which in (None, "part1"):
        rows, _ = part1_clearance(N_WORKERS); all_rows += rows
        with open(f"outputs/e51b_part1.pkl","wb") as fh: pickle.dump(rows, fh)
    if which in (None, "part2"):
        rows, _ = part2_hard(N_WORKERS); all_rows += rows
        with open(f"outputs/e51b_part2.pkl","wb") as fh: pickle.dump(rows, fh)
    if which in (None, "part3"):
        rows, _ = part3_rate(N_WORKERS, N_SUBSET); all_rows += rows
        with open(f"outputs/e51b_part3.pkl","wb") as fh: pickle.dump(rows, fh)
    if which in (None, "part4"):
        rows = part4_e40(); all_rows += rows
        with open(f"outputs/e51b_part4.pkl","wb") as fh: pickle.dump(rows, fh)

    if which is None:
        save_csv(all_rows)
    else:
        with open(f"outputs/e51b_{which}_rows.pkl","wb") as fh: pickle.dump(all_rows, fh)
        print(f"saved partial rows -> outputs/e51b_{which}_rows.pkl (run with no arg, or all 4 parts, then combine manually)")
