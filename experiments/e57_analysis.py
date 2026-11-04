"""
e57_analysis — analysis of the corridor-width sweep exactly as fixed in docs/e57_analysis_plan.md.

  Primary (H1): paired FULL - NO_WINGWASH in strike-free completion, pooled over the two narrowest bands
           (52-58 and 46-52 mm), 95% cluster-bootstrap CI over courses, exact McNemar, pre-stated verdict.
  Secondary (descriptive): each band, all bands pooled; reached, end-to-end detection, mean min-clearance.
  Sensitivity: strike radius 13.6 mm with no goal-zone exception (strike_free := completed and min_clear >= 13.6).

Reads outputs/e57_runs.jsonl (works on a partial run; only complete pairs are used, and no verdict is
printed until every pair of the primary comparison is present).

Run:  python experiments/e57_analysis.py
"""
import sys, json, numpy as np
sys.path.insert(0, 'experiments'); sys.path.insert(0, '.')
from e53_analysis import mcnemar_exact
from e56_analysis import boot, verdict, pair_rows

BANDS = ["70-76", "64-70", "58-64", "52-58", "46-52"]; NARROW = ("52-58", "46-52"); N_PER_BAND = 40; TIP_REACH_MM = 13.6

def line(label, PC):
    A = np.vstack(PC); n = len(A)
    sf = lambda X: 100*np.mean(X[:,0]-X[:,1]); rc = lambda X: 100*np.mean(X[:,2]-X[:,3])
    det = lambda X: 100*(X[:,4].sum()-X[:,5].sum())/max(X[:,6].sum(), 1); clr = lambda X: np.mean(X[:,7]-X[:,8])
    b = int(((A[:,0]==1)&(A[:,1]==0)).sum()); c = int(((A[:,0]==0)&(A[:,1]==1)).sum()); lo, hi = boot(PC, sf)
    print(f"\n{label}: {n} pairs")
    print(f"  strike-free   {int(A[:,0].sum())}/{n} vs {int(A[:,1].sum())}/{n}   diff {sf(A):+.1f} pp  95% CI [{lo:+.1f}, {hi:+.1f}]  "
          f"discordant {b}/{c}  McNemar p={mcnemar_exact(b,c):.4f}")
    lo2, hi2 = boot(PC, rc); print(f"  reached       {int(A[:,2].sum())}/{n} vs {int(A[:,3].sum())}/{n}   diff {rc(A):+.1f} pp  95% CI [{lo2:+.1f}, {hi2:+.1f}]")
    lo3, hi3 = boot(PC, det); T = int(A[:,6].sum())
    print(f"  detection e2e {int(A[:,4].sum())}/{T} vs {int(A[:,5].sum())}/{T}   diff {det(A):+.1f} pp  95% CI [{lo3:+.1f}, {hi3:+.1f}]")
    lo4, hi4 = boot(PC, clr); print(f"  min-clearance {A[:,7].mean():.2f} vs {A[:,8].mean():.2f} mm   diff {clr(A):+.2f} mm  95% CI [{lo4:+.2f}, {hi4:+.2f}]")
    return lo, hi

def pairs(R, strict=False):
    """-> {band: [1-row array per complete pair]}; strict rescoring replaces the harness strike flag."""
    D = {}
    for r in R:
        r = dict(r)
        if strict: r['strike_free'] = int(r['completed'] and r['min_clear_mm'] >= TIP_REACH_MM)
        D.setdefault(r['cell'], {}).setdefault(r['course_id'], {})[r['variant']] = r
    return {b: [np.array([pair_rows(v['FULL'], v['NO_WINGWASH'])]) for _, v in sorted(D.get(b, {}).items()) if len(v) == 2] for b in BANDS}

if __name__ == "__main__":
    R = [json.loads(l) for l in open("outputs/e57_runs.jsonl")]
    P = pairs(R); complete = all(len(P[b]) == N_PER_BAND for b in NARROW)
    print(f"{len(R)} flights; complete pairs per band: " + ", ".join(f"{b}: {len(P[b])}" for b in BANDS))
    narrow = [x for b in NARROW for x in P[b]]
    if narrow:
        lo, hi = line("PRIMARY (H1), bands 52-58 and 46-52 mm pooled, FULL vs NO_WINGWASH", narrow)
        print(f"  VERDICT: {verdict(lo, hi)}" if complete else "  (run incomplete: no verdict)")
        S = pairs(R, strict=True); line(f"  sensitivity, strike radius {TIP_REACH_MM} mm and no goal-zone exception", [x for b in NARROW for x in S[b]])
    print("\nSECONDARY (descriptive, no multiplicity correction)")
    for b in BANDS:
        if P[b]: line(f"  band {b} mm", P[b])
    allp = [x for b in BANDS for x in P[b]]
    if allp: line("  all bands pooled", allp)
