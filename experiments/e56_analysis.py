"""
e56_analysis — analysis of the e56 runs exactly as fixed in docs/e56_analysis_plan.md.

  Part A (confirm): paired FULL - NO_WINGWASH at feeler range 0.05 m on the 200 new courses; strike-free
           completion with a 95% cluster-bootstrap CI over courses (10,000 resamples, percentile,
           default_rng(20261005)), exact two-sided McNemar, and the pre-stated verdict. Secondary: reached,
           end-to-end per-opening detection, mean min-clearance.
  Part B (gain): strike-free completion and mean min-clearance for Kc x {0, 1, 2, 4, 8}; 0x and 1x are the
           seed-index 1..3 rows of outputs/stats_pass1.csv. Paired difference vs 0x, bootstrap over courses.
  Check:   the repeated pass1 flight must reproduce the stored row.

Reads outputs/e56_runs.jsonl (so it also works on a partial run; only complete pairs are used).

Run:  python experiments/e56_analysis.py
"""
import sys, csv, json, numpy as np
sys.path.insert(0, 'experiments')
from e53_analysis import mcnemar_exact, B, RNG_SEED

MARGIN = 5.0

def boot(per_course, stat):
    rng = np.random.default_rng(RNG_SEED); n = len(per_course)
    out = np.array([stat(np.vstack([per_course[j] for j in rng.integers(0, n, n)])) for _ in range(B)])
    return np.percentile(out, 2.5), np.percentile(out, 97.5)

def verdict(lo, hi):
    if lo > 0: return "ADVANTAGE CONFIRMED"
    if -MARGIN <= lo and hi <= MARGIN: return "NO MEANINGFUL ADVANTAGE (equivalent within the margin)"
    if hi < 0: return "WING-WASH HURTS"
    return "INCONCLUSIVE"

def pair_rows(F, N):
    g = lambda r, c: float(r[c])
    return [g(F,'strike_free'), g(N,'strike_free'), g(F,'completed'), g(N,'completed'), g(F,'n_detected'), g(N,'n_detected'),
            g(F,'n_openings'), g(F,'min_clear_mm'), g(N,'min_clear_mm')]

def report(label, PC, decide=False):
    """PC: list over courses of arrays (rows = paired trials) with columns of pair_rows."""
    A = np.vstack(PC); n = len(A)
    sf = lambda X: 100*np.mean(X[:,0]-X[:,1]); rc = lambda X: 100*np.mean(X[:,2]-X[:,3])
    det = lambda X: 100*(X[:,4].sum()-X[:,5].sum())/X[:,6].sum(); clr = lambda X: np.mean(X[:,7]-X[:,8])
    b = int(((A[:,0]==1)&(A[:,1]==0)).sum()); c = int(((A[:,0]==0)&(A[:,1]==1)).sum())
    lo, hi = boot(PC, sf)
    print(f"\n{label}: {n} paired trials over {len(PC)} courses")
    print(f"  strike-free   {int(A[:,0].sum())}/{n} vs {int(A[:,1].sum())}/{n}   diff {sf(A):+.1f} pp  95% CI [{lo:+.1f}, {hi:+.1f}]  "
          f"discordant {b}/{c}  McNemar p={mcnemar_exact(b,c):.4f}")
    if decide: print(f"  VERDICT: {verdict(lo, hi)}")
    lo2, hi2 = boot(PC, rc); print(f"  reached       {int(A[:,2].sum())}/{n} vs {int(A[:,3].sum())}/{n}   diff {rc(A):+.1f} pp  95% CI [{lo2:+.1f}, {hi2:+.1f}]")
    lo3, hi3 = boot(PC, det); T = int(A[:,6].sum())
    print(f"  detection e2e {int(A[:,4].sum())}/{T} vs {int(A[:,5].sum())}/{T}   diff {det(A):+.1f} pp  95% CI [{lo3:+.1f}, {hi3:+.1f}]")
    lo4, hi4 = boot(PC, clr); print(f"  min-clearance {A[:,7].mean():.2f} vs {A[:,8].mean():.2f} mm   diff {clr(A):+.2f} mm  95% CI [{lo4:+.2f}, {hi4:+.2f}]")

if __name__ == "__main__":
    R = [json.loads(l) for l in open("outputs/e56_runs.jsonl")]
    P1 = {(int(r['course_id']), int(r['seed_idx']), r['variant']): r for r in csv.DictReader(open("outputs/stats_pass1.csv"))}

    chk = [r for r in R if r['set'] == 'check']
    for r in chk:
        ref = P1[(r['course_id'], r['seed_idx'], 'FULL')]
        same = (r['completed'] == int(ref['completed']) and r['crashed'] == int(ref['crashed'])
                and abs(r['min_clear_mm'] - float(ref['min_clear_mm'])) < 1e-9 and r['n_detected'] == int(ref['n_detected']))
        print(f"harness check, course {r['course_id']}: min-clear {r['min_clear_mm']!r} vs stored {ref['min_clear_mm']}  -> {'IDENTICAL' if same else 'DIFFERS'}")

    C = {}
    for r in R:
        if r['set'] == 'confirm': C.setdefault(r['course_id'], {})[r['variant']] = r
    PC = [np.array([pair_rows(v['FULL'], v['NO_WINGWASH'])]) for k, v in sorted(C.items()) if len(v) == 2]
    if PC: report(f"PART A, feeler range 0.05 m, FULL vs NO_WINGWASH ({len(C)-len(PC)} unpaired courses pending)", PC, decide=(len(PC) == len(C)))

    G = {}
    for r in R:
        if r['set'] == 'gain': G[(r['kc_mult'], r['course_id'], r['seed_idx'])] = r
    mults = sorted({k[0] for k in G})
    if mults:
        print("\nPART B, centring-gain sweep (exploratory); reference 0x = NO_WINGWASH, 1x = FULL from stats_pass1.csv")
        for m in [1.0] + mults:
            PCg = []
            for k in range(40):
                rows = []
                for j in (1, 2, 3):
                    F = P1.get((k, j, 'FULL')) if m == 1.0 else G.get((m, k, j)); N = P1.get((k, j, 'NO_WINGWASH'))
                    if F is not None and N is not None: rows.append(pair_rows(F, N))
                if rows: PCg.append(np.array(rows))
            if PCg: report(f"  Kc x {m:g} vs 0x", PCg)
