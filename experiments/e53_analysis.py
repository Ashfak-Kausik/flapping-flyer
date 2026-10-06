"""
e53_analysis — pre-registered analysis of outputs/stats_pass1.csv (and, if present,
outputs/stats_legsweep.csv). See docs/truewall_results.md, section T2.0, for the rule.

  * paired difference FULL - NO_WINGWASH for strike-free completion (primary), per-opening
    detection on jointly-completed trials, and mean min-clearance;
  * 95% CI: cluster bootstrap over courses (each course carries all its paired per-seed outcomes),
    10,000 resamples, percentile interval, default_rng(20261005);
  * exact two-sided McNemar on the pooled paired trials for the binary outcomes;
  * decision against the +/-5 pp margin.

Run:  python experiments/e53_analysis.py [pass1.csv] [legsweep.csv]
"""
import sys, csv, os, math, numpy as np

B = 10000; RNG_SEED = 20261005; MARGIN = 5.0

def load(path):
    P = {}
    for r in csv.DictReader(open(path)):
        P.setdefault((r['cell'], int(r['course_id']), int(r['seed_idx'])), {})[r['variant']] = r
    return P

def mcnemar_exact(b, c):
    """two-sided exact (binomial) McNemar p on discordant counts b, c."""
    n = b + c
    if n == 0: return 1.0
    k = min(b, c); return min(1.0, 2*sum(math.comb(n, i) for i in range(k+1))/2**n)

def boot_ci(per_course, stat):
    """per_course: list (one entry per course) of arrays; stat: f(list of arrays) -> float."""
    rng = np.random.default_rng(RNG_SEED); n = len(per_course); out = np.empty(B)
    for i in range(B):
        out[i] = stat([per_course[j] for j in rng.integers(0, n, n)])
    out = out[~np.isnan(out)]
    return np.percentile(out, 2.5), np.percentile(out, 97.5)

def verdict(lo, hi):
    if -MARGIN <= lo and hi <= MARGIN: return "EQUIVALENT"
    if lo > MARGIN: return "WING-WASH HELPS"
    if hi < -MARGIN: return "WING-WASH HURTS"
    return "UNDERPOWERED"

def analyse_pass1(path):
    P = load(path); courses = sorted({k[1] for k in P})
    print(f"\n===== {path}: {len(P)} paired trials, {len(courses)} courses =====")
    assert all(len(v) == 2 for v in P.values()), "unpaired trials present"
    g = lambda r, c: float(r[c])
    # per-course arrays, one row per seed: [sfF, sfN, compF, compN, clrF, clrN, hitF, hitN, nop, joint]
    PC = []
    for k in courses:
        rows = []
        for key in sorted(x for x in P if x[1] == k):
            F, N = P[key]['FULL'], P[key]['NO_WINGWASH']; joint = int(F['completed']) and int(N['completed'])
            rows.append([g(F,'strike_free'), g(N,'strike_free'), g(F,'completed'), g(N,'completed'),
                         g(F,'min_clear_mm'), g(N,'min_clear_mm'), g(F,'n_detected'), g(N,'n_detected'),
                         g(F,'n_openings'), joint])
        PC.append(np.array(rows))
    A = np.vstack(PC); n = len(A)
    def binary(name, iF, iN, decide=False):
        f, nw = A[:, iF].sum(), A[:, iN].sum(); b = int(((A[:, iF] == 1) & (A[:, iN] == 0)).sum()); c = int(((A[:, iF] == 0) & (A[:, iN] == 1)).sum())
        d = 100*(f-nw)/n
        lo, hi = boot_ci(PC, lambda S: 100*np.mean(np.vstack(S)[:, iF]-np.vstack(S)[:, iN]))
        p = mcnemar_exact(b, c)
        print(f"{name}: FULL {int(f)}/{n} ({100*f/n:.1f}%)  NO_WINGWASH {int(nw)}/{n} ({100*nw/n:.1f}%)")
        print(f"   paired diff {d:+.1f} pp, 95% CI [{lo:+.1f}, {hi:+.1f}] pp;  discordant FULL-only {b}, NO_WINGWASH-only {c}"
              f" (ratio {b/c:.2f})" if c else f"   paired diff {d:+.1f} pp, 95% CI [{lo:+.1f}, {hi:+.1f}] pp;  discordant FULL-only {b}, NO_WINGWASH-only {c}")
        print(f"   exact McNemar p = {p:.4f}" + (f"   ==> {verdict(lo, hi)} (margin +/-{MARGIN:.0f} pp)" if decide else ""))
        pc = np.array([100*np.mean(S[:, iF]-S[:, iN]) for S in PC])
        print(f"   per-course diff: >0 on {int((pc>0).sum())}, <0 on {int((pc<0).sum())}, =0 on {int((pc==0).sum())} courses; sd {pc.std(ddof=1):.1f} pp")
    binary("PRIMARY strike-free completion", 0, 1, decide=True)
    binary("(context) reached goal", 2, 3)
    # detection on jointly-completed trials
    def det(S):
        M = np.vstack(S); J = M[M[:, 9] == 1]
        return 100*(J[:, 6].sum()-J[:, 7].sum())/J[:, 8].sum() if J[:, 8].sum() > 0 else np.nan
    J = A[A[:, 9] == 1]; lo, hi = boot_ci(PC, det)
    print(f"SECONDARY per-opening detection on {len(J)} jointly-completed trials: FULL {int(J[:,6].sum())}/{int(J[:,8].sum())} "
          f"({100*J[:,6].sum()/J[:,8].sum():.1f}%)  NO_WINGWASH {int(J[:,7].sum())}/{int(J[:,8].sum())} ({100*J[:,7].sum()/J[:,8].sum():.1f}%)")
    print(f"   paired diff {det(PC):+.1f} pp, 95% CI [{lo:+.1f}, {hi:+.1f}] pp")
    # clearance
    dclr = lambda S: np.mean(np.vstack(S)[:, 4]-np.vstack(S)[:, 5]); lo, hi = boot_ci(PC, dclr)
    pc = np.array([np.mean(S[:, 4]-S[:, 5]) for S in PC])
    print(f"SECONDARY mean min-clearance (all {n} trials): FULL {A[:,4].mean():.2f} mm  NO_WINGWASH {A[:,5].mean():.2f} mm")
    print(f"   paired diff {dclr(PC):+.2f} mm, 95% CI [{lo:+.2f}, {hi:+.2f}] mm;  Cohen's d_z over courses = {pc.mean()/pc.std(ddof=1):+.2f}")
    print("per-seed strike-free (FULL, NO_WINGWASH):", [(int(sum(S[j,0] for S in PC)), int(sum(S[j,1] for S in PC))) for j in range(PC[0].shape[0])])

def analyse_legsweep(path):
    P = load(path); print(f"\n===== {path}: {len(P)} paired trials =====")
    tb = tc = tf = tn = tt = 0
    for cell in sorted({k[0] for k in P}, key=int):
        K = [k for k in P if k[0] == cell]; F = [int(P[k]['FULL']['strike_free']) for k in K]; N = [int(P[k]['NO_WINGWASH']['strike_free']) for k in K]
        b = sum(f and not n for f, n in zip(F, N)); c = sum(n and not f for f, n in zip(F, N))
        print(f"  legs={cell}: FULL {sum(F)}/{len(K)}  NO_WINGWASH {sum(N)}/{len(K)}  diff {sum(F)-sum(N):+d}  discordant {b}/{c}  McNemar p = {mcnemar_exact(b, c):.3f}")
        tb += b; tc += c; tf += sum(F); tn += sum(N); tt += len(K)
    print(f"  pooled: FULL {tf}/{tt}  NO_WINGWASH {tn}/{tt}  diff {tf-tn:+d}  discordant {tb}/{tc}  McNemar p = {mcnemar_exact(tb, tc):.4f}")

if __name__ == "__main__":
    p1 = sys.argv[1] if len(sys.argv) > 1 else "outputs/stats_pass1.csv"
    ls = sys.argv[2] if len(sys.argv) > 2 else "outputs/stats_legsweep.csv"
    if os.path.exists(p1): analyse_pass1(p1)
    if os.path.exists(ls): analyse_legsweep(ls)
