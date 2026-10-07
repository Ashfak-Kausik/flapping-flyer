"""
e54_analysis — per axis/level paired FULL - NO_WINGWASH on outputs/degraded_feeler.csv:
strike-free completion and END-TO-END per-opening detection (hits on reached flights / all openings
in the 40 courses), each with a 95% cluster-bootstrap CI over courses (10,000 resamples, percentile,
default_rng(20261005)) and an exact two-sided McNemar test on strike-free. The undegraded reference
row is the seed-index-1 slice of outputs/stats_pass1.csv (same courses, same noise seeds).

Run:  python experiments/e54_analysis.py [degraded_feeler.csv]
"""
import sys, csv, numpy as np
from e53_analysis import mcnemar_exact, B, RNG_SEED

def boot(M, stat):
    rng = np.random.default_rng(RNG_SEED); n = len(M); out = np.array([stat(M[rng.integers(0, n, n)]) for _ in range(B)])
    return np.percentile(out, 2.5), np.percentile(out, 97.5)

def cell(label, P):
    """P: {course_id: {variant: row}} -> printed markdown row."""
    ks = sorted(P); g = lambda k, v, c: float(P[k][v][c])
    # columns: sfF sfN hitF hitN nop reachF reachN clrF clrN
    M = np.array([[g(k,'FULL','strike_free'), g(k,'NO_WINGWASH','strike_free'), g(k,'FULL','n_detected'), g(k,'NO_WINGWASH','n_detected'),
                   g(k,'FULL','n_openings'), g(k,'FULL','completed'), g(k,'NO_WINGWASH','completed'),
                   g(k,'FULL','min_clear_mm'), g(k,'NO_WINGWASH','min_clear_mm')] for k in ks])
    n = len(M); sf = lambda X: 100*np.mean(X[:,0]-X[:,1]); det = lambda X: 100*(X[:,2].sum()-X[:,3].sum())/X[:,4].sum()
    b = int(((M[:,0]==1)&(M[:,1]==0)).sum()); c = int(((M[:,0]==0)&(M[:,1]==1)).sum())
    lo, hi = boot(M, sf); dlo, dhi = boot(M, det); T = int(M[:,4].sum())
    print(f"| {label} | {int(M[:,0].sum())}/{n} | {int(M[:,1].sum())}/{n} | {sf(M):+.1f} pp | [{lo:+.1f}, {hi:+.1f}] | {b}/{c}, p={mcnemar_exact(b,c):.3f} | "
          f"{int(M[:,2].sum())}/{T} | {int(M[:,3].sum())}/{T} | {det(M):+.1f} pp | [{dlo:+.1f}, {dhi:+.1f}] | "
          f"{int(M[:,5].sum())} / {int(M[:,6].sum())} | {M[:,7].mean():.1f} / {M[:,8].mean():.1f} |")
    return sf(M)

if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else "outputs/degraded_feeler.csv"
    base = {}
    for r in csv.DictReader(open("outputs/stats_pass1.csv")):
        if r['seed_idx'] == '1': base.setdefault(int(r['course_id']), {})[r['variant']] = r
    D = {}
    for r in csv.DictReader(open(path)):
        D.setdefault((r['axis'], float(r['level'])), {}).setdefault(int(r['course_id']), {})[r['variant']] = r
    print("| axis / level | FULL strike-free | NO_WW strike-free | diff | 95% CI (pp) | discordant F/N, McNemar | FULL det e2e | NO_WW det e2e | diff | 95% CI (pp) | reached F / N | mean min-clr mm F / N |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|")
    cell("idealized (pass1 seed 1)", base)
    for axis in ("range", "dropout", "noise"):
        lv = sorted((l for a, l in D if a == axis), reverse=(axis == "range"))
        for l in lv: cell(f"{axis} {l:g}", D[(axis, l)])
