"""
e50 — RANDOMIZED-LAYOUT BATCH: turns "works on a course" into "works over N random courses."

Samples random Manhattan containment passages (random leg count/turn-directions/lengths/widths and
random breaches), runs the e48 mission on each under noise, and reports DISTRIBUTIONS: completion
rate, detection rate, localization error (mean +/- std), false-positive rate.

Two envelopes:
  safe  — widths >=64mm, breaches mid-leg (spaced from turns), lengths >=15mm  -> clean inspection
          statistics given good navigation (the headline table).
  hard  — widths 56-64mm, breaches allowed near turns -> a deliberately-hard subset that captures
          where navigation degrades (honest limits).

Courses are rejection-sampled for self-intersection (non-adjacent legs kept apart). Full batch is
compute-heavy -> run on a fast machine; a couple of courses validate the harness here.

Run:  python experiments/e50_batch.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import experiments.e47_realistic_course as e47
import experiments.e48_mission as e48

LEFT={'E':'N','N':'W','W':'S','S':'E'}; RIGHT={'E':'S','S':'W','W':'N','N':'E'}

def sample_course(rng, hard=False):
    n=int(rng.choice([3,4])) if hard else int(rng.choice([2,3]))   # safe headline = <=3 legs (reliable); hard = 3-4 legs
    dirs=['E']
    for _ in range(n-1):
        dirs.append((LEFT if rng.random()<0.5 else RIGHT)[dirs[-1]])
    wlo,whi=(0.056,0.064) if hard else (0.064,0.076)
    W=rng.uniform(wlo,whi,n); L=rng.uniform(0.16,0.26,n)
    legs=[(dirs[i],float(L[i]),float(W[i])) for i in range(n)]
    breaches=[]
    for i in range(n):
        for _ in range(int(rng.choice([0,1,2],p=[0.3,0.5,0.2]))):
            side=int(rng.choice([+1,-1])); ln=float(rng.uniform(0.015,0.090))
            fc=rng.uniform(0.30,0.70) if hard else rng.uniform(0.40,0.62)
            hf=(ln/L[i])/2; f0,f1=fc-hf,fc+hf
            lo,hi=(0.12,0.90) if hard else (0.25,0.82)
            if f0>lo and f1<hi: breaches.append((side,i,float(f0),float(f1)))
    return legs, breaches

def _seg_dist(a,b,c,d):
    """min distance between 2D segments ab and cd (coarse: sample points)."""
    pts=np.linspace(0,1,8); best=1e9
    for t in pts:
        p=a+(b-a)*t
        for u in pts: best=min(best,np.linalg.norm(p-(c+(d-c)*u)))
    return best

def valid(geo):
    P=geo['P']; wmax=max(geo['W'])
    for i in range(len(P)-1):
        for j in range(i+2,len(P)-1):
            if _seg_dist(P[i],P[i+1],P[j],P[j+1])<wmax*1.3: return False
    return True

def score_quiet(geo, breaches, detected, tol=0.06):
    truth=e48._truth_spans(geo,breaches); hits=0; locs=[]
    used=set()
    for (s,li,p0,p1) in truth:
        cen=(p0+p1)/2
        cand=[(k,d) for k,d in enumerate(detected) if k not in used
              and (d[0]=='L')==(s==+1)
              and np.linalg.norm((np.array(d[1])+np.array(d[2]))/2-cen)<tol]
        if cand:
            k,d=min(cand,key=lambda kd:np.linalg.norm((np.array(kd[1][1])+np.array(kd[1][2]))/2-cen))
            used.add(k); hits+=1
            locs.append(np.linalg.norm((np.array(d[1])+np.array(d[2]))/2-cen)*1e3)
    fp=len(detected)-len(used)
    return hits, len(truth), fp, locs

def batch(N=30, hard=False, level=1.0, seed0=0, verbose=False):
    rng=np.random.default_rng(1234+ (1 if hard else 0))
    comp=0; runs=0; all_hits=all_tot=all_fp=0; all_locs=[]; n_with_breach=0
    tag="HARD" if hard else "SAFE"
    for k in range(N):
        for _ in range(40):
            legs,breaches=sample_course(rng,hard); geo=e47.build_geometry(legs,breaches)
            if valid(geo) and len(breaches)>0: break
        else: continue
        r=e48.run(geo, level=level, seed=seed0+k, fuse=True); runs+=1
        reached = r['reached']=='finish'
        comp+=reached
        if reached:
            h,t,fp,locs=score_quiet(geo,breaches,r['detected'])
            all_hits+=h; all_tot+=t; all_fp+=fp; all_locs+=locs; n_with_breach+=1
            if verbose: print(f"  [{tag} {k}] legs={len(legs)} breaches={t} reached=Y det={h}/{t} fp={fp}")
        elif verbose: print(f"  [{tag} {k}] legs={len(legs)} breaches={len(breaches)} reached=N ({r['reached']})")
    locs=np.array(all_locs)
    print(f"\n=== {tag} batch: {runs} courses, level {level} ===")
    print(f"  completion rate     : {comp}/{runs} = {100*comp/max(runs,1):.0f}%")
    print(f"  detection rate      : {all_hits}/{all_tot} = {100*all_hits/max(all_tot,1):.0f}%  (completed courses)")
    print(f"  localization error  : {locs.mean():.1f} +/- {locs.std():.1f} mm (median {np.median(locs):.1f}, max {locs.max():.1f})" if len(locs) else "  localization: n/a")
    print(f"  false positives     : {all_fp} over {comp} completed courses ({all_fp/max(comp,1):.2f}/course)")
    return dict(runs=runs,comp=comp,hits=all_hits,tot=all_tot,fp=all_fp,locs=all_locs)

if __name__=="__main__":
    print("##### SAFE envelope (headline statistics) #####")
    batch(N=40, hard=False, level=1.0, verbose=True)
    print("\n##### HARD subset (where navigation degrades) #####")
    batch(N=20, hard=True, level=1.0, verbose=True)