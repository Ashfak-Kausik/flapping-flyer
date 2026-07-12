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
    maxper = 2 if hard else 1                          # realistic density: <=1 opening per leg in the safe envelope
    for i in range(n):
        cnt = int(rng.choice(list(range(maxper+1)), p=([0.4,0.6] if maxper==1 else [0.3,0.5,0.2])))
        for _ in range(cnt):
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
    comp=0; runs=0; all_hits=all_tot=all_fp=0; all_locs=[]; n_with_breach=0; courses_found=0
    tag="HARD" if hard else "SAFE"; percourse=[]; skipped=0
    for k in range(N):
        for _ in range(40):
            legs,breaches=sample_course(rng,hard); geo=e47.build_geometry(legs,breaches)
            if valid(geo) and len(breaches)>0: break
        else: skipped+=1; continue
        r=e48.run(geo, level=level, seed=seed0+k, fuse=True); runs+=1
        reached = r['reached']=='finish'
        comp+=reached
        row=dict(course_id=k, ensemble=tag, n_legs=len(legs), widths_mm=[round(w*1e3,2) for (_,_,w) in legs],
                  n_openings=len(breaches), completed=reached, n_detected=0, n_false_positives=0, loc_errors_mm=[])
        if reached:
            h,t,fp,locs=score_quiet(geo,breaches,r['detected'])
            all_hits+=h; all_tot+=t; all_fp+=fp; all_locs+=locs; n_with_breach+=1
            if h>=1: courses_found+=1
            row.update(n_detected=h, n_false_positives=fp, loc_errors_mm=locs)
            if verbose: print(f"  [{tag} {k}] legs={len(legs)} breaches={t} reached=Y det={h}/{t} fp={fp}")
        elif verbose: print(f"  [{tag} {k}] legs={len(legs)} breaches={len(breaches)} reached=N ({r['reached']})")
        percourse.append(row)
    locs=np.array(all_locs)
    print(f"\n=== {tag} batch: {runs} courses, level {level} ===" + (f"  ({skipped} course slots skipped: no valid geometry in 40 tries)" if skipped else ""))
    print(f"  completion rate     : {comp}/{runs} = {100*comp/max(runs,1):.0f}%")
    print(f"  detection rate      : {all_hits}/{all_tot} = {100*all_hits/max(all_tot,1):.0f}%  (per-breach, completed courses)")
    print(f"  per-course found >=1: {courses_found}/{n_with_breach} = {100*courses_found/max(n_with_breach,1):.0f}%  (course flagged >=1 opening)")
    print(f"  localization error  : {locs.mean():.1f} +/- {locs.std():.1f} mm (median {np.median(locs):.1f}, max {locs.max():.1f})  n={len(locs)}" if len(locs) else "  localization: n/a")
    print(f"  false positives     : {all_fp} over {comp} completed courses ({all_fp/max(comp,1):.2f}/course)")
    return dict(runs=runs,comp=comp,hits=all_hits,tot=all_tot,fp=all_fp,locs=all_locs,
                n_with_breach=n_with_breach,courses_found=courses_found,percourse=percourse,skipped=skipped)

def save_csv(results, path="outputs/e50_batch.csv"):
    """Persist one row per course (both ensembles) plus aggregate rows per ensemble,
    reproducible from a file (same pattern as e37/e38/e41's CSV exports)."""
    import csv
    with open(path,"w",newline="") as fh:
        w=csv.writer(fh)
        w.writerow(["course_id","ensemble","n_legs","widths_mm","n_openings","completed",
                     "n_detected","n_false_positives","loc_errors_mm"])
        for tag,res in results.items():
            for row in res['percourse']:
                w.writerow([row['course_id'], row['ensemble'], row['n_legs'],
                            ";".join(str(x) for x in row['widths_mm']), row['n_openings'],
                            int(bool(row['completed'])), row['n_detected'], row['n_false_positives'],
                            ";".join(f"{x:.3f}" for x in row['loc_errors_mm'])])
        w.writerow([])
        w.writerow(["ensemble","runs","completed","comp_rate","hits","tot_openings","detect_rate",
                     "courses_found","n_with_breach","per_course_rate","loc_mean_mm","loc_std_mm",
                     "n_loc","fp_total","fp_per_course","skipped"])
        for tag,res in results.items():
            locs=np.array(res['locs'])
            w.writerow([tag, res['runs'], res['comp'], res['comp']/max(res['runs'],1),
                        res['hits'], res['tot'], res['hits']/max(res['tot'],1),
                        res['courses_found'], res['n_with_breach'],
                        res['courses_found']/max(res['n_with_breach'],1),
                        locs.mean() if len(locs) else "", locs.std() if len(locs) else "",
                        len(locs), res['fp'], res['fp']/max(res['comp'],1), res['skipped']])
    print("saved ->",path)

if __name__=="__main__":
    print("##### SAFE envelope (headline statistics) #####")
    safe=batch(N=40, hard=False, level=1.0, verbose=True)
    print("\n##### HARD subset (where navigation degrades) #####")
    hard=batch(N=20, hard=True, level=1.0, verbose=True)
    save_csv(dict(SAFE=safe, HARD=hard))