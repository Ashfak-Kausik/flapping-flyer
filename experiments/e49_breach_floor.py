"""
e49 — BREACH-LENGTH FLOOR: how SHORT an opening can the flyer still flag, and how well does it
size it? (the measured version of the "~35mm" number, which was an extrapolation.)

Mirrors e46's width-floor method but sweeps BREACH LENGTH at fixed corridor width (64mm) and
noise: one opening on a straight corridor, length swept 15..80mm, N seeds. Reports, vs true
length: detection rate, localization error, and size estimate (raw + calibrated).

Analytic floor (checked against the sweep): the 90-deg side feeler reads OPEN only while the
nose is within the gap; near each edge the perpendicular ray still clips the wall corner, so the
"open" span is shorter than the true gap by ~a fixed edge margin (independent of length). Below
a length ~= that margin the feeler never reads a clean OPEN and detection fails. So we expect a
roughly constant size underestimate and a hard detection floor a little above that margin.

Full sweep is compute-heavy -> run on a fast machine; a couple of points validate here.
Run:  python experiments/e49_breach_floor.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import experiments.e45_breach_inspection as e45

Wd=0.064   # corridor width for this study

def run_one(length_m, level=1.0, seed=0, xc=0.13, LTOT=0.26):
    """Place a single left-wall opening of given length centered at xc; fly; return detection."""
    e45.BREACHES=[('L', xc-length_m/2.0, xc+length_m/2.0)]; e45.LTOT=LTOT
    r=e45.run(level=level, seed=seed, fuse=True, tmax=LTOT/e45.e.Vc*3.0+10)
    det=[d for d in r['detected'] if d[0]=='L']
    if det:
        d=min(det, key=lambda d:abs((d[1]+d[2])/2.0 - xc))
        dc=(d[1]+d[2])/2.0; dl=d[2]-d[1]
        return dict(detected=True, loc_err_mm=abs(dc-xc)*1e3, size_raw_mm=dl*1e3, reached=r['reached'])
    return dict(detected=False, loc_err_mm=np.nan, size_raw_mm=np.nan, reached=r['reached'])

def sweep(lengths_mm=(10,12,15,18,20,25,30,35,40,50,60,80), level=1.0, seeds=(1,2,3)):
    print(f"BREACH-LENGTH FLOOR (corridor {Wd*1e3:.0f}mm, level {level}, {len(seeds)} seeds)")
    print(" true(mm) | detect rate | loc err(mm) | size raw(mm) | size err raw")
    rows=[]
    for L in lengths_mm:
        dets=[]; locs=[]; sizes=[]
        for sd in seeds:
            r=run_one(L/1e3, level=level, seed=sd)
            dets.append(r['detected'])
            if r['detected']: locs.append(r['loc_err_mm']); sizes.append(r['size_raw_mm'])
        dr=np.mean(dets); loc=np.mean(locs) if locs else np.nan; sz=np.mean(sizes) if sizes else np.nan
        rows.append((L, dr, loc, sz))
        print(f"  {L:5.0f}   |   {dr*100:3.0f}%     |   {loc:6.1f}    |   {sz:6.1f}     |   {sz-L:+6.1f}")
    return rows

def calibrate(rows):
    """linear size model: raw_detected = a*true + b  ->  true_est = (raw - b)/a. Fit over
    reliably-detected points only."""
    pts=[(r[0],r[3]) for r in rows if r[1]>=0.99 and not np.isnan(r[3])]
    if len(pts)<2: print("not enough points to calibrate"); return None
    T=np.array([p[0] for p in pts]); D=np.array([p[1] for p in pts])
    a,b=np.polyfit(T,D,1); resid=D-(a*T+b); rms=float(np.sqrt(np.mean(resid**2)))
    print(f"\nsize calibration (raw = a*true + b): a={a:.3f}, b={b:.1f}mm ; invert: true_est=(raw-{b:.1f})/{a:.3f}")
    print(f"  -> residual RMS {rms:.1f}mm ; (flat +16mm offset only fits the 50-80mm range, over-adds below)")
    return a,b

def make_figure(rows, cal=None, path="outputs/e49_breach_floor.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    L=np.array([r[0] for r in rows]); dr=np.array([r[1] for r in rows]); sz=np.array([r[3] for r in rows])
    fig,ax=plt.subplots(1,2,figsize=(11,4.2))
    ax[0].plot(L,dr*100,'o-',color="#0d9488"); ax[0].set_xlabel("true breach length (mm)")
    ax[0].set_ylabel("detection rate (%)"); ax[0].set_title("detection vs breach length"); ax[0].grid(alpha=0.3); ax[0].set_ylim(-5,105)
    ax[1].plot(L,sz,'o-',color="#2563eb",label="raw detected"); ax[1].plot(L,L,'k--',lw=1,label="ideal (y=x)")
    if cal is not None:
        a,b=cal; ax[1].plot(L,(sz-b)/a,'^--',color="#f59e0b",label=f"calibrated (a={a:.2f},b={b:.0f})")
    ax[1].set_xlabel("true breach length (mm)"); ax[1].set_ylabel("estimated length (mm)")
    ax[1].set_title("size estimate vs true"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)

if __name__=="__main__":
    rows=sweep()
    cal=calibrate(rows)
    make_figure(rows, cal)
    ok=[r for r in rows if r[1]>=0.99]
    if ok: print(f"\nDETECTION FLOOR (100% detected): {min(r[0] for r in ok):.0f}mm at corridor {Wd*1e3:.0f}mm, 1x noise")
    partial=[r for r in rows if 0<r[1]<0.99]
    if partial: print(f"partial-detection band: {min(r[0] for r in partial):.0f}-{max(r[0] for r in partial):.0f}mm")