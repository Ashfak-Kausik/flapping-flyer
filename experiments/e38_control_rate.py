"""
e38 — CONTROL-RATE TEST (sim-to-real): the controller + navigation run at a realistic loop
rate with ZERO-ORDER HOLD (recompute every N sim steps, hold the wing command in between),
while the wings keep flapping at the full sim rate. Slower loops add delay that can
destabilise a controller that looked fine at 10 kHz -- this finds the limit.
Measures over the course: reached finish? min wall clearance (crash detector)? crashed?
Run:  python experiments/e38_control_rate.py     (saves outputs/e38_control_rate.png)
TIP: set SCALE=1.0 in e36 for a faster sweep (same corners, same limit).
"""
import sys; sys.path.insert(0, '.'); import numpy as np
import experiments.e36_reactive_course as e
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna, BIG
from src.safety import Clearance, WINGREACH
from src.noise import NoiseModel
from experiments.e31_corner import bodyframe

SAFE_BUF=0.020; KVEER=0.9
RATES=[10000, 2000, 1000, 500, 250]   # Hz; 10kHz = every sim step (baseline)
SEEDS=[1, 2, 3]

def run_one(rate_hz=10000, level=0.0, seed=0):
    nm=NoiseModel(level, seed)
    fly=Flyer(e.build_model())
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4),control_dt=1.0/rate_hz)
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0; ant=Antenna(fly); clr=Clearance(fly, n_rays=24)
    ctrl.reset(); fly.reset(kin=kin,height=e.CRUISE); ctrl.h_ref=e.CRUISE
    N=max(1, round((1.0/rate_hz)/fly.dt)); dt_c=N*fly.dt   # control timestep (ZOH)
    t=0.0; stepi=0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; state="CRUISE"; tdir=0; turn0=None
    minc=1e3; crashed=False; reached=False; tmax=e.run_timeout()
    floor=dict(axis=2,sign=1,pos=0.0); pl=[]
    while t<tmax:
        if stepi % N == 0:                       # ---- control + navigation loop @ rate_hz ----
            s=nm.sense(fly.sense()); psi=s['yaw']; x,y,z=fly.x_com; spd=np.hypot(s['vx'],s['vy'])
            b=bodyframe(s); u_fwd=b['vx']; v_lat=b['vy']
            nose_f+=((psi-nose_f+np.pi)%(2*np.pi)-np.pi)*dt_c/0.04; nrate=(nose_f-nose_prev)/dt_c; nose_prev=nose_f
            f0,fLp,fRp,f50p,f50m,dL,dR=nm.feel(ant.feel([0,30,-30,50,-50,90,-90])); fwd,fL,fR=f0,fLp,fRp
            rd=ctrl.roll_dist; rd_f+=(rd-rd_f)*dt_c/0.10; pl=e.planes(psi,fly.x_com,dL,dR)
            left_near=min(f50p,dL); right_near=min(f50m,dR); safe=0.0
            if right_near<SAFE_BUF: safe+=KVEER*(SAFE_BUF-right_near)/SAFE_BUF
            if left_near <SAFE_BUF: safe-=KVEER*(SAFE_BUF-left_near)/SAFE_BUF
            slow=0.35 if min(left_near,right_near)<SAFE_BUF else 1.0
            if state=="CRUISE":
                Vcmd=e.Vc*np.clip((fwd-e.STOP)/0.04,0.0,1.0)*slow
                pref+=(np.clip(e.Ksteer*(min(fL,e.FMAX)-min(fR,e.FMAX)),-0.5,0.5)+safe)*dt_c
                roll_ref=np.clip(-e.Kc*rd_f-e.Kd*v_lat,-np.radians(2.5),np.radians(2.5))
                if fwd<e.STOP and min(fL,fR)<e.STOP and spd<0.03: tdir=+1 if fL>=fR else -1; state="TURN"; turn0=nose_f; I_y=0.0
            else:
                Vcmd=0.0; pref+=tdir*e.YAWRATE*dt_c; roll_ref=np.clip(-e.KLAT*v_lat,-np.radians(2),np.radians(2))
                turned=abs(((nose_f-turn0+np.pi)%(2*np.pi)-np.pi))
                if fwd>e.CLEAR and turned>np.radians(20): state="CRUISE"; I_s=0.0; rd_f=0.0
                elif turned>np.radians(175): tdir=-tdir; turn0=nose_f
            er=Vcmd-u_fwd; I_s=np.clip(I_s+er*dt_c,-0.6,0.6); pr=np.clip(0.30*er+1.1*I_s,-np.radians(8),np.radians(8))
            eyaw=((nose_f-pref+np.pi)%(2*np.pi)-np.pi); I_y=np.clip(I_y+eyaw*dt_c,-1.0,1.0)
            uy=np.clip(0.14*eyaw+0.03*nrate+0.10*I_y,-0.3,0.3)
            u=ctrl.update(b,dt_c,pitch_ref=pr,roll_ref=roll_ref,vy_ref=0.0)
            kin.set_control(thrust=u[0],roll=u[1]-e.ROLL_FF*uy,pitch=u[2],yaw=uy)   # held until next update
        fly.step(kin,t,surface=(pl+[floor]))     # ---- physics every sim step; wall+floor aero held (ZOH) ----
        x,y,z=fly.x_com
        if int(t/0.04)!=int((t-fly.dt)/0.04):
            c=clr.min_clearance(); minc=min(minc,c)
            if c<WINGREACH: crashed=True
        if np.hypot(e.FINISH[0]-x,e.FINISH[1]-y)<e.FIN_ZONE: reached=True; break
        if z<0.0 or z>0.2: break                 # diverged
        t+=fly.dt; stepi+=1
    return dict(reached=reached, min_clear_mm=minc*1e3, crashed=crashed)

def sweep(rates=RATES, seeds=SEEDS, level=0.0, verbose=True):
    rows=[]
    for R in rates:
        cs=[]; reach=0; crash=0
        for sd in seeds:
            r=run_one(R,level,sd); cs.append(r['min_clear_mm']); reach+=r['reached']; crash+=r['crashed']
            if verbose: print(f"  {R:>6}Hz seed {sd}: reached={'Y' if r['reached'] else 'N'} minClear={r['min_clear_mm']:.1f}mm {'CRASH' if r['crashed'] else 'ok'}")
        cs=np.array(cs); rows.append(dict(rate=R, clear_mean=cs.mean(), clear_std=cs.std(), reach_rate=reach/len(seeds), crash_rate=crash/len(seeds)))
        if verbose: print(f"  => {R}Hz: {cs.mean():.1f}+-{cs.std():.1f}mm  reach {reach}/{len(seeds)}  crash {crash}/{len(seeds)}\n")
    return rows

def make_figure(rows, path="outputs/e38_control_rate.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    R=[r['rate'] for r in rows]; cm=[r['clear_mean'] for r in rows]; cstd=[r['clear_std'] for r in rows]
    rr=[r['reach_rate']*100 for r in rows]; cr=[r['crash_rate']*100 for r in rows]
    fig,(a1,a2)=plt.subplots(1,2,figsize=(12,4.6)); fig.patch.set_facecolor("white")
    a1.errorbar(R,cm,yerr=cstd,marker='o',color="#1d6fb8",capsize=4); a1.axhline(WINGREACH*1e3,color="#c0392b",ls="--",label=f"wingtip {WINGREACH*1e3:.1f}mm")
    a1.set_xscale("log"); a1.set_xlabel("control rate (Hz, log)"); a1.set_ylabel("min clearance (mm)"); a1.set_title("Clearance vs control rate"); a1.legend(fontsize=9); a1.grid(alpha=0.25,which="both")
    a2.plot(R,rr,marker='o',color="#2e9e4f",label="reached (%)"); a2.plot(R,cr,marker='s',color="#c0392b",label="crashed (%)")
    a2.set_xscale("log"); a2.set_xlabel("control rate (Hz, log)"); a2.set_ylabel("rate (%)"); a2.set_ylim(-5,105); a2.set_title("Completion & crash vs control rate"); a2.legend(fontsize=9); a2.grid(alpha=0.25,which="both")
    fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)

if __name__=="__main__":
    print(f"control-rate sweep on SCALE={e.SCALE} course; rates {RATES} Hz x {len(SEEDS)} seeds (noise level 0)")
    rows=sweep()
    print("\n rate(Hz) | clearance(mm) | reached | crashed")
    for r in rows: print(f"  {r['rate']:>6} | {r['clear_mean']:5.1f} +- {r['clear_std']:4.1f} | {r['reach_rate']*100:3.0f}%    | {r['crash_rate']*100:3.0f}%")
    make_figure(rows)