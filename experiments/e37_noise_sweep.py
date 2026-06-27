"""
e37 — SENSOR-NOISE SWEEP: how well does the reactive navigation survive realistic sensor
noise? Adds Gaussian noise (src/noise.py) to the antenna ranges and the IMU (which feeds the
wing-wash roll_dist signal through the observer), scaled by `level`, and measures over the
course:  reached the finish?   minimum wall clearance (crash detector)   did it crash?
Runs several seeds per level and reports mean +/- std, then plots clearance-vs-noise.

TIP: set SCALE=1.0 in e36_reactive_course.py before sweeping -- the tight-clearance corners
are identical at any scale, so 1x gives the same curve far faster than 3x.

Run:  python experiments/e37_noise_sweep.py        (saves outputs/e37_noise_sweep.png)
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
LEVELS=[0, 0.5, 1.0, 1.5, 2.0, 3.0]   # x realistic noise
SEEDS=[1, 2, 3]                        # repeats per level (noise is stochastic)
CONTROL_RATE=1000                      # Hz: realistic loop rate (single operating point, matches e38)

def run_one(level, seed):
    nm=NoiseModel(level, seed)
    fly=Flyer(e.build_model())
    ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),feedforward=True,Q=(150,150,20,2,2,250,250,6e4),control_dt=1.0/CONTROL_RATE)
    ctrl.K[:,0]=0.0; ctrl.K[:,1]=0.0; ant=Antenna(fly); clr=Clearance(fly, n_rays=24)
    ctrl.reset(); fly.reset(kin=kin,height=e.CRUISE); ctrl.h_ref=e.CRUISE
    N=max(1, round((1.0/CONTROL_RATE)/fly.dt)); dt_c=N*fly.dt          # zero-order hold between control updates
    t=0.0; stepi=0; I_s=0.0; pref=0.0; nose_f=nose_prev=I_y=0.0; rd_f=0.0; state="CRUISE"; tdir=0; turn0=None
    minc=1e3; crashed=False; reached=False; tmax=e.run_timeout()
    floor=dict(axis=2,sign=1,pos=0.0); pl=[]
    while t<tmax:
        if stepi % N == 0:                                            # sense + navigate + control @ CONTROL_RATE
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
            kin.set_control(thrust=u[0],roll=u[1]-e.ROLL_FF*uy,pitch=u[2],yaw=uy)
        fly.step(kin,t,surface=(pl+[floor]))                          # physics every sim step; wall+floor aero held
        x,y,z=fly.x_com
        if int(t/0.04)!=int((t-fly.dt)/0.04):
            c=clr.min_clearance(); minc=min(minc,c)
            if c<WINGREACH: crashed=True
        if np.hypot(e.FINISH[0]-x,e.FINISH[1]-y)<e.FIN_ZONE: reached=True; break
        t+=fly.dt; stepi+=1
    return dict(reached=reached, min_clear_mm=minc*1e3, crashed=crashed)

def sweep(levels=LEVELS, seeds=SEEDS, verbose=True):
    rows=[]
    for L in levels:
        cs=[]; reach=0; crash=0
        for sd in seeds:
            r=run_one(L,sd); cs.append(r['min_clear_mm']); reach+=r['reached']; crash+=r['crashed']
            if verbose: print(f"  level {L:>4}  seed {sd}:  reached={'Y' if r['reached'] else 'N'}  minClear={r['min_clear_mm']:.1f}mm  {'CRASH' if r['crashed'] else 'ok'}")
        cs=np.array(cs)
        rows.append(dict(level=L, clear_mean=cs.mean(), clear_std=cs.std(), clear_min=cs.min(),
                         reach_rate=reach/len(seeds), crash_rate=crash/len(seeds)))
        if verbose: print(f"  => level {L}: clearance {cs.mean():.1f}+-{cs.std():.1f}mm  reach {reach}/{len(seeds)}  crash {crash}/{len(seeds)}\n")
    return rows

def make_figure(rows, path="outputs/e37_noise_sweep.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    L=[r['level'] for r in rows]; cm=[r['clear_mean'] for r in rows]; cs=[r['clear_std'] for r in rows]
    rr=[r['reach_rate']*100 for r in rows]; cr=[r['crash_rate']*100 for r in rows]
    fig,(ax1,ax2)=plt.subplots(1,2,figsize=(12,4.6)); fig.patch.set_facecolor("white")
    ax1.errorbar(L,cm,yerr=cs,marker='o',color="#1d6fb8",capsize=4,label="min clearance (mean+-std)")
    ax1.axhline(WINGREACH*1e3,color="#c0392b",ls="--",label=f"wingtip threshold {WINGREACH*1e3:.1f}mm")
    ax1.set_xlabel("sensor-noise level (x realistic)"); ax1.set_ylabel("min wall clearance (mm)")
    ax1.set_title("Clearance vs sensor noise"); ax1.legend(fontsize=9); ax1.grid(alpha=0.25)
    ax2.plot(L,rr,marker='o',color="#2e9e4f",label="reached finish (%)")
    ax2.plot(L,cr,marker='s',color="#c0392b",label="crashed (%)")
    ax2.set_xlabel("sensor-noise level (x realistic)"); ax2.set_ylabel("rate (%)")
    ax2.set_title("Completion & crash rate vs noise"); ax2.set_ylim(-5,105); ax2.legend(fontsize=9); ax2.grid(alpha=0.25)
    fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)

if __name__=="__main__":
    print(f"noise sweep on SCALE={e.SCALE} course ({e.path_length()*1e3:.0f}mm); {len(LEVELS)} levels x {len(SEEDS)} seeds")
    rows=sweep(); 
    print("\nlevel | clearance(mm) | reached | crashed")
    for r in rows: print(f"  {r['level']:>4} | {r['clear_mean']:5.1f} +- {r['clear_std']:4.1f} | {r['reach_rate']*100:3.0f}%    | {r['crash_rate']*100:3.0f}%")
    make_figure(rows)