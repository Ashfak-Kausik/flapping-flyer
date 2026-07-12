"""
e42 — PHASE 3: observer FIDELITY + BANDWIDTH of the wing-wash sensor.

e41 measured the TRUE wall-induced roll disturbance (torque/Ixx) vs distance. But navigation
reads the disturbance OBSERVER's estimate, ctrl.roll_dist. This experiment asks how faithfully
the estimate tracks the truth:
  * FIDELITY: hold the flyer hovering level near a wall at known distance d; once settled, the
    observer's roll_dist should equal the true disturbance from e41. gain(d)=estimate/true.
    (This is the number that bounds the e41 resolution figures: if gain<1, resolution coarsens.)
  * BANDWIDTH/LATENCY: step the wall in from far to distance d and watch roll_dist rise; the
    10-90% rise time is the sensor's latency (the disturbance observer has lag).

Holding level rejects the wall's roll torque, so the flyer stays put laterally (the GE force is
a vertical-lift asymmetry = a roll torque, not a direct side force) and d stays ~constant.

Run:  python experiments/e42_observer_fidelity.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
from src.flyer import Flyer
from src.controller import design
from src.noise import NoiseModel
from experiments.e31_corner import bodyframe
import experiments.e41_wingwash_sensor as e41

CRUISE=0.05

def _run_near_wall(d_wall, secs=3.0, level=0.0, step_in_at=None, d_far=0.20):
    """Level hover with a wall at lateral distance d_wall (on +y). Records (t, roll_dist, y).
    If step_in_at is set, the wall starts far (d_far) and jumps to d_wall at that time."""
    fly=Flyer("models/flyer.xml"); ctrl,kin,info=design(fly,dist_obs=True,dist_states=(3,),
        feedforward=True,Q=(150,150,20,2,2,250,250,6e4),control_dt=1e-3)
    nm=NoiseModel(level,1); fly.reset(kin=kin,height=CRUISE); ctrl.reset(); ctrl.h_ref=CRUISE
    N=10; dt_c=N*fly.dt; t=0.0; si=0; floor=dict(axis=2,sign=1,pos=0.0)
    rec=[]; Iy=0.0
    while t<secs:
        dnow = d_far if (step_in_at is not None and t<step_in_at) else d_wall
        wall=dict(normal=[0.0,-1.0,0.0], point=[0.0,dnow,0.0])
        if si%N==0:
            s=nm.sense(fly.sense()); b=bodyframe(s)
            y=fly.x_com[1]; Iy=float(np.clip(Iy+y*dt_c,-0.02,0.02))     # hold y=0 (fixed distance) w/ integral
            roll_hold=float(np.clip(-45.0*y-9.0*b['vy']-120.0*Iy, -np.radians(9), np.radians(9)))
            u=ctrl.update(b,dt_c,pitch_ref=0.0,roll_ref=roll_hold,vy_ref=0.0,vx_ref=0.0)
            kin.set_control(thrust=u[0],roll=u[1],pitch=u[2],yaw=u[3] if len(u)>3 else 0.0)
            rec.append((t, ctrl.roll_dist, fly.x_com[1], b['vy']))
        fly.step(kin,t,surface=[wall,floor]); t+=fly.dt; si+=1
    return np.array(rec)

def fidelity(d_wall, secs=3.0, level=0.0):
    rec=_run_near_wall(d_wall, secs=secs, level=level)
    settled = rec[rec[:,0]>secs*0.6]
    est = float(np.mean(settled[:,1])); ydrift=float(np.mean(settled[:,2])); vy=float(np.mean(np.abs(settled[:,3])))
    d_eff = d_wall - ydrift
    tru = e41.measure(d_eff)['roll_acc']
    return dict(d_mm=d_wall*1e3, d_eff_mm=d_eff*1e3, est=est, true=tru,
                gain=(est/tru if tru!=0 else np.nan), ydrift_mm=ydrift*1e3, vy_settle=vy)

def bandwidth(d_wall=0.015, secs=2.5, step_in_at=1.0):
    rec=_run_near_wall(d_wall, secs=secs, step_in_at=step_in_at)
    t=rec[:,0]; rd=rec[:,1]
    pre = rd[t<step_in_at]; base=float(np.mean(pre[-20:])) if len(pre)>20 else 0.0
    post = rd[t>=step_in_at]; tp=t[t>=step_in_at]
    final=float(np.mean(rd[t>secs*0.8])); span=final-base
    if abs(span)<1e-6: return dict(rise_ms=np.nan, base=base, final=final)
    lvl=lambda f: base+f*span
    def crossing(frac):
        tgt=lvl(frac); idx=np.where((post-base)/span>=frac)[0]
        return tp[idx[0]] if len(idx) else np.nan
    t10,t90=crossing(0.1),crossing(0.9)
    return dict(rise_ms=(t90-t10)*1e3 if not np.isnan(t90) else np.nan, t10=t10, t90=t90,
                base=base, final=final, step_at=step_in_at)

def save_csv(fid_rows, bw, path="outputs/e42_observer_fidelity.csv"):
    import csv
    with open(path,"w",newline="") as fh:
        w=csv.writer(fh)
        w.writerow(["d_mm","d_eff_mm","est_rad_s2","true_rad_s2","gain","ydrift_mm"])
        for r in fid_rows:
            w.writerow([r['d_mm'],r['d_eff_mm'],r['est'],r['true'],r['gain'],r['ydrift_mm']])
        w.writerow([])
        w.writerow(["bw_base_rad_s2","bw_final_rad_s2","rise_10_90_ms"])
        w.writerow([bw['base'],bw['final'],bw['rise_ms']])
    print("saved ->",path)

if __name__=="__main__":
    print("OBSERVER FIDELITY (level hover near wall, no noise): estimate roll_dist vs e41 true")
    print(" d(mm) | d_eff | est (rad/s^2) | true (rad/s^2) | gain | ydrift(mm)")
    fid_rows=[]
    for dmm in [12,15,18,22,26,30]:
        r=fidelity(dmm/1e3); fid_rows.append(r)
        print(f" {r['d_mm']:5.0f} | {r['d_eff_mm']:5.1f} | {r['est']:10.0f}    | {r['true']:10.0f}     | {r['gain']:.3f} | {r['ydrift_mm']:+.2f}")
    print("\nBANDWIDTH (wall stepped 200mm->15mm):")
    b=bandwidth()
    print(f"  base {b['base']:.0f} -> final {b['final']:.0f} rad/s^2 ; 10-90% rise = {b['rise_ms']:.1f} ms")
    save_csv(fid_rows, b)