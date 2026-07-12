"""
e43 — PHASE 3 capstone: the TWO-WALL DIFFERENTIAL — the signal navigation actually uses.

e41/e42 characterized a SINGLE wall. In a corridor the centering signal is the IMBALANCE
between the two walls: with the flyer offset by delta from center in a width-W corridor, the
near wall is at d1=W/2-delta and the far at d2=W/2+delta, and the net roll disturbance is
their difference. It is ZERO at center (the null the centering servos to) and grows with
offset. This experiment measures that net signal directly (static aero, both walls present),
locates the null, gets the centering sensitivity dT/d(offset) at center, and checks whether
the two single-wall curves simply SUPERPOSE (they should: ground_effect sums kappa over
surfaces).

Run:  python experiments/e43_two_wall.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
from src.flyer import Flyer
from src.controller import design
import experiments.e41_wingwash_sensor as e41

CRUISE=0.05; Wd=0.064; WINGREACH=0.0133

def measure_corridor(d1, d2, thrust=0.5, h=CRUISE):
    """Net cycle-averaged roll torque with a wall at +d1 (left) and a wall at -d2 (right)."""
    fly=Flyer("models/flyer.xml"); ctrl,kin,info=design(fly,control_dt=1e-3)
    kin.set_control(thrust=thrust,roll=0.0,pitch=0.0,yaw=0.0); fly.reset(kin=kin,height=h)
    walls=[dict(normal=[0.0,-1.0,0.0],point=[0.0, d1,0.0]),    # left wall at +d1, normal toward flyer (-y)
           dict(normal=[0.0, 1.0,0.0],point=[0.0,-d2,0.0])]    # right wall at -d2, normal toward flyer (+y)
    Tx=e41.cycle_avg_roll(fly,kin,walls)
    fly.reset(kin=kin,height=h); Tx0=e41.cycle_avg_roll(fly,kin,None)
    Ixx=float(fly.I[0,0]); dT=Tx-Tx0
    return dict(T_nNm=dT*1e9, roll_acc=dT/Ixx)

def sweep_offset(W=Wd, n=11, thrust=0.5):
    dmax=W/2-WINGREACH                                          # furthest offset before wingtip strike
    offs=np.linspace(-dmax, dmax, n)
    print(f"two-wall differential, corridor W={W*1e3:.0f}mm (half {W/2*1e3:.0f}mm); offset +/-{dmax*1e3:.1f}mm")
    print(" offset(mm) | d_near | d_far | net roll torque (nN·m) | net roll accel (rad/s^2)")
    rows=[]
    for off in offs:
        d1=W/2-off; d2=W/2+off                                 # off>0 -> nearer the LEFT(+) wall
        r=measure_corridor(d1,d2,thrust=thrust)
        rows.append((off*1e3, d1*1e3, d2*1e3, r['T_nNm'], r['roll_acc']))
        print(f"  {off*1e3:+7.1f}   | {d1*1e3:5.1f}  | {d2*1e3:5.1f} | {r['T_nNm']:9.2f}            | {r['roll_acc']:9.1f}")
    return rows

def save_offset_csv(rows, k, path="outputs/e43_offset_sweep.csv"):
    import csv
    with open(path,"w",newline="") as fh:
        w=csv.writer(fh)
        w.writerow(["offset_mm","d_near_mm","d_far_mm","T_nNm","roll_acc_rad_s2"])
        for row in rows: w.writerow(row)
        w.writerow([]); w.writerow(["near_center_slope_rad_s2_per_mm"]); w.writerow([k])
    print("saved ->",path)

def save_superposition_csv(W=Wd, offs_mm=(0,4,8,12), path="outputs/e43_superposition.csv"):
    import csv
    rows=[]
    for off in offs_mm:
        d1=W/2-off/1e3; d2=W/2+off/1e3
        tw=measure_corridor(d1,d2)['roll_acc']
        s1=e41.measure(d1)['roll_acc']; s2=e41.measure(d2)['roll_acc']
        sd=s1-s2
        rows.append((off, tw, sd, tw/sd if sd else float('nan')))
    with open(path,"w",newline="") as fh:
        w=csv.writer(fh)
        w.writerow(["offset_mm","two_wall_rad_s2","single_diff_rad_s2","ratio"])
        for row in rows: w.writerow(row)
    print("saved ->",path)
    return rows

def superposition_check(W=Wd, offs_mm=(0,4,8,12)):
    """Does the two-wall net equal single-wall(d1) - single-wall(d2)? (additivity test)"""
    print("\nsuperposition check: two-wall net  vs  single(d_near) - single(d_far)")
    print(" offset(mm) | two-wall (rad/s^2) | single-diff (rad/s^2) | ratio")
    for off in offs_mm:
        d1=W/2-off/1e3; d2=W/2+off/1e3
        tw=measure_corridor(d1,d2)['roll_acc']
        s1=e41.measure(d1)['roll_acc']; s2=e41.measure(d2)['roll_acc']
        sd=s1-s2
        print(f"  {off:+7.1f}   | {tw:9.1f}          | {sd:9.1f}             | {tw/sd if sd else float('nan'):.3f}")

def make_figure(rows, path="outputs/e43_two_wall.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    off=np.array([r[0] for r in rows]); ra=np.array([r[4] for r in rows])
    m=np.abs(off)<=np.max(off)/2                     # near-center slope (exclude near-wall 1/d^4 blowup)
    k=np.polyfit(off[m],ra[m],1)[0]
    fig,ax=plt.subplots(figsize=(7,4.5))
    ax.plot(off,ra,'o-',color="#0d9488"); ax.axhline(0,color='k',lw=0.6); ax.axvline(0,color='k',lw=0.6)
    ax.set_xlabel("lateral offset from corridor center (mm)"); ax.set_ylabel("net roll accel (rad/s²)")
    ax.set_title(f"two-wall differential (centering signal) — null at center, near-center slope {k:.0f} (rad/s²)/mm")
    ax.grid(alpha=0.3); fig.tight_layout(); fig.savefig(path,dpi=120); print("saved ->",path)
    print(f"centering sensitivity at center: {k:.0f} (rad/s^2) per mm offset")
    return k

if __name__=="__main__":
    rows=sweep_offset()
    k=make_figure(rows)
    save_offset_csv(rows, k)
    superposition_check()
    save_superposition_csv()