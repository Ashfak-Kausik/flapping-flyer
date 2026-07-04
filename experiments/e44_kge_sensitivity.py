"""
e44 — PHASE 3 honesty: K_GE SENSITIVITY. Every wing-wash figure (and the phase-1
rate-robustness) rests on one number — K_GE, the ground-effect coefficient ground_effect.py
flags as the wind-tunnel value (=1.0, Cheeseman-Bennett). This study re-runs the load-bearing
results at K_GE in {0.5, 1.0, 1.5} to show how much the conclusions move if the real wing-wash
is half or 1.5x the modelled strength.

Two parts:
  centering_scaling()  — how the centering SIGNAL scales with K_GE (fast; static aero).
  rate_robustness()    — the key claim under test: the wing-wash PASSIVELY stabilises the flyer
                         and rescued low control rates (e38). Does 250Hz still complete if the
                         aero is weaker/stronger? Re-runs the e38 course sweep per K_GE.
                         (full SCALE=3 course per run -> run on a fast machine, not the sandbox.)

K_GE is varied by rebinding ground_effect.kappa_pts's default coefficient.

Run:  python experiments/e44_kge_sensitivity.py
"""
import sys; sys.path.insert(0,'.'); import numpy as np
import src.ground_effect as ge
import experiments.e43_two_wall as e43
import experiments.e38_control_rate as e38

def set_kge(k): ge.kappa_pts.__defaults__=(float(k), ge.KAPPA_MAX)   # rebind default coefficient

def centering_scaling(kges=(0.5,1.0,1.5), offsets_mm=(4,8,12)):
    print("CENTERING SIGNAL vs K_GE (two-wall net roll accel, rad/s^2)")
    print(" K_GE | " + " | ".join(f"off {o}mm" for o in offsets_mm))
    base={}
    for k in kges:
        set_kge(k); vals=[]
        for o in offsets_mm:
            d1=e43.Wd/2-o/1e3; d2=e43.Wd/2+o/1e3
            vals.append(e43.measure_corridor(d1,d2)['roll_acc'])
        if k==1.0: base=dict(zip(offsets_mm,vals))
        print(f" {k:4.2f} | " + " | ".join(f"{v:8.1f}" for v in vals))
    set_kge(1.0)
    print("  (signal is ~linear in K_GE below saturation -> centering AUTHORITY scales with the unknown)")

def rate_robustness(kges=(0.5,1.0,1.5), rates=(1000,500,250), seeds=(1,2,3)):
    print("\nRATE-ROBUSTNESS vs K_GE  (e38 course; does the wing-wash still rescue low rates?)")
    print(" K_GE | rate(Hz) | reach | crash | clearance(mm, completed)")
    for k in kges:
        set_kge(k)
        for rate in rates:
            reach=0; crash=0; cl=[]
            for sd in seeds:
                r=e38.run_one(rate_hz=rate, level=0.0, seed=sd)
                if r['reached']: reach+=1; cl.append(r['min_clear_mm'])
                if r['crashed']: crash+=1
            mc=f"{np.mean(cl):.1f}" if cl else "  -  "
            print(f" {k:4.2f} | {rate:7d}  |  {reach}/{len(seeds)}  |  {crash}/{len(seeds)}  | {mc}")
    set_kge(1.0)
    print("  -> if 250Hz completion drops at K_GE=0.5, the rate-robustness claim is conditional on")
    print("     the wing-wash being at least ~modelled strength: states the sim-to-real dependence honestly.")

if __name__=="__main__":
    centering_scaling()
    rate_robustness()