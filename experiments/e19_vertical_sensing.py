"""
e19_vertical_sensing.py — ARC A (2): de-confounding the vertical axis -> 3D vector.

The floor adds lift, but the vertical disturbance observer failed before because it
read the disturbance from vz, which carries the huge vertical wingbeat ripple. The
fix mirrors the roll axis: DISTRUST vz (Rk high) and let the observer read the
vertical disturbance from the ripple-free HEIGHT channel; subtract the constant
hover-trim bias. The vertical estimate then calibrates to the true ground-effect
lift, giving a vertical rangefinder — the third axis of the proximity vector.

(A) Calibration: floor_dist (ε̂) vs the independently measured ground-effect lift
    ε_true = ΔFz/M, evaluated at the height the flyer actually settles at.
(B) Distance estimation: fit ε̂(height) on training heights, invert, test on
    held-out heights -> estimated floor distance vs true.

Honest caveat: the vertical axis senses a surface ABOVE or BELOW with the same sign
(both increase lift in this model), so it gives vertical proximity MAGNITUDE, not
up/down disambiguation — that needs another cue (e.g. known altitude). Roll+pitch
(e18) + this vertical axis now give the full 3D proximity vector's three components,
each calibrated with its own sensitivity.
"""
import sys
from pathlib import Path
import csv
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design
from src import aero, ground_effect as ge

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
fly = Flyer(ROOT / "models" / "flyer.xml"); M = fly.M; g = 9.81
print("designing LQG + 3D disturbance observer (vz distrusted)...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(2, 3, 4), feedforward=False,
                         Rk=(100, 100, 100, 1e-4, 1e-4, 1e-7), Q=(5, 5, 20, 2, 2, 250, 250, 6e4))
FLOOR = dict(axis=2, sign=1, pos=0.0)


def eps_true(h_m, ncyc=10, navg=6):
    m, d = fly.model, fly.data; per = 2*np.pi/kin.W; N = int(ncyc*per/fly.dt); t0 = (ncyc-navg)*per
    vg = {s: None for s in "RL"}; vf = {s: None for s in "RL"}; acc = []
    for i in range(N):
        t = i*fly.dt; d.qpos[fly._qf:fly._qf+3] = [0, 0, h_m]; d.qpos[fly._qf+3:fly._qf+7] = [1, 0, 0, 0]; d.qvel[fly._vf:fly._vf+6] = 0
        fly._prescribe_wings(kin, t); mujoco.mj_forward(m, d); Fg = Ff = 0.0
        for s, bid in fly.wings.items():
            fg, _, vg[s], _ = ge.wing_aero_ge(m, d, bid, fly.strips[s], vg[s], fly.dt, FLOOR, fly.R)
            ff, _, vf[s], _ = aero.wing_aero(m, d, bid, fly.strips[s], vf[s], fly.dt)
            Fg += fg[2]; Ff += ff[2]
        if t >= t0: acc.append(Fg - Ff)
    return np.mean(acc) / M


def held(h_ref_mm, floor=True, T=1.6):
    ctrl.reset(); ctrl.h_ref = h_ref_mm/1e3; fly.reset(kin=kin, height=h_ref_mm/1e3)
    for i in range(int(T/fly.dt)):
        u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=FLOOR if floor else None)
    return ctrl.floor_dist, fly.sense()['height']*1e3


bias, _ = held(50, floor=False)
print(f"free-air vertical bias = {bias:+.3f} m/s² (subtracted)\n")

train = [8, 11, 15, 21, 30, 42]
print("(A) calibration at the SETTLED height:")
print(f" {'h_ref':>6}{'settled':>9}{'eps_hat':>9}{'eps_true':>9}{'ratio':>7}")
eh_tr, et_tr, hs_tr = [], [], []
for hr in train:
    eh, hs = held(hr); eh -= bias; et = eps_true(hs/1e3)
    eh_tr.append(eh); et_tr.append(et); hs_tr.append(hs)
    print(f" {hr:4d}mm{hs:8.1f}mm{eh:8.3f}{et:8.3f}{eh/et:6.2f}")
slope = np.polyfit(et_tr, eh_tr, 1)[0]
print(f" -> ε̂ = {slope:.2f}·ε_true  (calibrated vertical sensor)")

# fit ε̂(distance):  1/sqrt(ε) = m·h + b   -> invert to estimate floor distance
inv = 1/np.sqrt(np.array(eh_tr)); m_, b_ = np.polyfit(np.array(hs_tr), inv, 1)
A = 1/m_**2; h0 = b_/m_
est = lambda e: float(np.sqrt(A/e) - h0)

test = [9, 13, 18, 25, 36]
print("\n(B) distance estimation on HELD-OUT heights:")
print(f" {'h_ref':>6}{'true(settled)':>14}{'est dist':>10}{'err':>7}")
tt, ee = [], []
for hr in test:
    eh, hs = held(hr); eh -= bias; e = est(eh)
    tt.append(hs); ee.append(e)
    print(f" {hr:4d}mm{hs:12.1f}mm{e:9.1f}mm{e-hs:+6.1f}mm")
rmse = float(np.sqrt(np.mean((np.array(ee)-np.array(tt))**2)))
print(f" -> floor-distance RMSE on held-out heights = {rmse:.2f} mm")

fig, ax = plt.subplots(1, 2, figsize=(12, 5))
ax[0].plot(et_tr, eh_tr, "o", color="tab:orange", ms=9); lim = [0, max(et_tr)*1.05]
ax[0].plot(lim, [slope*x for x in lim], "-", color="tab:orange", label=f"fit slope {slope:.2f}")
ax[0].plot(lim, lim, "--", color="gray", lw=1, label="1:1")
ax[0].set_xlabel("true ground-effect ε = ΔFz/M  (m/s²)"); ax[0].set_ylabel("vertical estimate ε̂ (m/s²)")
ax[0].set_title("(A) Vertical axis de-confounded & calibrated"); ax[0].legend(); ax[0].grid(alpha=0.3)
ax[1].plot([0, max(tt)+3], [0, max(tt)+3], "--", color="gray", lw=1, label="1:1")
ax[1].plot(tt, ee, "o", color="tab:red", ms=10, label=f"held-out (RMSE {rmse:.1f} mm)")
ax[1].plot(hs_tr, [est(e) for e in eh_tr], "x", color="tab:orange", label="training fit")
ax[1].set_xlabel("true floor distance (mm)"); ax[1].set_ylabel("estimated distance (mm)")
ax[1].set_title("(B) Floor rangefinder"); ax[1].legend(); ax[1].grid(alpha=0.3)
plt.tight_layout(); fig.savefig(OUT/"e19_vertical_sensing.png", dpi=130)
with open(OUT/"e19_vertical_sensing.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["set", "h_ref_mm", "settled_mm", "eps_hat", "eps_true_or_est"])
    for hr, hs, eh, et in zip(train, hs_tr, eh_tr, et_tr): w.writerow(["train", hr, f"{hs:.1f}", f"{eh:.3f}", f"{et:.3f}"])
    for hr, hs, e in zip(test, tt, ee): w.writerow(["test", hr, f"{hs:.1f}", "", f"{e:.1f}"])
print(f"saved: {OUT/'e19_vertical_sensing.png'} and .csv")