"""
e16_disturbance_observer.py — STAGE 6 (3/N): a disturbance observer turns the
flow-proximity signal from a contaminated proxy into a calibrated distance sensor.

We augment the LQG Kalman filter with a roll angular-acceleration disturbance
state delta on the wx equation (controller.py, dist_obs=True). Because the filter
already accounts for the known control u, delta_hat captures the WALL torque, not
the manoeuvre roll that corrupted e15's raw u_roll signal.

(A) Calibration: held at a range of standoffs, delta_hat equals the independently
    measured wall torque T_d/Ixx (slope ~1, ratio ~1.0) — a physically calibrated
    estimate, not just a monotone proxy.
(B) Distance estimation: fit the delta_hat(gap) law on a TRAINING set of standoffs
    and invert it; on HELD-OUT test standoffs the estimated gap matches the true
    gap — the flyer reads its distance to the wall from its own dynamics.

Honest scope: this is the clean SENSING upgrade. The reactive loop benefits
(stronger lateral authority Q[vy] for genuine active standoff once the model-
dependent passive repulsion is cancelled); close-range active regulation stays
touchy because delta ~ (R/4d)^2 steepens near contact — a control refinement for
later, separate from the sensing result shown here.
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
fly = Flyer(ROOT / "models" / "flyer.xml")
Ixx = fly.I[0, 0]; R = fly.R; TIP = 13.3
print("designing LQG + disturbance observer...")
ctrl, kin, info = design(fly, dist_obs=True, feedforward=False, Q=(5, 150, 20, 2, 2, 250, 250, 6e4))


def _com():
    c = np.zeros(3)
    for i in range(1, fly.model.nbody):
        c += fly.model.body_mass[i] * fly.data.xipos[i]
    return c / fly.M


def open_loop_dTx(yw, ncyc=10, navg=6):
    surf = dict(axis=1, sign=-1, pos=yw / 1e3); m, d = fly.model, fly.data
    period = 2 * np.pi / kin.W; N = int(ncyc * period / fly.dt); t0 = (ncyc - navg) * period
    vg = {s: None for s in "RL"}; vf = {s: None for s in "RL"}; acc = []
    for i in range(N):
        t = i * fly.dt
        d.qpos[fly._qf:fly._qf+3] = [0, 0, 0.05]; d.qpos[fly._qf+3:fly._qf+7] = [1, 0, 0, 0]; d.qvel[fly._vf:fly._vf+6] = 0
        fly._prescribe_wings(kin, t); mujoco.mj_forward(m, d); com = _com(); Tg = Tf = 0.0
        for s, bid in fly.wings.items():
            fg, tg, vg[s], _ = ge.wing_aero_ge(m, d, bid, fly.strips[s], vg[s], fly.dt, surf, R)
            ff, tf, vf[s], _ = aero.wing_aero(m, d, bid, fly.strips[s], vf[s], fly.dt)
            Tg += tg[0] + np.cross(d.xipos[bid]-com, fg)[0]; Tf += tf[0] + np.cross(d.xipos[bid]-com, ff)[0]
        if t >= t0: acc.append(Tg - Tf)
    return float(np.mean(acc))


def held_dhat(yw, T=0.6):
    ctrl.reset(); fly.reset(kin=kin, height=0.05)
    for i in range(int(T / fly.dt)):
        u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=dict(axis=1, sign=-1, pos=yw/1e3)); fly.x_com[1] = 0; fly.v[1] = 0
    return ctrl.roll_dist


# ---- (A) calibration on training standoffs ----
train_walls = [16, 18, 20, 24, 30, 38]
print("\n(A) calibration: delta_hat vs independently-measured T_d/Ixx")
print(f" {'wall':>6}{'gap':>7}{'delta_hat':>11}{'T_d/Ixx':>10}{'ratio':>7}")
dh_tr, truth_tr, gap_tr = [], [], []
for yw in train_walls:
    de = held_dhat(yw); td = open_loop_dTx(yw) / Ixx; g = yw - TIP
    dh_tr.append(de); truth_tr.append(td); gap_tr.append(g)
    print(f" {yw:4d}mm{g:6.1f}mm{de:10.0f}{td:9.0f}{de/td:6.2f}")
slope = np.polyfit(truth_tr, dh_tr, 1)[0]
print(f" -> delta_hat = {slope:.2f} * (T_d/Ixx): calibrated proximity signal")

# fit delta_hat(gap) law:  1/sqrt(delta) = m*gap + b   (i.e. delta = A/(gap+g0)^2)
inv = 1.0 / np.sqrt(np.array(dh_tr)); m, b = np.polyfit(np.array(gap_tr), inv, 1)
A = 1.0 / m**2; g0 = b / m
def est_gap(delta):
    return float(np.sqrt(A / delta) - g0)

# ---- (B) distance estimation on held-out test standoffs ----
test_walls = [17, 22, 27, 34]
print("\n(B) distance estimation on HELD-OUT standoffs (fit from training only):")
print(f" {'wall':>6}{'true gap':>9}{'est gap':>9}{'error':>8}")
true_g, est_g = [], []
for yw in test_walls:
    de = held_dhat(yw); g = yw - TIP; eg = est_gap(de)
    true_g.append(g); est_g.append(eg)
    print(f" {yw:4d}mm{g:7.1f}mm{eg:7.1f}mm{eg-g:+6.1f}mm")
rmse = float(np.sqrt(np.mean((np.array(est_g) - np.array(true_g))**2)))
print(f" -> distance-estimate RMSE on held-out standoffs = {rmse:.2f} mm")

# ---- figure ----
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
ax[0].plot(truth_tr, dh_tr, "o", color="tab:blue", ms=8)
lim = [0, max(truth_tr)*1.05]; ax[0].plot(lim, [slope*x for x in lim], "-", color="tab:blue", lw=1.5, label=f"fit slope {slope:.2f}")
ax[0].plot(lim, lim, "--", color="gray", lw=1, label="1:1 ideal")
ax[0].set_xlabel("independently measured  T_d / Ixx  (rad/s²)")
ax[0].set_ylabel("observer estimate  δ̂  (rad/s²)")
ax[0].set_title("(A) δ̂ is calibrated to the true wall torque"); ax[0].legend(); ax[0].grid(alpha=0.3)
gg = np.linspace(min(true_g+gap_tr)-1, max(true_g+gap_tr)+1, 50)
ax[1].plot(gap_tr, gap_tr, color="lightgray")  # placeholder for axis scale
ax[1].plot([0, max(true_g)+3], [0, max(true_g)+3], "--", color="gray", lw=1, label="1:1 ideal")
ax[1].plot(true_g, est_g, "o", color="tab:green", ms=9, label=f"held-out tests (RMSE {rmse:.1f} mm)")
ax[1].plot(gap_tr, [est_gap(d) for d in dh_tr], "x", color="tab:blue", ms=7, label="training fit")
ax[1].set_xlabel("true gap: wingtip → wall (mm)"); ax[1].set_ylabel("estimated gap from δ̂ (mm)")
ax[1].set_title("(B) The flyer estimates its distance to the wall"); ax[1].legend(); ax[1].grid(alpha=0.3)
plt.tight_layout(); fig.savefig(OUT / "e16_disturbance_observer.png", dpi=130)
with open(OUT / "e16_disturbance_observer.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["set", "wall_mm", "true_gap_mm", "delta_hat", "Td_over_Ixx_or_estgap"])
    for yw, g, de, td in zip(train_walls, gap_tr, dh_tr, truth_tr): w.writerow(["train", yw, f"{g:.1f}", f"{de:.0f}", f"{td:.0f}"])
    for yw, g, eg in zip(test_walls, true_g, est_g): w.writerow(["test", yw, f"{g:.1f}", "", f"{eg:.2f}"])
print(f"\nsaved: {OUT/'e16_disturbance_observer.png'}")
print(f"saved: {OUT/'e16_disturbance_observer.csv'}")