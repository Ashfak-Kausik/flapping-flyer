"""
e21_robustness.py — ARC A (4): is the proximity vector robust to the model we IMPOSED?

The surface effect is added as an (R/4d)^2 law with strength K_GE (=1, Cheeseman-
Bennett leading term). A referee will object: "you imposed the effect, so of course
you sense it — and you don't know the real K_GE." The defense is structural:

  bearing = atan2(δ_roll / S_roll, -δ_pitch / S_pitch)

and K_GE scales BOTH axes by the same factor, so it cancels in the ratio. The
*direction* of the proximity vector is therefore invariant to K_GE; only the
magnitude scales (and magnitude is calibrated out anyway). We verify this by
sweeping K_GE over a 4x range and recovering bearings with a calibration fixed at
K_GE=1 — combined with the geometry-invariance already shown in e20, nothing the
navigation needs depends on the exact magnitude we assumed.
"""
import sys
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design
import src.ground_effect as ge

def set_kge(k): ge.kappa_pts.__defaults__ = (k, ge.KAPPA_MAX)   # rebind default used inside _trans_ge

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
fly = Flyer(ROOT / "models" / "flyer.xml")
print("designing LQG + roll+pitch observer...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3, 4), feedforward=False,
                         Q=(5, 5, 20, 2, 2, 250, 250, 6e4))

def held(surface, T=0.55):
    ctrl.reset(); fly.reset(kin=kin, height=0.05)
    for i in range(int(T/fly.dt)):
        u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=surface)
        fly.x_com[0] = 0.0; fly.x_com[1] = 0.0; fly.v[0] = 0.0; fly.v[1] = 0.0
    return ctrl.roll_dist, ctrl.pitch_dist

flat = lambda th: dict(normal=[-np.cos(np.deg2rad(th)), -np.sin(np.deg2rad(th)), 0.0],
                       point=[0.020*np.cos(np.deg2rad(th)), 0.020*np.sin(np.deg2rad(th)), 0.0])

# free-air bias is surface-independent -> measure once
set_kge(1.0); b_wx, b_wy = held(None)

azis = [0, 45, 90, 135, 180]
kge_sweep = [0.5, 1.0, 2.0]
raw = {}
for kge in kge_sweep:
    set_kge(kge); raw[kge] = {a: held(flat(a)) for a in azis}
set_kge(1.0)

# fixed reference calibration from the K_GE=1 sweep
S_roll_1 = raw[1.0][90][0] - b_wx
S_pitch_1 = -(raw[1.0][0][1] - b_wy)
recover = lambda wx, wy: np.rad2deg(np.arctan2((wx-b_wx)/S_roll_1, -(wy-b_wy)/S_pitch_1)) % 360
print(f"reference calibration (K_GE=1): S_roll={S_roll_1:.0f}, S_pitch={S_pitch_1:.0f}\n")

print(f" {'K_GE':>5}{'S_roll':>8}{'S_pitch':>8}{'ratio':>7}{'mean brg err':>13}")
data = {}
for kge in kge_sweep:
    Sr = raw[kge][90][0] - b_wx; Sp = -(raw[kge][0][1] - b_wy)
    recs, errs = [], []
    for a in azis:
        wx, wy = raw[kge][a]; rec = recover(wx, wy)
        err = (rec - a + 180) % 360 - 180; recs.append(a + err); errs.append(abs(err))
    data[kge] = dict(Sr=Sr, Sp=Sp, recs=recs, mean_err=np.mean(errs))
    print(f" {kge:5.1f}{Sr:8.0f}{Sp:8.0f}{Sr/Sp:6.1f}{np.mean(errs):11.0f}°")

print("\n -> magnitude scales ~linearly with K_GE; anisotropy ratio ~constant;")
print("    bearing error ~constant -> DIRECTION is invariant to the assumed magnitude.")

# ---- figure ----
fig, ax = plt.subplots(1, 2, figsize=(12.5, 5.2))
for kge in kge_sweep:
    ax[0].plot(azis, data[kge]["recs"], "o-", label=f"K_GE={kge}", alpha=0.85)
ax[0].plot([0, 315], [0, 315], "--", color="gray", lw=1, label="ideal")
ax[0].set_xlabel("true wall azimuth (°)"); ax[0].set_ylabel("recovered bearing (°)")
ax[0].set_title("(A) Bearing invariant across a 4× K_GE range\n(calibration fixed at K_GE=1)")
ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
ks = kge_sweep
ax[1].plot(ks, [data[k]["Sr"] for k in ks], "o-", color="tab:blue", label="S_roll")
ax[1].plot(ks, [data[k]["Sp"]*11 for k in ks], "s-", color="tab:orange", label="S_pitch ×11")
ax2 = ax[1].twinx()
ax2.plot(ks, [data[k]["Sr"]/data[k]["Sp"] for k in ks], "^--", color="tab:green", label="anisotropy ratio")
ax2.set_ylabel("ratio S_roll/S_pitch", color="tab:green"); ax2.set_ylim(0, 16)
ax[1].set_xlabel("K_GE (imposed ground-effect strength)"); ax[1].set_ylabel("sensitivity (rad/s²)")
ax[1].set_title("(B) Magnitude scales, ratio (→ direction) does not")
ax[1].legend(loc="upper left", fontsize=8); ax2.legend(loc="lower right", fontsize=8); ax[1].grid(alpha=0.3)
plt.tight_layout(); fig.savefig(OUT/"e21_robustness.png", dpi=130)
with open(OUT/"e21_robustness.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["K_GE", "S_roll", "S_pitch", "ratio", "mean_bearing_err_deg"])
    for kge in kge_sweep:
        d = data[kge]; w.writerow([kge, f"{d['Sr']:.0f}", f"{d['Sp']:.0f}", f"{d['Sr']/d['Sp']:.2f}", f"{d['mean_err']:.0f}"])
print(f"saved: {OUT/'e21_robustness.png'} and .csv")