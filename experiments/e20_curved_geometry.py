"""
e20_curved_geometry.py — ARC A (3): does the proximity vector survive real geometry?

A1 used flat infinite walls. Real clutter is curved, convex, tilted, finite. The
thesis says the proximity vector points at the DOMINANT nearby surface regardless,
because the body-integrated aero disturbance is dominated by the nearest geometry
(~1/d^2). We test the horizontal compass (roll+pitch) on:

  (i)  CONCAVE curved wall — flyer inside a vertical cylinder (a round tunnel),
       displaced toward different bearings. Does it point at the nearest wall?
  (ii) CONVEX curved obstacle — a vertical pillar / rubble chunk at various bearings.
       Does it point at the chunk?
  (iii) TILTED flat wall — a wall leaning 35 deg from vertical at various bearings.
       Does the horizontal bearing still come out right?

Per-axis sensitivities S_roll, S_pitch are calibrated ONCE on a flat wall (as in
e18); everything else is held out. Vertical axis is excluded here to avoid the known
floor/ceiling sign ambiguity — this is a horizontal-plane test.
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

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
fly = Flyer(ROOT / "models" / "flyer.xml")
print("designing LQG + roll+pitch observer...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3, 4), feedforward=False,
                         Q=(5, 5, 20, 2, 2, 250, 250, 6e4))


def held(surface, px=0.0, py=0.0, T=0.7):
    ctrl.reset(); fly.reset(kin=kin, height=0.05)
    for i in range(int(T/fly.dt)):
        u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=surface)
        fly.x_com[0] = px; fly.x_com[1] = py; fly.v[0] = 0.0; fly.v[1] = 0.0
    return ctrl.roll_dist, ctrl.pitch_dist


# --- one-time calibration on a flat wall (bearing 90 deg and 0 deg) ---
b_wx, b_wy = held(None)
flat = lambda th: dict(normal=[-np.cos(np.deg2rad(th)), -np.sin(np.deg2rad(th)), 0.0],
                       point=[0.020*np.cos(np.deg2rad(th)), 0.020*np.sin(np.deg2rad(th)), 0.0])
S_roll = held(flat(90))[0] - b_wx
S_pitch = -(held(flat(0))[1] - b_wy)
recover = lambda wx, wy: np.rad2deg(np.arctan2((wx-b_wx)/S_roll, -(wy-b_wy)/S_pitch)) % 360
print(f"calibrated on flat wall: S_roll={S_roll:.0f}, S_pitch={S_pitch:.0f}  ({S_roll/S_pitch:.0f}:1)\n")

azis = list(range(0, 360, 45))
results = {}

# (i) concave: inside a vertical cylinder, displaced toward each bearing
Rc, off = 0.028, 0.010
rows = []
for a in azis:
    th = np.deg2rad(a)
    wx, wy = held(dict(type="cyl_in", center=[0, 0], radius=Rc), px=off*np.cos(th), py=off*np.sin(th))
    rec = recover(wx, wy); err = (rec - a + 180) % 360 - 180; rows.append((a, rec, err))
results["concave tunnel (R=28mm)"] = rows

# (ii) convex: a pillar at each bearing, nearest surface ~7mm from wingtip reach
Rp, Dp = 0.010, 0.030
rows = []
for a in azis:
    th = np.deg2rad(a)
    wx, wy = held(dict(type="cyl_out", center=[Dp*np.cos(th), Dp*np.sin(th)], radius=Rp))
    rec = recover(wx, wy); err = (rec - a + 180) % 360 - 180; rows.append((a, rec, err))
results["convex pillar (R=10mm)"] = rows

# (iii) tilted flat wall, leaning 35 deg from vertical, at each bearing
tilt = np.deg2rad(35)
rows = []
for a in azis:
    th = np.deg2rad(a)
    n = [-np.cos(th)*np.cos(tilt), -np.sin(th)*np.cos(tilt), np.sin(tilt)]   # tilt normal up-and-out
    nrm = np.array(n)/np.linalg.norm(n)
    wx, wy = held(dict(normal=nrm.tolist(), point=[0.020*np.cos(th), 0.020*np.sin(th), 0.0]))
    rec = recover(wx, wy); err = (rec - a + 180) % 360 - 180; rows.append((a, rec, err))
results["tilted wall (35deg)"] = rows

print(f" {'geometry':<26}{'mean err':>9}{'max err':>9}")
for name, rows in results.items():
    errs = [abs(r[2]) for r in rows]
    print(f" {name:<26}{np.mean(errs):7.0f}°{max(errs):7.0f}°")

# ---- figure: one compass per geometry ----
fig, axes = plt.subplots(1, 3, figsize=(15, 5.2))
th_c = np.linspace(0, 2*np.pi, 100)
for ax, (name, rows) in zip(axes, results.items()):
    ax.plot(np.cos(th_c), np.sin(th_c), color="lightgray", lw=1)
    for a, rec, err in rows:
        ax.plot(np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a)), "s", color="firebrick", ms=10)
        r = np.deg2rad(rec)
        ax.annotate("", xy=(0.82*np.cos(r), 0.82*np.sin(r)), xytext=(0, 0),
                    arrowprops=dict(arrowstyle="-|>", color="tab:blue", lw=2))
    errs = [abs(r[2]) for r in rows]
    ax.set_aspect("equal"); ax.set_xlim(-1.3, 1.3); ax.set_ylim(-1.3, 1.3); ax.axis("off")
    ax.set_title(f"{name}  (mean err {np.mean(errs):.0f}°)", fontsize=11, pad=8)
axes[0].plot([], [], "s", color="firebrick", label="nearest-surface bearing")
axes[0].plot([], [], color="tab:blue", lw=2, label="sensed bearing")
axes[0].legend(loc="lower center", bbox_to_anchor=(0.5, -0.18), fontsize=8, ncol=2)
plt.tight_layout(); fig.savefig(OUT/"e20_curved_geometry.png", dpi=130, bbox_inches="tight")
with open(OUT/"e20_curved_geometry.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["geometry", "true_bearing_deg", "recovered_deg", "err_deg"])
    for name, rows in results.items():
        for a, rec, err in rows: w.writerow([name, a, f"{rec:.0f}", f"{err:.0f}"])
print(f"\nsaved: {OUT/'e20_curved_geometry.png'} and .csv")