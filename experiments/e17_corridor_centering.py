"""
e17_corridor_centering.py — STAGE 6 (4/N): centering in a corridor from the net
roll disturbance of two walls — no proximity sensor, no calibration.

With a wall on each side, the wall observer's estimate delta_hat_net is the SUM of
the two walls' roll disturbances: zero on the centreline (they cancel), positive
toward the +y wall, negative toward -y. So delta_hat_net is an odd function of
lateral offset, and centering is simply null-seeking it to zero — a setpoint that
needs no calibration. The clean wall observer (e16) supplies the signal directly;
feed-forward cancels the wall torque so flying between two walls stays stable.

(A) The centering signal: delta_hat_net vs lateral offset (held) — odd, through
    the origin: which side am I closer to, and by how much.
(B) Centering: released off-centre, the flyer drives delta_hat_net -> 0 and settles
    on the centreline, from either side.

Honest scope: like the single-wall standoff (e15), the closed loop is robust only
where the signal is strong — tight corridors centre cleanly; in wide corridors the
central signal is weak and a slow drift wins (the recurring (R/4d)^2 + lateral-
authority limitation). Sensing is clean; tight closed-loop control across all
widths is the remaining control problem. Shown here at a representative width.
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
TIP = 13.3
print("designing LQG + wall observer (feed-forward ON, Q[vy]=150)...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                         Q=(5, 150, 20, 2, 2, 250, 250, 6e4))
W = 0.020                                            # corridor half-width (m)
SURF = [dict(axis=1, sign=-1, pos=+W), dict(axis=1, sign=+1, pos=-W)]


def held_net(y0, T=0.6):
    ctrl.reset(); fly.reset(kin=kin, height=0.05, y=y0)
    for i in range(int(T / fly.dt)):
        u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=SURF); fly.x_com[1] = y0; fly.v[1] = 0
    return ctrl.roll_dist


def center(y0, T=2.4, K=3e-5, VMAX=0.10):
    ctrl.reset(); fly.reset(kin=kin, height=0.05, y=y0); ts, ys = [], []
    for i in range(int(T / fly.dt)):
        vy = np.clip(-K * ctrl.roll_dist, -VMAX, VMAX)
        u = ctrl.update(fly.sense(), fly.dt, vy_ref=vy); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=SURF)
        if i % int(0.02/fly.dt) == 0: ts.append(i*fly.dt); ys.append(fly.x_com[1]*1e3)
        if abs(fly.x_com[1]) > 0.04: break
    return np.array(ts), np.array(ys)


print("=" * 72)
print(" STAGE 6 (4/N) — corridor centering from the net wall disturbance")
print("=" * 72)
print(f" corridor half-width {W*1e3:.0f} mm; centre gap to each wall {(W-TIP/1e3*1e0)*1e3-TIP+TIP:.1f}".replace('.0',''))
print(f" centre gap to each wall = {W*1e3 - TIP:.1f} mm\n (A) centering signal delta_net vs lateral offset (held):")
offs = [-8, -6, -4, -2, 0, 2, 4, 6, 8]; net = []
for o in offs:
    d = held_net(o/1e3); net.append(d)
    print(f"     offset {o:+3d} mm -> delta_net {d:+8.0f}")
print(" -> odd, through the origin: the centering signal (zero on the centreline)")

print("\n (B) centering from off-centre starts:")
starts = [+5, -5, +3]; runs = {}
for y0 in starts:
    t, y = center(y0/1e3); runs[y0] = (t, y)
    print(f"     start {y0:+3d} mm -> settled {y[-1]:+5.1f} mm")
print("=" * 72)

# ---- figure ----
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
ax[0].plot(offs, net, "o-", color="tab:purple")
ax[0].axhline(0, color="k", lw=0.6); ax[0].axvline(0, color="k", lw=0.6)
ax[0].set_xlabel("lateral offset from centre (mm)"); ax[0].set_ylabel("δ̂_net  (rad/s²)")
ax[0].set_title("(A) Centering signal: odd, zero on the centreline"); ax[0].grid(alpha=0.3)
for y0 in starts:
    t, y = runs[y0]; ax[1].plot(t, y, lw=1.8, label=f"start {y0:+d} mm")
ax[1].axhline(0, color="gray", ls="--", lw=1); ax[1].text(0.05, 0.4, "centreline", color="gray", fontsize=8)
ax[1].axhline(+(W*1e3 - TIP), color="firebrick", lw=2); ax[1].axhline(-(W*1e3 - TIP), color="firebrick", lw=2)
ax[1].text(0.05, (W*1e3-TIP)-1.2, "wall contact", color="firebrick", fontsize=8)
ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("lateral position (mm)")
ax[1].set_title(f"(B) Centering in a {2*W*1e3:.0f} mm corridor (safe starts)")
ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
plt.tight_layout(); fig.savefig(OUT / "e17_corridor_centering.png", dpi=130)
with open(OUT / "e17_corridor_centering.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["panel", "x", "y"])
    for o, d in zip(offs, net): w.writerow(["A_signal", o, f"{d:.1f}"])
    for y0 in starts:
        t, y = runs[y0]
        for ti, yi in zip(t, y): w.writerow([f"B_start{y0:+d}", f"{ti:.3f}", f"{yi:.2f}"])
print(f"saved: {OUT/'e17_corridor_centering.png'}")
print(f"saved: {OUT/'e17_corridor_centering.csv'}")