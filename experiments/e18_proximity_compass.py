"""
e18_proximity_compass.py — ARC A (1): the 2D proximity compass.

Stage 6 sensed a wall on ONE axis (roll) because we pre-placed the walls left/right.
Real clutter is at arbitrary bearings. Here we put a single vertical wall at eight
azimuths around the hovering flyer and ask: do the roll AND pitch disturbance
estimates together recover the *direction* to the wall?

Mechanism: a surface enhances the lift of the wing strips nearest it; the resulting
torque is about the axis perpendicular to the surface bearing. A wall to the side
(+y) -> roll torque; a wall ahead (+x) -> pitch torque; in between -> a mix. So the
multi-axis disturbance observer (dist_states=(3,4) = wx,wy) yields a 2-vector whose
direction is the bearing to the wall — a proximity COMPASS, read from the
controller's own residuals, no proximity sensor.
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
print("designing LQG + roll+pitch disturbance observer...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3, 4), feedforward=False,
                         Q=(5, 5, 20, 2, 2, 250, 250, 6e4))
D = 0.020                                          # wall distance from flyer centre (m)


def wall_at(theta_deg):
    th = np.deg2rad(theta_deg)
    return dict(normal=[-np.cos(th), -np.sin(th), 0.0],          # points toward flyer
                point=[D*np.cos(th), D*np.sin(th), 0.0])


def held(surface, T=0.7):
    """Pin x,y position (attitude free) and read the roll & pitch disturbance."""
    ctrl.reset(); fly.reset(kin=kin, height=0.05)
    for i in range(int(T / fly.dt)):
        u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=surface)
        fly.x_com[0] = 0.0; fly.x_com[1] = 0.0; fly.v[0] = 0.0; fly.v[1] = 0.0
    return ctrl.roll_dist, ctrl.pitch_dist


# free-air bias (no wall) to subtract
b_wx, b_wy = held(None)
print(f"free-air bias: roll {b_wx:+.1f}, pitch {b_wy:+.1f} rad/s²  (subtracted)")

azis = list(range(0, 360, 45))
print(f"\n wall at distance {D*1e3:.0f} mm, swept around the flyer:")
raw = []
for a in azis:
    wx, wy = held(wall_at(a)); raw.append((a, wx - b_wx, wy - b_wy))

# per-axis sensitivity from the cardinal directions (one-time calibration)
S_roll  = [wx for a, wx, wy in raw if a == 90][0]      # +y wall -> +roll
S_pitch = [-wy for a, wx, wy in raw if a == 0][0]      # +x wall -> -pitch (so make +)
print(f" planform anisotropy: S_roll={S_roll:.0f}, S_pitch={S_pitch:.0f} rad/s²  ->  {S_roll/S_pitch:.1f} : 1")
print(f" (lateral sensing ~{S_roll/S_pitch:.0f}x stronger than fore/aft — wings are long-span, short-chord)\n")

print(f" {'azimuth':>8}{'δ_roll':>9}{'δ_pitch':>9}{'recovered':>11}{'err':>7}")
rows = []
for a, wx, wy in raw:
    rec = np.rad2deg(np.arctan2(wx / S_roll, -wy / S_pitch)) % 360     # anisotropy-calibrated bearing
    err = (rec - a + 180) % 360 - 180
    rows.append((a, wx, wy, rec, err))
    print(f" {a:6d}° {wx:+8.0f}{wy:+8.0f}{rec:9.0f}°{err:+6.0f}°")
errs = [abs(r[4]) for r in rows]
print(f"\n -> calibrated bearing error: mean {np.mean(errs):.0f}°, max {max(errs):.0f}°  (cardinals exact; diagonals worst)")

# ---- figure: Cartesian compass + bearing recovery ----
fig, ax = plt.subplots(1, 2, figsize=(12, 5.6))
th = np.linspace(0, 2*np.pi, 100)
ax[0].plot(np.cos(th), np.sin(th), color="lightgray", lw=1)
for a, wx, wy, rec, err in rows:
    ax[0].plot(np.cos(np.deg2rad(a)), np.sin(np.deg2rad(a)), "s", color="firebrick", ms=11)
    r = np.deg2rad(rec)
    ax[0].annotate("", xy=(0.85*np.cos(r), 0.85*np.sin(r)), xytext=(0, 0),
                   arrowprops=dict(arrowstyle="-|>", color="tab:blue", lw=2.2))
ax[0].plot([], [], "s", color="firebrick", label="true wall position")
ax[0].plot([], [], color="tab:blue", lw=2.2, label="sensed bearing")
ax[0].set_aspect("equal"); ax[0].set_xlim(-1.3, 1.3); ax[0].set_ylim(-1.3, 1.3)
ax[0].set_title("Proximity compass: sensed bearing vs wall"); ax[0].legend(loc="lower right", fontsize=8); ax[0].axis("off")
ax[1].plot([r[0] for r in rows], [r[3] for r in rows], "o-", color="tab:blue", label="recovered")
ax[1].plot([0, 315], [0, 315], "--", color="gray", lw=1, label="ideal")
ax[1].set_xlabel("true wall azimuth (°)"); ax[1].set_ylabel("recovered bearing (°)")
ax[1].set_title(f"Bearing recovery (anisotropy {S_roll/S_pitch:.0f}:1, calibrated)")
ax[1].legend(); ax[1].grid(alpha=0.3)
plt.tight_layout(); fig.savefig(OUT / "e18_proximity_compass.png", dpi=130)
with open(OUT / "e18_proximity_compass.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["azimuth_deg", "delta_roll", "delta_pitch", "recovered_deg", "err_deg"])
    for r in rows: w.writerow([r[0], f"{r[1]:.0f}", f"{r[2]:.0f}", f"{r[3]:.0f}", f"{r[4]:.0f}"])
print(f"saved: {OUT/'e18_proximity_compass.png'}  and .csv")