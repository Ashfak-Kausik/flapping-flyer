"""
e22_forward_cruise.py — ARC B (1): forward cruise via optic flow, and a bounded
straight-tube fly-through.

Why hover-LQG cannot cruise (diagnosed): forward velocity vx is a HIDDEN state the
Kalman infers from attitude near hover. Once the flyer actually translates, the
hover-linearised estimator stays stuck near zero (measured here: estimate -0.003 m/s
while the true speed was 0.239), so the speed loop regulates a fiction and the flyer
runs away. The cruise aerodynamics themselves are benign — a 1-2 deg pitch trim holds
0.05-0.20 m/s (drag is ~3% of weight).

Fix (this file): a speed-via-pitch CASCADE.
  * sense() gains an optic-flow analog (vx, vy = world horizontal velocity), the cue
    real flying insects use for speed/altitude — hover never needed it, cruise does.
  * an OUTER loop reads vx directly and commands a pitch reference (PI on speed error);
  * the INNER LQG holds that pitch (its strength). vx is removed from the LQR (K[:,0]=0)
    so the broken hover-frame vx estimate never enters control.

Results: forward speed now tracks a command (vs runaway), no lateral drift, altitude
held. A straight round tube is then flown end-to-end by adding lateral roll-centering
on top of cruise.

HONEST LIMITATION: the cruise+centering loop holds for ~4 s / ~0.3 m, then a slow
lateral instability grows (the wing nears the wall, ground effect spikes, cruise and
centering diverge together). So the fly-through is sound for a FINITE tube traversed
before that point; a robust arbitrary-length tube needs a proper lateral POSITION loop
(not velocity-nulling) — the next Arc B control task. See outputs/tube_flythrough.mp4.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design

fly = Flyer(ROOT / "models" / "flyer.xml")
print("designing (speed-via-pitch cascade; vx removed from LQR)...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                         Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
ctrl.K[:, 0] = 0.0                      # outer loop owns forward speed; LQR keeps attitude+vy
Z0 = 0.05


def cruise_step(s, dt, Vc, state, Kp=0.25, Ki=0.9, vy_ref=0.0):
    e = Vc - s['vx']; state['I'] = np.clip(state['I'] + e*dt, -0.5, 0.5)
    pref = np.clip(Kp*e + Ki*state['I'], -np.radians(8), np.radians(8))
    return ctrl.update(s, dt, vy_ref=vy_ref, pitch_ref=pref)


# (A) free-air cruise tracking
print("\n(A) free-air cruise tracking (optic-flow speed feedback):")
print(f" {'commanded':>10}{'settled vx':>12}{'lateral':>10}{'altitude':>10}")
for Vc in (0.05, 0.10, 0.15):
    ctrl.reset(); fly.reset(kin=kin, height=Z0); st = {'I': 0.0}
    for i in range(int(3.5/fly.dt)):
        u = cruise_step(fly.sense(), fly.dt, Vc, st); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i*fly.dt, surface=None)
    s = fly.sense()
    print(f" {Vc:9.2f} {fly.v[0]:11.3f}{fly.v[1]*1e3:+8.1f}mm/s{s['height']*1e3:8.1f}mm")

# (B) straight round-tube fly-through (bounded)
Rc = 0.032; SURF = dict(type="cyl_in_x", center=[0.0, Z0], radius=Rc); WR = 0.0133
ctrl.reset(); fly.reset(kin=kin, height=Z0)
for i in range(int(0.5/fly.dt)):
    u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
    fly.step(kin, i*fly.dt, surface=None)
bx = ctrl.roll_dist
ctrl.reset(); fly.reset(kin=kin, height=Z0); fly.x_com[1] = 0.005; st = {'I': 0.0}
ts, xs, ys, clr = [], [], [], []
for i in range(int(5.0/fly.dt)):
    s = fly.sense()
    vy = np.clip(-3e-5*(ctrl.roll_dist - bx), -0.08, 0.08)
    u = cruise_step(s, fly.dt, 0.07, st, vy_ref=vy); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
    fly.step(kin, i*fly.dt, surface=SURF)
    if i % int(0.05/fly.dt) == 0:
        rr = np.hypot(fly.x_com[1], fly.x_com[2]-Z0)
        ts.append(i*fly.dt); xs.append(fly.x_com[0]*1e3); ys.append(fly.x_com[1]*1e3); clr.append((Rc-rr-WR)*1e3)
crash = next((t for t, c in zip(ts, clr) if c < 0), None)
print(f"\n(B) straight tube R={Rc*1e3:.0f}mm, cruise 0.07 m/s, start 5mm off-centre:")
print(f" clean traversal to x={max(x for x,c in zip(xs,clr) if c>0):.0f}mm; wall contact at t={crash:.1f}s"
      if crash else " no contact in window")

fig, ax = plt.subplots(1, 2, figsize=(12, 4.5))
ax[0].plot(ts, xs, color="tab:blue"); ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("forward distance (mm)")
ax[0].set_title("(A) Forward progress (cruise)"); ax[0].grid(alpha=0.3)
if crash: ax[0].axvline(crash, color="firebrick", ls="--", lw=1, label="wall contact"); ax[0].legend(fontsize=8)
ax[1].plot(ts, ys, color="tab:green", label="lateral y"); ax[1].plot(ts, clr, color="tab:orange", label="wall clearance")
ax[1].axhline(0, color="firebrick", lw=1); ax[1].set_xlabel("time (s)"); ax[1].set_ylabel("mm")
ax[1].set_title("(B) Centering, then the slow instability"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
plt.tight_layout(); fig.savefig(ROOT/"outputs"/"e22_forward_cruise.png", dpi=130)
print(f"saved: {ROOT/'outputs'/'e22_forward_cruise.png'}")