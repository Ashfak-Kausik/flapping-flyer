"""
e26 — Vertical FLOOR-FOLLOWING via the ground-effect proximity signal.

THEORY / WHY FLOOR-FOLLOWING (not tube-centering)
  Ground effect is applied per wing strip as kappa = 1 + K_GE (R/4d)^2. The wings
  span sideways, so a floor below boosts every strip equally -> a symmetric LIFT
  rise (less thrust needed), NOT a torque. Measured: thrust correction is monotonic
  in floor clearance (strong near, ~0 far) -> floor distance is directional and
  readable from thrust effort.
  Inside a symmetric tube the same signal is U-shaped (both walls boost lift) -> it
  gives |offset| but NOT up/down direction. So vertical direction is only observable
  against a FLOOR (or an asymmetric ceiling model, left as future work). We therefore
  hold a set CLEARANCE above the floor; as the floor ramps, the flyer tracks it.

LOOP
  signal  = low-pass(thrust command)          # vertical proximity (less thrust = closer)
  capture dstar = signal at the desired clearance during flat cruise
  h_ref  += clip(-Kh*(signal - dstar)) dt      # closer than target -> climb; farther -> descend

RESULT
  Tracks a 0->+25->0 mm floor ramp holding ~30 mm clearance (range 24-43 mm), cruise
  speed undisturbed. Up-slope tracks tighter than down-slope (1/d^2 signal weakens with
  distance) — improvable by inverting the calibration to linearize the gain.
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design

Vc = 0.07; CSTAR = 0.030; Kh = 8.0


def floor_h(x):
    if x < 0.15: return 0.0
    if x < 0.30: return 0.025*(x-0.15)/0.15
    if x < 0.45: return 0.025
    if x < 0.60: return 0.025*(1-(x-0.45)/0.15)
    return 0.0


def run():
    fly = Flyer(Path("models/flyer.xml"))
    ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                             Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
    ctrl.K[:, 0] = 0.0
    ctrl.reset(); fly.reset(kin=kin, height=CSTAR); ctrl.h_ref = CSTAR
    thr_f = None; I_s = 0.0; dstar = None; t = 0.0; log = []
    while t < 11.0:
        s = fly.sense(); x = fly.x_com[0]; fh = floor_h(x)
        surf = dict(axis=2, sign=1, pos=fh)
        e = Vc-s['vx']; I_s = np.clip(I_s+e*fly.dt, -0.5, 0.5); pr = np.clip(0.25*e+0.9*I_s, -np.radians(8), np.radians(8))
        u = ctrl.update(s, fly.dt, pitch_ref=pr)
        thr_f = u[0] if thr_f is None else thr_f + (u[0]-thr_f)*fly.dt/0.2
        if 2.0 <= t < 2.05: dstar = thr_f
        if t >= 2.05 and dstar is not None:
            ctrl.h_ref = np.clip(ctrl.h_ref + np.clip(-Kh*(thr_f-dstar), -0.03, 0.03)*fly.dt, 0.02, 0.10)
        kin.set_control(thrust=u[0], roll=u[1], pitch=u[2]); fly.step(kin, t, surface=surf)
        log.append([t, x*1e3, fly.x_com[2]*1e3, fh*1e3, (fly.x_com[2]-fh)*1e3, u[0], thr_f, s['vx']*1e3])
        t += fly.dt
    return np.array(log)


def make_figure(L, path="outputs/e26_floor_following.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t, x, z, fh, clr, vx = L[:, 0], L[:, 1], L[:, 2], L[:, 3], L[:, 4], L[:, 7]
    fig, ax = plt.subplots(2, 1, figsize=(10, 7)); fig.patch.set_facecolor('white')
    ax[0].fill_between(x, 0, fh, color='#8d6e4f', alpha=0.6, label='floor (ramp)')
    ax[0].plot(x, z, color='#1d6fb8', lw=2, label='flyer altitude')
    ax[0].set_xlabel('x (mm)'); ax[0].set_ylabel('height (mm)'); ax[0].legend(fontsize=9)
    ax[0].set_title('Floor-following: flyer rides over the hill holding clearance'); ax[0].grid(alpha=0.3)
    ax[1].axhline(CSTAR*1e3, color='k', ls='--', lw=1, label=f'target {CSTAR*1e3:.0f} mm')
    ax[1].plot(t, clr, color='#127a3d', lw=1.6, label='clearance (sensed via thrust)')
    ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('clearance (mm)'); ax[1].set_ylim(0, 60)
    ax[1].legend(fontsize=9); ax[1].grid(alpha=0.3)
    ax[1].set_title(f'Clearance held {clr[t>2.1].mean():.0f} mm (range {clr[t>2.1].min():.0f}-{clr[t>2.1].max():.0f}); cruise {vx[t>2.1].mean():.0f} mm/s')
    fig.tight_layout(); fig.savefig(path, dpi=110); print("saved figure ->", path)


if __name__ == "__main__":
    L = run(); m = L[:, 0] > 2.1
    print(f"clearance: mean={L[m,4].mean():.1f}mm range=[{L[m,4].min():.1f},{L[m,4].max():.1f}] (target {CSTAR*1e3:.0f}); "
          f"cruise={L[m,7].mean():.1f}mm/s")
    np.savez("outputs/e26_floor_following.npz", L=L, cstar=CSTAR)
    make_figure(L)