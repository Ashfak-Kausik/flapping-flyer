"""
e27 — Takeoff and landing (the mission bookends).

TAKEOFF  hold low on the pad, then rate-limited climb of h_ref to cruise altitude.
  Ground effect boosts lift at liftoff, so a gentle climb-rate command (not full
  thrust) avoids a leap. Result: ~31 mm/s climb, <2 mm overshoot.

LANDING  controlled descent that FLARES on the ground-effect proximity signal:
  as the floor nears, the (low-pass) thrust signal grows; we cut the descent rate
  by up to ~80% so it settles gently, and apply a small thrust-cut to sink through
  the lift cushion. Result: touchdown speed ~0 mm/s.

HONEST LIMITS
  - We do NOT use MuJoCo contact: the 50 um wing geoms fall through the floor in the
    solver, so "touchdown" is modelled as a soft arrival at minimal clearance, not a
    physical contact. The DESCENT PROFILE (controlled, cushioned, gentle) is the result.
  - The capped ground-effect cushion (KAPPA_MAX) holds the flyer ~6 mm off the floor;
    it floats in its own cushion rather than resting (real ground-effect behaviour).
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design

Vc = 0.07; PAD = 0.006; CRUISE = 0.050; TOUCH = 0.003
CLIMB_RATE = 0.018; DESC_RATE = 0.014
FLOOR = dict(axis=2, sign=1, pos=0.0)


def run():
    fly = Flyer(Path("models/flyer.xml"))
    ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                             Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
    ctrl.K[:, 0] = 0.0
    ctrl.reset(); fly.reset(kin=kin, height=PAD); ctrl.h_ref = PAD
    t = 0.0; I_s = 0.0; thr_f = None; href = PAD; log = []
    while t < 12.5:
        s = fly.sense()
        if t < 1.0:   ph = 0; href = PAD; Vcmd = 0.0
        elif t < 4.0: ph = 1; href = min(href+CLIMB_RATE*fly.dt, CRUISE); Vcmd = 0.0
        elif t < 6.5: ph = 2; href = CRUISE; Vcmd = Vc
        else:         ph = 3; Vcmd = 0.0
        e = Vcmd-s['vx']; I_s = np.clip(I_s+e*fly.dt, -0.5, 0.5); pr = np.clip(0.25*e+0.9*I_s, -np.radians(8), np.radians(8))
        u = ctrl.update(s, fly.dt, pitch_ref=pr)
        thr_f = u[0] if thr_f is None else thr_f + (u[0]-thr_f)*fly.dt/0.15
        thrust = u[0]
        if ph == 3:
            closeness = np.clip((-thr_f-0.012)/0.020, 0.0, 1.0)
            href = max(href - DESC_RATE*(1.0-0.8*closeness)*fly.dt, TOUCH)
            if closeness > 0.5 and abs(s['vz']) < 0.02:
                thrust = u[0] - 0.012*closeness
        ctrl.h_ref = href
        kin.set_control(thrust=thrust, roll=u[1], pitch=u[2]); fly.step(kin, t, surface=FLOOR)
        log.append([t, fly.x_com[0]*1e3, fly.x_com[2]*1e3, href*1e3, s['vz']*1e3, thrust, thr_f, s['vx']*1e3, ph])
        t += fly.dt
    return np.array(log)


def make_figure(L, path="outputs/e27_takeoff_landing.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t, z, href, vz, ph = L[:, 0], L[:, 2], L[:, 3], L[:, 4], L[:, 8]
    cols = {0: '#cccccc', 1: '#bfe3c0', 2: '#bcd3f0', 3: '#f0cdbc'}
    fig, ax = plt.subplots(2, 1, figsize=(10, 7), sharex=True); fig.patch.set_facecolor('white')
    for p, lab in [(0, 'sit'), (1, 'takeoff'), (2, 'cruise'), (3, 'land')]:
        seg = t[ph == p]
        if len(seg): ax[0].axvspan(seg.min(), seg.max(), color=cols[p], alpha=0.5)
    ax[0].axhspan(-2, 0, color='#8d6e4f', alpha=0.5)
    ax[0].plot(t, z, color='#1d6fb8', lw=2, label='altitude'); ax[0].plot(t, href, 'k--', lw=1, label='h_ref')
    ax[0].set_ylabel('height (mm)'); ax[0].legend(fontsize=9, loc='upper right'); ax[0].grid(alpha=0.3)
    ax[0].set_title('Takeoff -> cruise -> landing  (shaded by phase; brown = floor)')
    ax[1].axhline(0, color='k', lw=0.5); ax[1].plot(t, vz, color='#127a3d', lw=1.4)
    ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('vertical speed (mm/s)'); ax[1].grid(alpha=0.3)
    la = L[L[:, 0] > L[-1, 0]-0.3]
    ax[1].set_title(f'Climb peak {L[(t>=1)&(t<4),4].max():.0f} mm/s; touchdown {la[:,4].mean():+.1f} mm/s')
    fig.tight_layout(); fig.savefig(path, dpi=110); print("saved figure ->", path)


if __name__ == "__main__":
    L = run()
    to = L[(L[:, 0] >= 1)&(L[:, 0] < 4)]; cr = L[(L[:, 0] >= 4)&(L[:, 0] < 6.5)]; la = L[L[:, 0] >= 6.5]
    print(f"takeoff peak climb {to[:,4].max():.0f} mm/s; cruise alt {cr[:,2].mean():.1f} mm; "
          f"touchdown vz {la[la[:,0]>la[-1,0]-0.3,4].mean():+.1f} mm/s; settle z {L[-1,2]:.1f} mm")
    np.savez("outputs/e27_takeoff_landing.npz", L=L)
    make_figure(L)