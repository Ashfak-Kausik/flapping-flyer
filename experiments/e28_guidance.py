"""
e28 — Goal-biased guidance (single "go to X" command -> homes to the point).

APPROACH (pure-pursuit, built on the VALIDATED e25 coordinated-turn law)
  Each step: bearing = atan2(goal-pos). Slew the heading reference toward that bearing
  (rate-limited) and feed it to the SAME feedforward+PD bank law e25 validated; the
  nose follows the velocity so sideslip stays ~0. A speed schedule ramps cruise down
  near the goal for a clean arrival. Reaching the capture radius hands off to landing.

  Hard-won lesson: an ad-hoc bearing loop with fresh gains diverges. Guidance MUST
  reuse the proven turn law (slew-limited reference, FF on dynamic gain, PD on the
  static gain, rate-error damping). Then goal-seeking is just "retarget the slew."

RESULT
  From cruise, homes to a goal 405 mm away requiring a 20 deg turn: reaches the 30 mm
  capture radius, sideslip mean ~2 deg / peak ~6 deg, altitude and speed steady.
  (Arrival hover-hold + landing handoff are wired in the integrated mission, not here.)
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design

Vc = 0.07; KFF = 60.0; KFB = 98.0; KP_BANK = 1.5; KD_BANK = 0.5; PSIDOT = 0.3
ROLL_FF = 175.0/10399.0; ALT = 0.05; CAPTURE = 0.030


def run(goal=(0.38, 0.14)):
    goal = np.asarray(goal)
    fly = Flyer(Path("models/flyer.xml"))
    ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                             Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
    ctrl.K[:, 0] = 0.0; ctrl.K[:, 1] = 0.0
    ctrl.reset(); fly.reset(kin=kin, height=ALT); ctrl.h_ref = ALT
    t = 0.0; I_s = 0.0; pref = 0.0; psiv_f = psiv_prev = nose_f = nose_prev = I_y = 0.0
    log = []; reached = False
    while t < 16.0:
        s = fly.sense(); x, y = fly.x_com[0], fly.x_com[1]
        dx, dy = goal[0]-x, goal[1]-y; dist = np.hypot(dx, dy); bearing = np.arctan2(dy, dx)
        spd = np.hypot(s['vx'], s['vy']); psi_vel = np.arctan2(s['vy'], s['vx'])
        psiv_f += ((psi_vel-psiv_f+np.pi) % (2*np.pi)-np.pi)*fly.dt/0.05
        vrate = ((psiv_f-psiv_prev+np.pi) % (2*np.pi)-np.pi)/fly.dt; psiv_prev = psiv_f
        nose_f += ((s['yaw']-nose_f+np.pi) % (2*np.pi)-np.pi)*fly.dt/0.04
        nrate = (nose_f-nose_prev)/fly.dt; nose_prev = nose_f
        target = 0.0 if t < 2.0 else bearing
        step = np.clip(((target-pref+np.pi) % (2*np.pi)-np.pi), -PSIDOT*fly.dt, PSIDOT*fly.dt)
        pref += step; pdot = step/fly.dt
        eb = ((pref-psiv_f+np.pi) % (2*np.pi)-np.pi)
        roll_ref = np.clip(pdot/KFF + (KP_BANK*eb+KD_BANK*(pdot-vrate))/KFB, -np.radians(1.2), np.radians(1.2))
        eyaw = ((nose_f-psiv_f+np.pi) % (2*np.pi)-np.pi); I_y = np.clip(I_y+eyaw*fly.dt, -1.5, 1.5)
        uy = np.clip(0.15*eyaw + 0.02*nrate + 0.15*I_y, -0.3, 0.3)
        Vcmd = Vc * (1.0 if t < 2.0 else np.clip(dist/0.12, 0.2, 1.0))   # decelerate into goal
        e = Vcmd-spd; I_s = np.clip(I_s+e*fly.dt, -0.6, 0.6); pr = np.clip(0.30*e+1.1*I_s, -np.radians(8), np.radians(8))
        u = ctrl.update(s, fly.dt, pitch_ref=pr, roll_ref=roll_ref)
        kin.set_control(thrust=u[0], roll=u[1]-ROLL_FF*uy, pitch=u[2], yaw=uy)
        fly.step(kin, t)
        beta = (np.degrees(s['yaw']-psi_vel)+180) % 360-180
        log.append([t, x*1e3, y*1e3, fly.x_com[2]*1e3, np.degrees(s['yaw']), np.degrees(psi_vel),
                    np.degrees(s['roll']), beta, spd*1e3, dist*1e3])
        if dist < CAPTURE: reached = True; break
        t += fly.dt
    return np.array(log), reached, goal*1e3


def make_figure(L, goal, path="outputs/e28_guidance.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t, x, y, beta, spd, dist = L[:, 0], L[:, 1], L[:, 2], L[:, 7], L[:, 8], L[:, 9]
    nose = L[:, 4]
    fig, ax = plt.subplots(1, 2, figsize=(12, 5)); fig.patch.set_facecolor('white')
    ax[0].plot(x, y, color='#1d6fb8', lw=2, label='path')
    ax[0].plot(0, 0, 'go', ms=9, label='start')
    ax[0].plot(goal[0], goal[1], 'r*', ms=16, label='goal')
    circ = plt.Circle((goal[0], goal[1]), 30, color='r', fill=False, ls='--', alpha=0.6)
    ax[0].add_patch(circ)
    for i in range(0, len(t), 500):
        a = np.radians(nose[i]); ax[0].arrow(x[i], y[i], 16*np.cos(a), 16*np.sin(a), head_width=7, color='#999', length_includes_head=True)
    ax[0].set_aspect('equal'); ax[0].set_xlabel('x (mm)'); ax[0].set_ylabel('y (mm)')
    ax[0].set_title('Guidance: one goal -> homes to it (arrows = nose)'); ax[0].legend(fontsize=9); ax[0].grid(alpha=0.3)
    ax[1].plot(t, beta, color='#127a3d', label='sideslip'); ax[1].axhline(0, color='k', lw=0.5)
    ax[1].plot(t, dist/10.0, color='#c0392b', ls='--', label='dist/10 (mm)')
    ax[1].axvline(2.0, color='orange', alpha=0.4, lw=1); ax[1].set_ylim(-15, 45)
    ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('deg  /  dist/10')
    st = (t > 2.0) & (dist > 40)
    ax[1].set_title(f'Sideslip mean {np.mean(np.abs(beta[st])):.1f} deg (coordinated); closing on goal'); ax[1].legend(fontsize=9); ax[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=110); print("saved figure ->", path)


if __name__ == "__main__":
    L, reached, goal = run()
    st = (L[:, 0] > 2.0) & (L[:, 9] > 40)
    print(f"reached capture={reached} at t={L[-1,0]:.1f}s, final dist {L[-1,9]:.1f} mm, pos=({L[-1,1]:.0f},{L[-1,2]:.0f}); "
          f"transit sideslip mean {np.mean(np.abs(L[st,7])):.1f} peak {np.max(np.abs(L[st,7])):.1f} deg; "
          f"alt {L[st,3].mean():.1f}+-{L[st,3].std():.1f} mm")
    np.savez("outputs/e28_guidance.npz", L=L, goal=goal)
    make_figure(L, goal)