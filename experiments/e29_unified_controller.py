"""
e29 — Unified controller: cruise, coordinated turn, and stop-and-hover on ONE
controller whose gains never change (no mode-switch -> nothing to diverge).

WHY (the dead end this resolves)
  Coordinated turns needed the LQG lateral gain zeroed (so an external bank law owns
  the bank). Hover/landing needed lateral hold. Switching the gain matrix mid-flight
  diverged every time (stale velocity + wound-up roll-disturbance feedforward kick it).

THE FIX (no gain switch)
  Keep K[:,0]=K[:,1]=0 ALWAYS. An OUTER loop owns roll_ref and computes it two ways:
    - MOVING/TURNING : e25 bank-steering law (FF on dynamic gain + PD on velocity
                       heading); nose follows velocity -> coordinated turn.
    - HOVER/STOP     : lateral-velocity damping  roll_ref = -KLAT * v_lat.
  The inner controller is identical in both; only the outer formula changes (smooth).
  Forward speed uses SIGNED body-forward velocity u_fwd (NOT total speed): total speed
  is always positive, so a stop loop on it cannot tell forward from backward and drives
  a backward runaway. Signed u_fwd fixes that.

RESULT
  cruise -> coordinated turn (sideslip ~2 deg) -> stop & hold (drift ~2 mm/1.5s),
  altitude steady, no divergence. This is the controller the integrated mission runs on.
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design

Vc = 0.07; PSIDOT = 0.3; KFF = 60.0; KFB = 98.0; KP_BANK = 1.5; KD_BANK = 0.5
KLAT = 3.0; ROLL_FF = 175.0/10399.0


def run(turn_to_deg=60.0):
    fly = Flyer(Path("models/flyer.xml"))
    ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                             Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
    ctrl.K[:, 0] = 0.0; ctrl.K[:, 1] = 0.0          # never changed again
    ctrl.reset(); fly.reset(kin=kin, height=0.05); ctrl.h_ref = 0.05
    t = 0.0; I_s = 0.0; pref = 0.0; psiv_f = psiv_prev = nose_f = nose_prev = I_y = 0.0
    log = []; TARGET = np.radians(turn_to_deg)
    while t < 11.0:
        s = fly.sense(); psi = s['yaw']; spd = np.hypot(s['vx'], s['vy'])
        u_fwd = s['vx']*np.cos(psi)+s['vy']*np.sin(psi)
        v_lat = -s['vx']*np.sin(psi)+s['vy']*np.cos(psi)
        psi_vel = np.arctan2(s['vy'], s['vx'])
        psiv_f += ((psi_vel-psiv_f+np.pi) % (2*np.pi)-np.pi)*fly.dt/0.05
        vrate = ((psiv_f-psiv_prev+np.pi) % (2*np.pi)-np.pi)/fly.dt; psiv_prev = psiv_f
        nose_f += ((psi-nose_f+np.pi) % (2*np.pi)-np.pi)*fly.dt/0.04
        nrate = (nose_f-nose_prev)/fly.dt; nose_prev = nose_f
        if t < 7.0:   # MOVE (cruise then turn)
            target = 0.0 if t < 2.0 else TARGET; Vcmd = Vc
            step = np.clip(((target-pref+np.pi) % (2*np.pi)-np.pi), -PSIDOT*fly.dt, PSIDOT*fly.dt); pref += step; pdot = step/fly.dt
            eb = ((pref-psiv_f+np.pi) % (2*np.pi)-np.pi)
            roll_ref = np.clip(pdot/KFF + (KP_BANK*eb+KD_BANK*(pdot-vrate))/KFB, -np.radians(1.2), np.radians(1.2))
            eyaw = ((nose_f-psiv_f+np.pi) % (2*np.pi)-np.pi)
            uy = np.clip(0.15*eyaw+0.02*nrate+0.15*I_y, -0.3, 0.3); I_y = np.clip(I_y+eyaw*fly.dt, -1.5, 1.5)
        else:         # HOVER / STOP
            Vcmd = 0.0; roll_ref = np.clip(-KLAT*v_lat, -np.radians(2.0), np.radians(2.0))
            eyaw = ((nose_f-TARGET+np.pi) % (2*np.pi)-np.pi)
            uy = np.clip(0.10*eyaw+0.02*nrate+0.12*I_y, -0.3, 0.3); I_y = np.clip(I_y+eyaw*fly.dt, -1.5, 1.5)
        e = Vcmd-u_fwd; I_s = np.clip(I_s+e*fly.dt, -0.6, 0.6); pr = np.clip(0.30*e+1.1*I_s, -np.radians(8), np.radians(8))
        u = ctrl.update(s, fly.dt, pitch_ref=pr, roll_ref=roll_ref)
        kin.set_control(thrust=u[0], roll=u[1]-ROLL_FF*uy, pitch=u[2], yaw=uy); fly.step(kin, t)
        beta = (np.degrees(psi-psi_vel)+180) % 360-180 if spd > 0.01 else 0.0
        log.append([t, fly.x_com[0]*1e3, fly.x_com[1]*1e3, fly.x_com[2]*1e3, np.degrees(psi),
                    np.degrees(psi_vel), np.degrees(s['roll']), beta, spd*1e3]); t += fly.dt
    return np.array(log)


def make_figure(L, path="outputs/e29_unified_controller.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t, x, y, beta = L[:, 0], L[:, 1], L[:, 2], L[:, 7]
    fig, ax = plt.subplots(1, 2, figsize=(12, 5)); fig.patch.set_facecolor('white')
    mv = t < 7.0
    ax[0].plot(x[mv], y[mv], color='#1d6fb8', lw=2, label='cruise+turn')
    ax[0].plot(x[~mv], y[~mv], color='#c0392b', lw=2, label='stop & hold')
    ax[0].plot(x[0], y[0], 'go', ms=8); ax[0].plot(x[-1], y[-1], 'ks', ms=8, label='final')
    ax[0].set_aspect('equal'); ax[0].set_xlabel('x (mm)'); ax[0].set_ylabel('y (mm)'); ax[0].legend(fontsize=9)
    ax[0].set_title('One controller: cruise -> coordinated turn -> stop & hold'); ax[0].grid(alpha=0.3)
    ax[1].plot(t, beta, color='#127a3d'); ax[1].axvspan(2, 7, alpha=0.08, color='orange'); ax[1].axvline(7, color='r', ls='--', alpha=0.5)
    ax[1].axhline(0, color='k', lw=0.5); ax[1].set_ylim(-15, 15); ax[1].set_xlabel('time (s)'); ax[1].set_ylabel('sideslip (deg)')
    cr = (t > 2.5) & (t < 6.5)
    ax[1].set_title(f'Sideslip: turn mean {np.mean(np.abs(beta[cr])):.1f} deg; stop at red line'); ax[1].grid(alpha=0.3)
    fig.tight_layout(); fig.savefig(path, dpi=110); print("saved ->", path)


if __name__ == "__main__":
    L = run(); cr = (L[:, 0] > 2.5) & (L[:, 0] < 6.5); st = L[:, 0] > 9.5
    print(f"turn sideslip mean {np.mean(np.abs(L[cr,7])):.1f} peak {np.max(np.abs(L[cr,7])):.1f} deg; "
          f"stop drift(last1.5s) {np.hypot(L[-1,1]-L[st,1][0],L[-1,2]-L[st,2][0]):.1f} mm; alt {L[st,3].mean():.1f} mm")
    np.savez("outputs/e29_unified_controller.npz", L=L); make_figure(L)