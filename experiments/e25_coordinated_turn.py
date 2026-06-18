"""
e25 — Coordinated turn via body/heading-frame reformulation.

THEORY
  A flapping flyer has no body sideforce; the only horizontal force comes from
  BANKING (rolling tilts the lift, its horizontal component is centripetal).
  In the heading frame the sideslip velocity obeys  v_dot = g*tan(phi) - u*psi_dot,
  so a coordinated turn (v=0) needs  phi = atan(u*psi_dot/g).  Yaw rotates the nose
  but makes NO force -> yaw-alone crabs (~80 deg sideslip).

ARCHITECTURE (what works)
  - decouple BOTH translational velocities from the LQG (K[:,0]=K[:,1]=0): it then
    holds attitude + height only, tracking our roll_ref / pitch_ref commands.
  - hold TOTAL speed with the cruise pitch loop.
  - BANK loop steers the VELOCITY heading to the commanded heading:
        roll_ref = psi_dot/KFF + (KP*e + KD*(psi_dot - vrate))/KFB
    (FF on the lower dynamic gain; PD feedback on the higher static gain; the
     derivative acts on the RATE ERROR, not the absolute rate).
  - YAW loop points the NOSE at the VELOCITY -> sideslip nulls automatically.

Run:  python experiments/e25_coordinated_turn.py
  -> prints calibration + turn stats, saves outputs/e25_coordinated_turn.png
"""
import sys; from pathlib import Path; import numpy as np
sys.path.insert(0, '.')
from src.flyer import Flyer
from src.controller import design

g = 9.81; Vc = 0.07; ROLL_FF = 175.0/10399.0
KFF, KFB = 60.0, 98.0; KP_BANK, KD_BANK = 1.5, 0.5
PSIDOT = 0.3; TURN_DEG = 90.0


def build():
    fly = Flyer(Path("models/flyer.xml"))
    ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                             Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
    ctrl.K[:, 0] = 0.0; ctrl.K[:, 1] = 0.0
    return fly, ctrl, kin


def calibrate(fly, ctrl, kin, banks=(-0.5, -0.2, 0.2, 0.5)):
    out = []
    for rd in banks:
        ctrl.reset(); fly.reset(kin=kin, height=0.05); I_s = 0.0; win = []
        for i in range(int(3.0/fly.dt)):
            t = i*fly.dt; s = fly.sense(); spd = np.hypot(s['vx'], s['vy'])
            e = Vc-spd; I_s = np.clip(I_s+e*fly.dt, -0.6, 0.6); pr = np.clip(0.30*e+1.1*I_s, -np.radians(8), np.radians(8))
            u = ctrl.update(s, fly.dt, pitch_ref=pr, roll_ref=(np.radians(rd) if t >= 1.5 else 0.0))
            kin.set_control(thrust=u[0], roll=u[1], pitch=u[2]); fly.step(kin, t)
            if 1.7 <= t < 2.3: win.append((t, np.arctan2(s['vy'], s['vx'])))
        w = np.array(win); rate = np.degrees(np.polyfit(w[:, 0], np.unwrap(w[:, 1]), 1)[0])
        out.append((rd, rate))
    return out


def coordinated_turn(fly, ctrl, kin, psidot=PSIDOT, turn_deg=TURN_DEG, settle=2.0):
    ctrl.reset(); fly.reset(kin=kin, height=0.05)
    pref = I_s = 0.0; t = 0.0; log = []
    psiv_f = psiv_prev = nose_f = nose_prev = I_y = 0.0
    TURN = np.radians(turn_deg); t_end = 2.0 + TURN/psidot
    while t < t_end + settle:
        s = fly.sense()
        pdot = psidot if (2.0 <= t < t_end) else 0.0
        pref += pdot*fly.dt
        psi_vel = np.arctan2(s['vy'], s['vx'])
        psiv_f += ((psi_vel-psiv_f+np.pi) % (2*np.pi)-np.pi)*fly.dt/0.05
        vrate = ((psiv_f-psiv_prev+np.pi) % (2*np.pi)-np.pi)/fly.dt; psiv_prev = psiv_f
        nose_f += ((s['yaw']-nose_f+np.pi) % (2*np.pi)-np.pi)*fly.dt/0.04
        nrate = (nose_f-nose_prev)/fly.dt; nose_prev = nose_f
        eb = ((pref-psiv_f+np.pi) % (2*np.pi)-np.pi)
        roll_ref = np.clip(pdot/KFF + (KP_BANK*eb + KD_BANK*(pdot-vrate))/KFB, -np.radians(1.2), np.radians(1.2))
        eyaw = ((nose_f-psiv_f+np.pi) % (2*np.pi)-np.pi)
        I_y = np.clip(I_y+eyaw*fly.dt, -1.5, 1.5)
        uy = np.clip(0.15*eyaw + 0.02*nrate + 0.15*I_y, -0.3, 0.3)
        spd = np.hypot(s['vx'], s['vy']); e = Vc-spd; I_s = np.clip(I_s+e*fly.dt, -0.6, 0.6)
        pr = np.clip(0.30*e + 1.1*I_s, -np.radians(8), np.radians(8))
        u = ctrl.update(s, fly.dt, pitch_ref=pr, roll_ref=roll_ref)
        kin.set_control(thrust=u[0], roll=u[1]-ROLL_FF*uy, pitch=u[2], yaw=uy)
        fly.step(kin, t)
        beta = (np.degrees(s['yaw']-psi_vel)+180) % 360-180
        log.append([t, fly.x_com[0]*1e3, fly.x_com[1]*1e3, fly.x_com[2]*1e3,
                    np.degrees(s['yaw']), np.degrees(psi_vel), np.degrees(s['roll']), beta, spd*1e3])
        t += fly.dt
    return np.array(log)


def make_figure(L, psidot=PSIDOT, turn_deg=TURN_DEG, path="outputs/e25_coordinated_turn.png"):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    t, x, y = L[:, 0], L[:, 1], L[:, 2]
    nose, vhead, roll, beta, spd = L[:, 4], L[:, 5], L[:, 6], L[:, 7], L[:, 8]
    tte = 2.0 + np.radians(turn_deg)/psidot; m = (t >= 2.0) & (t <= tte)
    fig, ax = plt.subplots(2, 2, figsize=(11, 8)); fig.patch.set_facecolor('white')
    ax[0, 0].plot(x, y, '-', color='#1d6fb8', lw=2); ax[0, 0].plot(x[0], y[0], 'go', ms=8, label='start')
    ax[0, 0].plot(x[m][0], y[m][0], 'o', color='orange', ms=7, label='turn begins'); ax[0, 0].plot(x[-1], y[-1], 'rs', ms=8, label='end')
    for i in range(0, len(t), 700):
        a = np.radians(nose[i]); ax[0, 0].arrow(x[i], y[i], 14*np.cos(a), 14*np.sin(a), head_width=6, color='#888', length_includes_head=True)
    ax[0, 0].set_aspect('equal'); ax[0, 0].set_xlabel('x (mm)'); ax[0, 0].set_ylabel('y (mm)')
    ax[0, 0].set_title('Flight path (grey arrows = nose heading)'); ax[0, 0].legend(fontsize=8); ax[0, 0].grid(alpha=0.3)
    ax[0, 1].plot(t, nose, label='nose heading', color='#c0392b'); ax[0, 1].plot(t, vhead, '--', label='velocity heading', color='#1d6fb8')
    ax[0, 1].axvspan(2.0, tte, alpha=0.08, color='orange'); ax[0, 1].set_xlabel('time (s)'); ax[0, 1].set_ylabel('deg')
    ax[0, 1].set_title('Nose vs velocity heading'); ax[0, 1].legend(fontsize=8); ax[0, 1].grid(alpha=0.3)
    ax[1, 0].plot(t, beta, color='#127a3d'); ax[1, 0].axvspan(2.0, tte, alpha=0.08, color='orange'); ax[1, 0].axhline(0, color='k', lw=0.5)
    ax[1, 0].set_xlabel('time (s)'); ax[1, 0].set_ylabel('sideslip (deg)'); ax[1, 0].set_ylim(-15, 15)
    ax[1, 0].set_title(f'Sideslip (turn: mean {np.mean(np.abs(beta[m])):.1f}, peak {np.max(np.abs(beta[m])):.1f} deg)'); ax[1, 0].grid(alpha=0.3)
    axb = ax[1, 1]; axb.plot(t, roll, color='#7d3c98'); axb.axvspan(2.0, tte, alpha=0.08, color='orange')
    axb.set_xlabel('time (s)'); axb.set_ylabel('roll (deg)', color='#7d3c98'); axb.grid(alpha=0.3)
    axc = axb.twinx(); axc.plot(t, spd, color='#e67e22', lw=1, alpha=0.7); axc.set_ylabel('speed (mm/s)', color='#e67e22'); axc.set_ylim(0, 120)
    axb.set_title('Bank angle (purple) & speed (orange)')
    fig.suptitle(f'e25 - Coordinated turn ({turn_deg:.0f} deg @ {psidot} rad/s, cruise {Vc*1e3:.0f} mm/s): crab eliminated', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.96]); fig.savefig(path, dpi=110); print("saved figure ->", path)


if __name__ == "__main__":
    fly, ctrl, kin = build()
    print("calibration (bank -> velocity turn rate):")
    for rd, rate in calibrate(fly, ctrl, kin):
        print(f"  {rd:+.1f} deg -> {rate:+.1f} deg/s")
    L = coordinated_turn(fly, ctrl, kin)
    tte = 2.0 + np.radians(TURN_DEG)/PSIDOT; m = (L[:, 0] >= 2.0) & (L[:, 0] <= tte)
    print(f"coordinated turn: vel_head reached {L[-1,5]:.0f} deg; "
          f"sideslip mean {np.mean(np.abs(L[m,7])):.1f}, peak {np.max(np.abs(L[m,7])):.1f} deg; "
          f"roll [{L[:,6].min():.1f},{L[:,6].max():.1f}] deg; dz {L[-1,3]-L[0,3]:+.1f} mm")
    np.savez("outputs/e25_coordinated_turn.npz", co=L, meta=[Vc, PSIDOT, TURN_DEG])
    make_figure(L)