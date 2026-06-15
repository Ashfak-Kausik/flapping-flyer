"""
e24_yaw_heading.py — ARC B: closing the yaw heading loop (the docs/07 open problem).

docs/07 logged yaw HEADING CONTROL as unsolved: under a constant split-cycle command
the free-flight yaw rate oscillated and feeding heading back tumbled the flyer. A
yaw-dynamics diagnosis resolved it.

DIAGNOSIS (the key insight). Imposing a steady body yaw rate on a level flapping flyer
and measuring the cycle-averaged torque shows dTz/dwz < 0 (~ -3.85 uN.mm per rad/s): yaw
is NATURALLY WELL-DAMPED (time constant ~0.19 s), a stable first-order rate response — a
constant u_yaw settles to a steady turn rate, it does NOT run away. So the oscillation was
never the yaw physics; it was the CONTROLLER feeding back a noisy wz rate term for damping
that the aerodynamics already provide.

CONTROLLER (the fix). Drop the noisy wz feedback. Use heading control on a LOW-PASS
filtered heading (removes wingbeat wobble), with:
  - proportional + derivative-of-filtered-heading (a clean rate), relying on natural
    aero damping for most of the damping;
  - INTEGRAL action to reject a constant yaw-torque bias (the LQG's own roll/pitch
    commands inject a small yaw cross-coupling -> proportional alone leaves a steady error);
  - a SLEW-LIMITED heading reference so the flyer turns gradually and u_yaw (hence its
    roll coupling) stays small;
  - feed-forward roll-coupling cancellation (u_roll -= 0.0168*u_yaw, from e23).

RESULT. Moderate commanded turns (<= ~45 deg) track cleanly and stably: e.g. +30 deg ->
30.5 deg, max|roll| 0.7 deg, altitude held. Large sharp turns (90 deg) reach the
neighbourhood but settle short and disturb altitude (the bigger maneuver drives more
coupling) — they need gentler slewing / altitude coordination, logged as refinement.
For tunnel bends (typically gentle) the heading loop now works.
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
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                         Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
ROLL_FF = 175.0 / 10399.0


def turn(tgt_deg, Kpsi=0.08, Kd=0.02, Ki=0.10, tau=0.04, slew=30.0, T=6.0, umax=0.3, log=False):
    ctrl.reset(); fly.reset(kin=kin, height=0.05)
    tgt = np.radians(tgt_deg); psi_f = 0.0; psi_prev = 0.0; pref = 0.0; I = 0.0
    ts, ph, roll, h = [], [], [], []
    for i in range(int(T / fly.dt)):
        s = fly.sense()
        psi_f += (s['yaw'] - psi_f) * fly.dt / tau                 # filtered heading
        rate = (psi_f - psi_prev) / fly.dt; psi_prev = psi_f       # clean rate
        pref += np.clip(tgt - pref, -np.radians(slew)*fly.dt, np.radians(slew)*fly.dt)
        I = np.clip(I + (psi_f - pref) * fly.dt, -1.5, 1.5)        # reject coupling bias
        uy = np.clip(Kpsi*(psi_f - pref) + Kd*rate + Ki*I, -umax, umax)
        u = ctrl.update(s, fly.dt)
        kin.set_control(thrust=u[0], roll=u[1] - ROLL_FF*uy, pitch=u[2], yaw=uy)
        fly.step(kin, i*fly.dt, surface=None)
        if log and i % int(0.1/fly.dt) == 0:
            ts.append(i*fly.dt); ph.append(np.degrees(psi_f)); roll.append(np.degrees(s['roll'])); h.append(s['height']*1e3)
    if log: return ts, ph, roll, h
    return np.degrees(psi_f), max(abs(np.degrees(fly.sense()['roll'])), 0)


print("commanded heading turns (PI+D on filtered heading, slew-limited, roll-FF):")
for tgt in (30, -30):
    ts, ph, roll, h = turn(tgt, log=True)
    print(f" target {tgt:+}°: final {ph[-1]:.1f}°  err {abs(ph[-1]-tgt):.1f}°  max|roll| {max(abs(r) for r in roll):.1f}°  h {h[-1]:.0f}mm")
    if tgt == 30:
        fig, ax = plt.subplots(figsize=(7, 4.2))
        ax.plot(ts, ph, color="tab:blue", label="heading")
        ax.axhline(tgt, ls="--", color="gray", lw=1, label="target")
        ax.plot(ts, roll, color="tab:red", lw=1, label="roll")
        ax.set_xlabel("time (s)"); ax.set_ylabel("degrees")
        ax.set_title(f"Commanded {tgt}° turn — stable, accurate, roll bounded")
        ax.legend(fontsize=8); ax.grid(alpha=0.3)
        plt.tight_layout(); fig.savefig(ROOT/"outputs"/"e24_yaw_heading.png", dpi=130)
        print(f" saved: {ROOT/'outputs'/'e24_yaw_heading.png'}")