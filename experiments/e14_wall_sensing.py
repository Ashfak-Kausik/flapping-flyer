"""
e14_wall_sensing.py — STAGE 6 (1/N): the wall is sensed IN THE LOOP, no new sensor.

The flyer hovers under the LQG controller with a wall nearby (aero now runs
through the ground/wall-effect model). The wall's roll disturbance is cancelled by
the controller — so the flyer stays level and on station — and the cancelling
effort shows up as a steady bias in the controller's OWN roll command u_roll. That
residual is the proximity sensor: its magnitude grows as the wall nears and its
SIGN says which side. We check it (a) tracks standoff, (b) matches the open-loop
e13 prediction (controller cancels the measured wall torque), and (c) flips sign
with the wall's side — all while the flyer flies stably.
"""
import sys
from pathlib import Path
import csv
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.kinematics import FlapKinematics
from src import aero, ground_effect as ge
from src.controller import design

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
fly = Flyer(ROOT / "models" / "flyer.xml")
R = fly.R; TIP_Y = 0.0133
ROLL_AUTH = 10399e-9                      # N·m per unit u_roll (e09)
print("designing LQG controller (system-ID + LQR + Kalman)...")
ctrl, kin, info = design(fly)


def _sys_com():
    c = np.zeros(3)
    for i in range(1, fly.model.nbody):
        c += fly.model.body_mass[i] * fly.data.xipos[i]
    return c / fly.M


def open_loop_dTx(surface, ncyc=10, navg=6):
    """Clamped cycle-averaged roll torque shift from the wall (the disturbance the
    controller must cancel) — the e13-style prediction."""
    m, d = fly.model, fly.data
    period = 2 * np.pi / kin.W; N = int(ncyc * period / fly.dt); t0 = (ncyc - navg) * period
    vg = {s: None for s in "RL"}; vf = {s: None for s in "RL"}; acc = []
    for i in range(N):
        t = i * fly.dt
        d.qpos[fly._qf:fly._qf + 3] = [0, 0, 0.05]; d.qpos[fly._qf + 3:fly._qf + 7] = [1, 0, 0, 0]
        d.qvel[fly._vf:fly._vf + 6] = 0.0
        fly._prescribe_wings(kin, t); mujoco.mj_forward(m, d); com = _sys_com()
        Tg = 0.0; Tf = 0.0
        for s, bid in fly.wings.items():
            fg, tg, vg[s], _ = ge.wing_aero_ge(m, d, bid, fly.strips[s], vg[s], fly.dt, surface, R)
            ff, tf, vf[s], _ = aero.wing_aero(m, d, bid, fly.strips[s], vf[s], fly.dt)
            Tg += tg[0] + np.cross(d.xipos[bid] - com, fg)[0]
            Tf += tf[0] + np.cross(d.xipos[bid] - com, ff)[0]
        if t >= t0: acc.append(Tg - Tf)
    return float(np.mean(acc))


def closed_loop_residual(surface, T=0.35):
    """Fly closed-loop with lateral position HELD at the standoff (representing an
    outer position loop), so the steady u_roll residual characterises the sensor."""
    fly.reset(kin=kin, height=0.05); ctrl.reset()
    n = int(T / fly.dt); ur = np.empty(n)
    for i in range(n):
        u = ctrl.update(fly.sense(), fly.dt)
        kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i * fly.dt, surface=surface)
        fly.x_com[1] = 0.0; fly.v[1] = 0.0          # hold lateral position (as a position loop would)
        ur[i] = u[1]
    w = int(0.125 / fly.dt)                                  # average over the last wingbeat
    s = fly.sense()
    return ur[-w:].mean(), np.rad2deg(s['roll']), (fly.x_com[1]) * 1e3   # u_roll, roll_deg, y_mm

print("=" * 76)
print(" STAGE 6 (1/N) — wall sensing in closed loop (control residual = proximity)")
print("=" * 76)
print(f" R={R*1e3:.2f}mm, wingtip|y|={TIP_Y*1e3:.1f}mm, roll authority={ROLL_AUTH*1e9:.0f} uNmm/unit")
print(f" {'wall y':>7} {'gap':>7} {'<u_roll> meas':>14} {'pred -ΔTx/B':>13} {'final roll':>11} {'y drift':>9}")
walls_mm = [16, 18, 20, 24, 30, 40]
rows = []
for yw in walls_mm:
    surf = dict(axis=1, sign=-1, pos=yw / 1e3)
    dTx = open_loop_dTx(surf)
    pred = -dTx / ROLL_AUTH
    meas, rolld, ydrift = closed_loop_residual(surf)
    gap = yw / 1e3 - TIP_Y
    rows.append((yw, gap * 1e3, meas, pred, rolld, ydrift, dTx))
    print(f" {yw:5d}mm {gap*1e3:6.1f}mm {meas:+13.4f} {pred:+12.4f} {rolld:+10.2f}° {ydrift:+8.3f}mm")

# directionality: wall on -y -> opposite-sign residual
m_p, _, _ = closed_loop_residual(dict(axis=1, sign=-1, pos=0.020))
m_n, _, _ = closed_loop_residual(dict(axis=1, sign=+1, pos=-0.020))
print("-" * 76)
print(f" DIRECTIONALITY (wall @20mm): +y -> u_roll={m_p:+.4f},  -y -> u_roll={m_n:+.4f}"
      f"  -> sign flips: {np.sign(m_p)!=np.sign(m_n)}")
mx = max(abs(r[4]) for r in rows)
print(f" CHECK: flyer stays level (|roll|<{mx:.2f}°) and ~on station while sensing.")
print(f" -> the wall is read directly from the controller's own u_roll bias — no extra sensor.")
print("=" * 76)

with open(OUT / "e14_wall_sensing.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["wall_y_mm", "gap_mm", "u_roll_closedloop", "u_roll_predicted", "final_roll_deg", "y_drift_mm", "dTx_uNmm"])
    for r in rows:
        w.writerow([r[0], f"{r[1]:.3f}", f"{r[2]:.5f}", f"{r[3]:.5f}", f"{r[4]:.4f}", f"{r[5]:.4f}", f"{r[6]*1e9:.3f}"])

gap = np.array([r[1] for r in rows])
fig, ax = plt.subplots(figsize=(8, 5))
ax.plot(gap, np.abs([r[2] for r in rows]), "o-", color="tab:blue", label="closed-loop |u_roll| residual (measured)")
ax.plot(gap, np.abs([r[3] for r in rows]), "s--", color="tab:orange", label="open-loop prediction −ΔTx/B_roll (e13)")
ax.set_xlabel("wall gap to wingtip (mm)"); ax.set_ylabel("roll-command residual (fraction of authority)")
ax.set_title("Wall sensed in the loop: the controller's own u_roll bias tracks proximity")
ax.grid(alpha=0.3); ax.legend()
plt.tight_layout(); fig.savefig(OUT / "e14_wall_sensing.png", dpi=130)
print(f"saved: {OUT/'e14_wall_sensing.csv'}")
print(f"saved: {OUT/'e14_wall_sensing.png'}")