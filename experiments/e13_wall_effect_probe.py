"""
e13_wall_effect_probe.py — flow-proximity feasibility, the WALL (directional) case.

The narrow-space claim needs more than "floor below" — it needs "wall to one side,
and which side." Here the flyer hovers LEVEL at 50 mm (vertically out of ground
effect, so the floor is irrelevant) and we sweep a vertical wall in from +y,
measuring how the cycle-averaged wrench shifts versus free air.

Model: a vertical surface acts as a REFLECTION PLANE for the near wing (an
endplate / image-wing effect), enhancing its lift via the same (R/4d)^2 proximity
law used for the floor (src/ground_effect.py, per-strip, distance to the wall).
The near wing gaining lift produces a ROLL torque whose SIGN tells which side the
wall is on — the directional signal an avoidance loop needs.

HONEST SCOPE: floor ground effect is well established; the wall/endplate magnitude
and even sign are less settled in the literature (a real wall may also exert a
lateral suction this model omits). So treat the MAGNITUDE as model-dependent and
calibration-pending, but the DIRECTIONALITY (a side-dependent roll signal that
flips with the wall's side) is the robust, usable result. K_GE is the coefficient
to pin down for sim-to-real.
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

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
AMP = 72.63
fly = Flyer(ROOT / "models" / "flyer.xml")
R = aero.wing_params_from_model(fly.model)["r_tip"]
WEIGHT = fly.M * 9.81
ROLL_AUTH = 10399e-9          # roll torque per unit u_roll (N·m/unit), from e09/B
TIP_Y = 0.0133                # max wingtip |y| over a cycle (measured)
kin = FlapKinematics(f_hz=80, stroke_amp_deg=AMP, feather_amp_deg=45)


def _sys_com():
    c = np.zeros(3)
    for i in range(1, fly.model.nbody):
        c += fly.model.body_mass[i] * fly.data.xipos[i]
    return c / fly.M


def measure(surface, height=0.05, ncyc=12, navg=8):
    m, d = fly.model, fly.data
    period = 2 * np.pi / kin.W; N = int(ncyc * period / fly.dt); t0 = (ncyc - navg) * period
    vn_g = {s: None for s in "RL"}; vn_f = {s: None for s in "RL"}
    acc_g = []; acc_f = []; tx_inst = []
    for i in range(N):
        t = i * fly.dt
        d.qpos[fly._qf:fly._qf + 3] = [0.0, 0.0, height]
        d.qpos[fly._qf + 3:fly._qf + 7] = [1.0, 0.0, 0.0, 0.0]
        d.qvel[fly._vf:fly._vf + 6] = 0.0
        fly._prescribe_wings(kin, t)
        mujoco.mj_forward(m, d)
        com = _sys_com()
        Fg = np.zeros(3); Tg = np.zeros(3); Ff = np.zeros(3); Tf = np.zeros(3)
        for s, bid in fly.wings.items():
            fg, tg, vn_g[s], _ = ge.wing_aero_ge(m, d, bid, fly.strips[s], vn_g[s], fly.dt, surface, R)
            ff, tf, vn_f[s], _ = aero.wing_aero(m, d, bid, fly.strips[s], vn_f[s], fly.dt)
            Fg += fg; Tg += tg + np.cross(d.xipos[bid] - com, fg)
            Ff += ff; Tf += tf + np.cross(d.xipos[bid] - com, ff)
        if t >= t0:
            acc_g.append(np.concatenate([Fg, Tg])); acc_f.append(np.concatenate([Ff, Tf]))
            tx_inst.append(Tg[0])
    return np.mean(acc_g, 0), np.mean(acc_f, 0), np.ptp(tx_inst)


print("=" * 74)
print(" flow-proximity feasibility probe — WALL (directional / which-side signal)")
print("=" * 74)
print(f" flyer hovers LEVEL at 50 mm; wall swept in from +y. wingtip reach |y|={TIP_Y*1e3:.1f} mm")
print(f" R={R*1e3:.2f} mm, weight={WEIGHT*1e6:.1f} uN, roll authority={ROLL_AUTH*1e9:.0f} uNmm/unit")
print(f" {'wall y':>8} {'gap':>7} {'gap/R':>6} {'dTx(roll)':>11} {'dFy(lat)':>10} {'dTz(yaw)':>10} {'roll-cmd eq':>11}")
walls_mm = np.array([15, 16, 18, 20, 24, 28, 34, 42, 55, 70])
rows = []
for yw in walls_mm:
    surf = dict(axis=1, sign=-1, pos=yw / 1e3)            # wall at +y, flyer on -y side
    Wg, Wf, _ = measure(surf)
    dFy = Wg[1] - Wf[1]; dTx = Wg[3] - Wf[3]; dTz = Wg[5] - Wf[5]
    gap = yw / 1e3 - TIP_Y
    roll_cmd = dTx / ROLL_AUTH                            # equivalent roll command to cancel it
    rows.append((yw, gap * 1e3, gap / R, dTx, dFy, dTz, roll_cmd))
    print(f" {yw:6d}mm {gap*1e3:6.1f}mm {gap/R:5.2f} {dTx*1e9:9.2f}uNmm {dFy*1e6:8.2f}uN {dTz*1e9:8.2f}uNmm {roll_cmd:10.4f}")

# directionality: wall on -y should flip dTx sign
surf_p = dict(axis=1, sign=-1, pos=0.020); Wp, Wf, _ = measure(surf_p)
surf_n = dict(axis=1, sign=+1, pos=-0.020); Wn, _, _ = measure(surf_n)
dTx_p = Wp[3] - Wf[3]; dTx_n = Wn[3] - Wf[3]
print("-" * 74)
print(f" DIRECTIONALITY (wall at 20 mm): +y wall dTx={dTx_p*1e9:+.2f}, -y wall dTx={dTx_n*1e9:+.2f} uNmm"
      f"  -> sign flips: {np.sign(dTx_p)!=np.sign(dTx_n)}")
THR = 0.01                                               # 1% roll-command-equiv = comfortably readable
detect = [r for r in rows if abs(r[6]) >= THR]
rng = max(r[1] for r in detect) if detect else 0
print(f" -> directional roll signal >= {THR*100:.0f}% roll-command-equivalent out to a {rng:.0f} mm gap"
      f" ({rng/1e3/R:.1f} wing-lengths)")
print(f" NOTE: magnitude is model-dependent (endplate/reflection model); the side-dependent")
print(f"       SIGN is the robust result. Real wall may add a lateral suction not modelled here.")
print("=" * 74)

with open(OUT / "e13_wall_effect.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["wall_y_mm", "gap_mm", "gap_over_R", "dTx_roll_uNmm", "dFy_lat_uN", "dTz_yaw_uNmm", "roll_cmd_equiv"])
    for r in rows:
        w.writerow([r[0], f"{r[1]:.3f}", f"{r[2]:.3f}", f"{r[3]*1e9:.4f}", f"{r[4]*1e6:.4f}",
                    f"{r[5]*1e9:.4f}", f"{r[6]:.5f}"])

gap = np.array([r[1] for r in rows]); dtx = np.array([r[3] for r in rows]) * 1e9
rce = np.abs([r[6] for r in rows]) * 100
fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
ax[0].plot(gap, dtx, "o-", color="tab:purple"); ax[0].axhline(0, color="k", lw=0.5)
ax[0].set_xlabel("wall gap to wingtip (mm)"); ax[0].set_ylabel("ΔTx  roll torque (µN·mm)")
ax[0].set_title("Directional wall signal: roll torque vs gap"); ax[0].grid(alpha=0.3)
ax[1].semilogy(gap, rce, "o-", color="tab:purple")
ax[1].axhline(1.0, color="tab:red", ls="--", lw=1, label="1% of roll authority (readable)")
ax[1].set_xlabel("wall gap to wingtip (mm)"); ax[1].set_ylabel("roll-command equivalent (% authority)")
ax[1].set_title("Wall disturbance the controller would feel"); ax[1].grid(alpha=0.3, which="both"); ax[1].legend()
plt.tight_layout(); fig.savefig(OUT / "e13_wall_effect.png", dpi=130)
print(f"saved: {OUT/'e13_wall_effect.csv'}")
print(f"saved: {OUT/'e13_wall_effect.png'}")