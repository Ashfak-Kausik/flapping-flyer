"""
e23_yaw_authority.py — ARC B (3): a yaw actuator from split-cycle flapping.

The controllability analysis (e10) found yaw uncontrollable: symmetric flapping with
amplitude/offset controls makes no net torque about the vertical axis — fatal for a
tunnel that bends. Here we add the standard flapping-MAV mechanism, SPLIT-CYCLE:
distort the stroke phase so a wing sweeps faster in one half-stroke than the other, so
its cycle-averaged fore/aft drag no longer cancels; applied oppositely on the two wings
the fore/aft forces at the wings' lateral offsets form a yaw couple.

  stroke = A*cos(xi),  xi = w*t + (K*u_yaw*mirror)*sin(w*t)   (mirror = +1 R, -1 L)

Result: clean linear sign-controllable yaw torque, ~49 rad/s^2 per unit u_yaw (a ~90
deg turn in ~0.5 s at u_yaw~0.5). Yaw is intrinsically weak vs roll (second-order drag
asymmetry, not the lift differential) — as in real insects.

Cross-coupling (honest): split-cycle also makes a ROLL torque ~5x the yaw, plus minor
pitch/thrust. Roll authority is ~60,900 rad/s^2/unit, so the roll loop cancels it with
u_roll~0.005. Couplings are real but easily absorbed — next step is to put yaw back in
the state and design the multi-input controller.
"""
import sys
from pathlib import Path
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.kinematics import FlapKinematics
from src import aero

fly = Flyer(ROOT / "models" / "flyer.xml")
kin = FlapKinematics(80.0); kin.PHI = np.deg2rad(72.63)
Izz = 7.31e-10


def cyc_wrench(uyaw, ncyc=12, navg=8):
    m, d = fly.model, fly.data; kin.set_control(yaw=uyaw)
    per = 2*np.pi/kin.W; N = int(ncyc*per/fly.dt); t0 = (ncyc-navg)*per
    vp = {s: None for s in "RL"}; F = np.zeros(3); T = np.zeros(3); n = 0
    for i in range(N):
        t = i*fly.dt
        d.qpos[fly._qf:fly._qf+3] = [0, 0, 0.05]; d.qpos[fly._qf+3:fly._qf+7] = [1, 0, 0, 0]
        d.qvel[fly._vf:fly._vf+6] = 0
        fly._prescribe_wings(kin, t); mujoco.mj_forward(m, d); f = np.zeros(3); tq = np.zeros(3)
        for s, bid in fly.wings.items():
            Fw, Tw, vp[s], _ = aero.wing_aero(m, d, bid, fly.strips[s], vp[s], fly.dt); f += Fw; tq += Tw
        if t >= t0: F += f; T += tq; n += 1
    return F/n, T/n


uys = np.linspace(-0.3, 0.3, 7)
Tz, Tx, Ty, Fz = [], [], [], []
print(" u_yaw    Tz(yaw)   Tx(roll)  Ty(pitch)     Fz   (uN.mm / uN)")
for uy in uys:
    F, T = cyc_wrench(uy); Tz.append(T[2]*1e9); Tx.append(T[0]*1e9); Ty.append(T[1]*1e9); Fz.append(F[2]*1e6)
    print(f" {uy:6.2f}{T[2]*1e9:10.1f}{T[0]*1e9:11.1f}{T[1]*1e9:11.1f}{F[2]*1e6:9.1f}")

yaw_auth = np.polyfit(uys, Tz, 1)[0]; roll_auth = np.polyfit(uys, Tx, 1)[0]
print(f"\n yaw authority   {yaw_auth:+.0f} uN.mm/unit  ->  {yaw_auth*1e-9/Izz:+.0f} rad/s^2 per unit u_yaw")
print(f" roll coupling   {roll_auth:+.0f} uN.mm/unit  ({abs(roll_auth/yaw_auth):.1f}x yaw; cancel u_roll~{abs(roll_auth)/10399:.3f})")

fig, ax = plt.subplots(1, 2, figsize=(12, 4.6))
ax[0].plot(uys, Tz, "o-", color="tab:blue", label="Tz (yaw, the control)")
ax[0].plot(uys, Tx, "s--", color="tab:red", label="Tx (roll, coupling)")
ax[0].axhline(0, color="k", lw=0.6); ax[0].set_xlabel("u_yaw (split-cycle)"); ax[0].set_ylabel("cycle-avg torque (uN.mm)")
ax[0].set_title("Split-cycle yaw torque (linear, controllable)"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
ax[1].plot(uys, Fz, "o-", color="tab:green"); ax[1].axhline(759.3, color="gray", ls="--", lw=1, label="weight")
ax[1].set_xlabel("u_yaw"); ax[1].set_ylabel("cycle-avg lift Fz (uN)")
ax[1].set_title("Lift coupling (~6% bump, trimmable)"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
plt.tight_layout(); fig.savefig(ROOT/"outputs"/"e23_yaw_authority.png", dpi=130)
print(f"saved: {ROOT/'outputs'/'e23_yaw_authority.png'}")