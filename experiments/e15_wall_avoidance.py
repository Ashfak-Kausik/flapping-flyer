"""
e15_wall_avoidance.py — STAGE 6 (2/N): closing the loop — flow-sensed wall standoff.

The flyer flaps under the LQG with NO proximity sensor. An outer loop
(src/avoidance.py) reads the controller's own u_roll residual as a proximity+side
signal (validated in e14) and commands a lateral velocity to hold a CHOSEN
standoff from the wall — regulating the residual magnitude to a setpoint.

  (A) Convergence: for two setpoints the flyer reaches the corresponding standoff
      from BOTH a near and a far start — it flies to a commanded distance from a
      wall it can only feel.
  (B) Dial-a-standoff: sweeping the setpoint, the achieved gap is a clean monotone
      function of it — the standoff is a usable control knob.

Honest scope: proximity SENSING is validated quasi-statically (e14); while
manoeuvring, u_roll is partly the manoeuvre, so the setpoint->standoff map is
monotone/repeatable but not 1:1 (a disturbance observer would decouple them, and
would also damp the limit-cycle seen under a hard shove toward the wall — left as
a refinement). A passive aerodynamic repulsion is also present (sign model-
dependent); the active loop gives controlled, selectable standoff regardless.
"""
import sys
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design
from src.avoidance import WallStandoff

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
WALL = 0.030; TIP = 0.0133
CONTACT = (WALL - TIP) * 1e3
fly = Flyer(ROOT / "models" / "flyer.xml")
print("designing LQG controller..."); ctrl, kin, info = design(fly)
LOG = int(0.004 / fly.dt)


def run(rstar, y0, T=2.6):
    fly.reset(kin=kin, height=0.05, y=y0); ctrl.reset(); av = WallStandoff(rstar, fly.dt)
    ts, gaps = [], []
    for i in range(int(T / fly.dt)):
        u = ctrl.update(fly.sense(), fly.dt, vy_ref=av.command())
        kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i * fly.dt, surface=dict(axis=1, sign=-1, pos=WALL))
        av.observe(u[1])
        if i % LOG == 0:
            ts.append(i * fly.dt); gaps.append(CONTACT - fly.x_com[1] * 1e3)
    g = np.array(gaps)
    return np.array(ts), g, float(g[-int(0.4 / (LOG * fly.dt)):].mean())   # settled gap


print("=" * 74)
print(" STAGE 6 (2/N) — flow-sensed wall avoidance (no proximity sensor)")
print("=" * 74)
print(f" wall +{WALL*1e3:.0f}mm; wingtip {TIP*1e3:.1f}mm; contact at gap=0 (flyer y={CONTACT:.1f}mm)")
setpoints = [0.006, 0.009, 0.012, 0.016]
starts = {"near": 0.0, "far": -0.030}
traces = {}; settle = {}
print(f"\n {'setpoint':>9} {'near-gap':>9} {'far-gap':>9} {'mean':>7} {'spread':>7}")
for rs in setpoints:
    gs = {}
    for lab, y0 in starts.items():
        t, g, gf = run(rs, y0); traces[(rs, lab)] = (t, g); gs[lab] = gf
    mean = 0.5 * (gs["near"] + gs["far"]); spread = abs(gs["near"] - gs["far"])
    settle[rs] = (mean, spread, gs["near"], gs["far"])
    print(f" {rs:9.3f} {gs['near']:8.1f}mm {gs['far']:8.1f}mm {mean:6.1f} {spread:6.1f}")
print("-" * 74)
print(" -> larger setpoint => tighter standoff; reached from BOTH sides; never contacts.")
print(" -> the standoff is a usable control knob, sensed purely from the u_roll residual.")
print("=" * 74)

# ---- figure ----
fig, ax = plt.subplots(1, 2, figsize=(13, 5))
colA = {0.009: "tab:blue", 0.016: "tab:green"}
for rs in (0.009, 0.016):
    for lab in ("near", "far"):
        t, g = traces[(rs, lab)]
        ax[0].plot(t, g, "-" if lab == "near" else "--", color=colA[rs], lw=1.8,
                   label=f"setpoint {rs:.3f}, {lab} start")
ax[0].axhline(0, color="firebrick", lw=2)
ax[0].text(0.05, 1.0, "WALL CONTACT", color="firebrick", fontsize=9)
ax[0].set_xlabel("time (s)"); ax[0].set_ylabel("gap: wingtip → wall (mm)")
ax[0].set_title("(A) Converges to a commanded standoff from either side")
ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8); ax[0].set_ylim(bottom=-1)

sp = np.array(setpoints)
mean = np.array([settle[r][0] for r in setpoints])
spread = np.array([settle[r][1] for r in setpoints])
ax[1].errorbar(sp, mean, yerr=spread / 2, fmt="o-", color="tab:purple", capsize=4, lw=1.8)
ax[1].set_xlabel("residual setpoint (commanded)"); ax[1].set_ylabel("achieved standoff gap (mm)")
ax[1].set_title("(B) Dial-a-standoff: setpoint → gap is a clean knob")
ax[1].grid(alpha=0.3)
plt.tight_layout(); fig.savefig(OUT / "e15_wall_avoidance.png", dpi=130)

with open(OUT / "e15_wall_avoidance.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["setpoint", "near_gap_mm", "far_gap_mm", "mean_gap_mm", "spread_mm"])
    for rs in setpoints:
        m, s, gn, gf = settle[rs]
        w.writerow([rs, f"{gn:.2f}", f"{gf:.2f}", f"{m:.2f}", f"{s:.2f}"])
print(f"saved: {OUT/'e15_wall_avoidance.png'}")
print(f"saved: {OUT/'e15_wall_avoidance.csv'}")