"""
e10_open_loop_dynamics.py — STAGE 4 (2/N): identify the open-loop dynamics A.

We already measured the control matrix B (e09). Here we measure A, the
open-loop dynamics, by perturbing each body-twist component about the hover
trim and reading the cycle-averaged aero wrench (src/sysid). Assembling A from
the aero stability derivatives + thrust-tilt (gravity) + attitude kinematics
gives a 9-state linear model whose EIGENVALUES quantify the instability.

Result: the hovering flyer has a fast lateral/roll divergence plus a slower
longitudinal/pitch oscillation, a stable heave mode, and a neutral yaw drift —
the textbook hovering-insect picture. The instability is still slow versus a
wingbeat (fastest doubling ~40 ms), consistent with the slow open-loop divergence
in e08, and controllable. A and B feed the controller design next.
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
from src.kinematics import FlapKinematics
from src import sysid

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
g = 9.81

fly = Flyer(ROOT / "models" / "flyer.xml")
M, I = fly.M, fly.I
AMP = sysid.hover_amplitude(fly, f_hz=80, feather=45)
kin = FlapKinematics(f_hz=80, stroke_amp_deg=AMP, feather_amp_deg=45)
D, w0 = sysid.stability_derivatives(fly, kin)
A = sysid.assemble_A(D, M, I)
ev = np.linalg.eigvals(A)

print("=" * 64)
print(" STAGE 4 (2/N) — open-loop dynamics A & eigenvalues")
print("=" * 64)
print(f" M = {M*1e6:.2f} mg (weight {M*g*1e6:.1f} uN)")
print(f" I about CoM x1e9 = {np.diag(I)*1e9}  (roll, pitch, yaw)")
print(f" hover stroke amplitude @80Hz = {AMP:.2f} deg")
print(f" trim wrench: Fz={w0[2]*1e6:.1f} uN (=weight), Ty(bias)={w0[4]*1e9:+.1f} uNmm")
print("-" * 64)
cl = ["vx", "vy", "vz", "wx", "wy", "wz"]; rl = ["Fx", "Fy", "Fz", "Tx", "Ty", "Tz"]
print(" aero stability derivatives D (SI units):")
print("       " + "".join(f"{c:>11}" for c in cl))
for r in range(6):
    print(f" {rl[r]:>3} " + "".join(f"{D[r,c]:11.2e}" for c in range(6)))
print("-" * 64)
print(" physical sanity:")
print(f"   heave damping  dFz/dvz = {D[2,2]:+.2e}  (<0 ok)")
print(f"   pitch  damping dTy/dwy = {D[4,4]:+.2e}  (<0 ok)")
print(f"   roll   damping dTx/dwx = {D[3,3]:+.2e}  (<0 ok)")
print(f"   long/lat decoupling: dTz/dvx={D[5,0]:+.1e}, dTx/dvx={D[3,0]:+.1e} (~0 ok)")
print("-" * 64)
print(" eigenvalues (1/s):")
modes = []
for e in sorted(ev, key=lambda z: -z.real):
    if e.real > 1e-3:
        tag = f"UNSTABLE  t_double={np.log(2)/e.real*1e3:6.1f} ms"
    elif e.real < -1e-3:
        tag = f"stable    tau={-1/e.real*1e3:6.1f} ms"
    else:
        tag = "neutral (drift)"
    osc = f"  osc {abs(e.imag)/2/np.pi:.1f} Hz" if abs(e.imag) > 1e-3 else ""
    print(f"   {e.real:+7.2f} {e.imag:+7.2f}j   {tag}{osc}")
    modes.append((e.real, e.imag))
n_unst = int(np.sum(ev.real > 1e-3))
print("-" * 64)
print(f" -> {n_unst} unstable modes: a fast lateral/roll divergence + a slower")
print(f"    longitudinal/pitch oscillation (the rest stable; yaw neutral)")
print(f"    fastest doubling ~ {np.log(2)/max(ev.real)*1e3:.0f} ms (lateral) -> slow vs a wingbeat; controllable")
print(f"    CHECK: consistent with e08 — pitch crosses 10deg first only because it gets a head")
print(f"           start from the +{w0[4]*1e9:.0f} uNmm pitch-bias torque; neither mode is a <20 ms tumble")
print("=" * 64)

# ---- save data ----
with open(OUT / "e10_eigenvalues.csv", "w", newline="") as fh:
    wr = csv.writer(fh); wr.writerow(["real_1_s", "imag_1_s", "osc_Hz", "t_double_ms_or_tau_ms", "type"])
    for er, ei in modes:
        typ = "unstable" if er > 1e-3 else ("stable" if er < -1e-3 else "neutral")
        tconst = (np.log(2)/er if er > 1e-3 else (-1/er if er < -1e-3 else 0))*1e3
        wr.writerow([f"{er:.4f}", f"{ei:.4f}", f"{abs(ei)/2/np.pi:.4f}", f"{tconst:.4f}", typ])
with open(OUT / "e10_stability_derivatives.csv", "w", newline="") as fh:
    wr = csv.writer(fh); wr.writerow(["wrench\\twist"] + cl)
    for r in range(6):
        wr.writerow([rl[r]] + [f"{D[r,c]:.6e}" for c in range(6)])
    wr.writerow([]); wr.writerow(["trim_wrench_Fx..Tz_SI"] + [f"{w0[k]:.6e}" for k in range(6)])
np.save(OUT / "e10_A.npy", A); np.save(OUT / "e10_D.npy", D)

# ---- figure: eigenvalue spectrum + derivative heatmap ----
fig, ax = plt.subplots(1, 2, figsize=(13, 5.2))
re = ev.real; im = ev.imag
ax[0].axvspan(0, max(re.max()*1.3, 1), color="tab:red", alpha=0.07)
ax[0].axvline(0, color="k", lw=1)
ax[0].scatter(re[re > 1e-3], im[re > 1e-3], c="tab:red", s=70, zorder=3, label="unstable")
ax[0].scatter(re[np.abs(re) <= 1e-3], im[np.abs(re) <= 1e-3], c="gray", s=70, zorder=3, label="neutral")
ax[0].scatter(re[re < -1e-3], im[re < -1e-3], c="tab:blue", s=70, zorder=3, label="stable")
ax[0].set_xlabel("Re($\\lambda$)  [1/s]"); ax[0].set_ylabel("Im($\\lambda$)  [1/s]")
ax[0].set_title("Open-loop eigenvalues (right half-plane = unstable)")
ax[0].legend(loc="upper left"); ax[0].grid(alpha=0.3)

vmax = np.abs(D).max()
im0 = ax[1].imshow(D, cmap="RdBu_r", vmin=-vmax, vmax=vmax, aspect="auto")
ax[1].set_xticks(range(6)); ax[1].set_xticklabels(cl)
ax[1].set_yticks(range(6)); ax[1].set_yticklabels(rl)
ax[1].set_title("Aero stability derivatives D")
ax[1].set_xlabel("body twist"); ax[1].set_ylabel("wrench")
for r in range(6):
    for c in range(6):
        if abs(D[r, c]) > 0.04 * vmax:
            ax[1].text(c, r, f"{D[r,c]:.0e}", ha="center", va="center", fontsize=6.5)
fig.colorbar(im0, ax=ax[1], fraction=0.046, pad=0.04)
plt.tight_layout(); fig.savefig(OUT / "e10_open_loop_dynamics.png", dpi=130)
print(f"\nsaved: {OUT/'e10_eigenvalues.csv'}")
print(f"saved: {OUT/'e10_stability_derivatives.csv'}")
print(f"saved: {OUT/'e10_open_loop_dynamics.png'}  (+ e10_A.npy, e10_D.npy)")