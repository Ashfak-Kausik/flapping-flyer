"""
e08_free_flight.py — STAGE 3: open-loop free flight & passive instability.

Cut the body loose and let it fly open-loop on the validated aero, with NO
control, starting from a small (2 deg) attitude perturbation. Prediction: it
does not hold — the perturbation grows (pitch first, then roll), the passive
instability of a hovering flapping flyer. The growth is MILD and slow (tens of
ms to ~10 deg, tumbling over ~1 s), matching the unstable eigenvalues measured
in e10 — a controller can comfortably stabilise it.

NOTE: the body is integrated as a single rigid body with the wings as massless
aero surfaces (see src/flyer.py). Driving dynamic wings on a free body injects a
large spurious inertial torque; this clean model matches the measured dynamics.
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

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)

AMP = 72.6   # ~ hover amplitude at 80 Hz (measured in e10), so this matches the linearisation
fly = Flyer(ROOT / "models" / "flyer.xml")
kin = FlapKinematics(f_hz=80, stroke_amp_deg=AMP, feather_amp_deg=45, rot_phase=0.0)
fly.reset(kin=kin, height=0.05, pitch_deg=2.0, roll_deg=2.0)   # small seed perturbation

T_END = 0.30
n = int(T_END / fly.dt)
t = np.empty(n); pitch = np.empty(n); roll = np.empty(n); h = np.empty(n)
for i in range(n):
    fly.step(kin, i * fly.dt)
    p, r, z = fly.attitude()
    t[i] = i * fly.dt; pitch[i] = np.rad2deg(p); roll[i] = np.rad2deg(r); h[i] = z * 1e3

def first_cross(sig, thresh_deg):
    idx = np.where(np.abs(sig) > thresh_deg)[0]
    return t[idx[0]] if len(idx) else None

THRESH = 10.0
tp = first_cross(pitch, THRESH); tr = first_cross(roll, THRESH)
print("=" * 56)
print(" STAGE 3 — open-loop free flight (clean rigid-body model)")
print("=" * 56)
print(f" flapping: 80 Hz, {AMP:.1f} deg stroke (~hover), no control")
print(f" seed: pitch=+2 deg, roll=+2 deg")
print(f" simulated {T_END*1e3:.0f} ms ({n} steps), finite = {np.all(np.isfinite([pitch,roll,h]))}")
print("-" * 56)
print(f" time to |pitch| > {THRESH:.0f} deg : {tp*1e3:.1f} ms" if tp else " pitch stayed small")
print(f" time to |roll|  > {THRESH:.0f} deg : {tr*1e3:.1f} ms" if tr else " roll stayed small")
mode = "PITCH" if (tp and (not tr or tp < tr)) else "ROLL"
print(f" -> first unstable mode: {mode}  (mild, slow growth — see e10 eigenvalues)")
print(f" peak |pitch| over window: {np.abs(pitch).max():.0f} deg")
print("=" * 56)

with open(OUT / "e08_free_flight.csv", "w", newline="") as fh:
    w = csv.writer(fh); w.writerow(["t_ms", "pitch_deg", "roll_deg", "height_mm"])
    for i in range(n):
        w.writerow([f"{t[i]*1e3:.4f}", f"{pitch[i]:.4f}", f"{roll[i]:.4f}", f"{h[i]:.4f}"])

fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax[0].plot(t*1e3, pitch, label="pitch", color="tab:red")
ax[0].plot(t*1e3, roll, label="roll", color="tab:blue")
ax[0].axhline(0, color="k", lw=0.5)
for thr in (THRESH, -THRESH):
    ax[0].axhline(thr, color="gray", ls=":", lw=0.8)
if tp: ax[0].axvline(tp*1e3, color="tab:red", ls="--", lw=0.8, alpha=0.6)
if tr: ax[0].axvline(tr*1e3, color="tab:blue", ls="--", lw=0.8, alpha=0.6)
ax[0].set_ylabel("attitude (deg)")
ax[0].set_title("Open-loop free flight: mild passive instability (pitch diverges first)")
ax[0].legend(loc="upper left")
ax[1].plot(t*1e3, h, color="tab:green", label="height")
ax[1].axhline(50, color="gray", ls=":", lw=0.8, label="start height")
ax[1].set_xlabel("time (ms)"); ax[1].set_ylabel("height (mm)")
ax[1].legend(loc="upper left")
plt.tight_layout(); fig.savefig(OUT / "e08_free_flight.png", dpi=130)
print(f"\nsaved: {OUT/'e08_free_flight.csv'}")
print(f"saved: {OUT/'e08_free_flight.png'}")