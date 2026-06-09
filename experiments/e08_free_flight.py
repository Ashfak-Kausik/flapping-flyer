"""
e08_free_flight.py — STAGE 3: open-loop free flight & instability.

Cut the thorax loose (free joint) and let it fly open-loop on the validated
aero, with NO control. Prediction: it does not hover — it diverges in attitude
(pitch first), the passive instability of a flapping flyer. We log attitude and
altitude, identify which mode diverges first, and time it. This instability is
what Stage 4's controller will exist to tame.
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

fly = Flyer(ROOT / "models" / "flyer.xml")
kin = FlapKinematics(f_hz=80, stroke_amp_deg=90, feather_amp_deg=45, rot_phase=0.0)
fly.reset(kin=kin, height=0.05)

T_END = 0.10
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
print("="*56)
print(" STAGE 3 — open-loop free flight")
print("="*56)
print(f" flapping: 80 Hz, 90 deg stroke, no control")
print(f" simulated {T_END*1e3:.0f} ms ({n} steps), finite = {np.all(np.isfinite([pitch,roll,h]))}")
print("-"*56)
print(f" time to |pitch| > {THRESH:.0f} deg : {tp*1e3:.1f} ms" if tp else " pitch stayed small")
print(f" time to |roll|  > {THRESH:.0f} deg : {tr*1e3:.1f} ms" if tr else " roll stayed small")
mode = "PITCH" if (tp and (not tr or tp < tr)) else "ROLL"
print(f" -> first unstable mode: {mode}  (flapping flyers are pitch-unstable)")
print(f" peak |pitch| reached: {np.abs(pitch).max():.0f} deg")
print("="*56)

with open(OUT/"e08_free_flight.csv","w",newline="") as fh:
    w = csv.writer(fh); w.writerow(["t_ms","pitch_deg","roll_deg","height_mm"])
    for i in range(n):
        w.writerow([f"{t[i]*1e3:.4f}", f"{pitch[i]:.4f}", f"{roll[i]:.4f}", f"{h[i]:.4f}"])

fig, ax = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
ax[0].plot(t*1e3, pitch, label="pitch", color="tab:red")
ax[0].plot(t*1e3, roll, label="roll", color="tab:blue")
ax[0].axhline(0, color="k", lw=0.5)
for thr in (THRESH, -THRESH):
    ax[0].axhline(thr, color="gray", ls=":", lw=0.8)
if tp: ax[0].axvline(tp*1e3, color="tab:red", ls="--", lw=0.8, alpha=0.6)
ax[0].set_ylabel("attitude (deg)")
ax[0].set_title("Open-loop free flight: passive instability (pitch diverges first)")
ax[0].legend(loc="upper left")
ax[1].plot(t*1e3, h, color="tab:green", label="height")
ax[1].axhline(50, color="gray", ls=":", lw=0.8, label="start height")
ax[1].set_xlabel("time (ms)"); ax[1].set_ylabel("height (mm)")
ax[1].legend(loc="upper left")
plt.tight_layout(); fig.savefig(OUT/"e08_free_flight.png", dpi=130)
print(f"\nsaved: {OUT/'e08_free_flight.csv'}")
print(f"saved: {OUT/'e08_free_flight.png'}")