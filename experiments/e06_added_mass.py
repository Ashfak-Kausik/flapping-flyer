"""
e06_added_mass.py — STAGE 2 CHECK (piece 5): added mass.

Added mass is a reaction to acceleration, so its signature is:
  (A) both wings AGREE in sign (no mirror cancellation),
  (B) it is LARGE near each stroke reversal (max |acceleration|) and
      AVERAGES to ~zero over a full cycle (a conservative reaction).
Saves the trace (CSV) and figure (PNG).
"""
import sys
from pathlib import Path
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.aero import strips_for_wing, added_mass_force

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "flyer.xml"))
data = mujoco.MjData(model)
DT = model.opt.timestep
WEIGHT = float(model.body_mass.sum()) * 9.81
wings = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"wing_{s}") for s in "RL"}
strips = {s: strips_for_wing(model, s, 20) for s in "RL"}

def jadr(n):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
    return model.jnt_qposadr[j], model.jnt_dofadr[j]
J = {n: jadr(n) for n in ("stroke_R","pitch_R","stroke_L","pitch_L")}
PHI = np.deg2rad(60); PSI = np.deg2rad(45); F_HZ = 40.0; W = 2*np.pi*F_HZ

def set_state(t):
    s  =  PHI*np.cos(W*t);  ds = -PHI*W*np.sin(W*t)
    ps = -PSI*np.tanh(3*np.sin(W*t))
    dps= -PSI*3*W*np.cos(W*t)*(1-np.tanh(3*np.sin(W*t))**2)
    data.qpos[J["stroke_R"][0]] =  s; data.qvel[J["stroke_R"][1]] =  ds
    data.qpos[J["stroke_L"][0]] = -s; data.qvel[J["stroke_L"][1]] = -ds
    data.qpos[J["pitch_R"][0]]  = ps; data.qvel[J["pitch_R"][1]]  = dps
    data.qpos[J["pitch_L"][0]]  = ps; data.qvel[J["pitch_L"][1]]  = dps
    mujoco.mj_forward(model, data)
    return np.rad2deg(s), np.rad2deg(ps)

# run several cycles, threading vn_prev per wing
mujoco.mj_resetData(model, data)
T_END = 4.0/F_HZ; n = int(T_END/DT)
vn_prev = {s: None for s in "RL"}
ts = np.empty(n); Fz_R = np.empty(n); Fz_L = np.empty(n); stroke_log = np.empty(n)
for i in range(n):
    t = i*DT; strk, _ = set_state(t)
    for s_lr, bid in wings.items():
        F, _, vn, _ = added_mass_force(model, data, bid, strips[s_lr], vn_prev[s_lr], DT)
        vn_prev[s_lr] = vn
        if s_lr == "R": Fz_R[i] = F[2]
        else: Fz_L[i] = F[2]
    ts[i] = t; stroke_log[i] = strk

Fz = Fz_R + Fz_L
m = ts > (T_END - 2.0/F_HZ)           # steady-state window (skip startup transient)
mean = Fz[m].mean(); rms = np.sqrt((Fz[m]**2).mean()); peak = np.abs(Fz[m]).max()

# (A) per-wing agreement: correlation of the two wing traces over steady window
corr = np.corrcoef(Fz_R[m], Fz_L[m])[0, 1]
print("(A) per-wing added-mass agreement:")
print(f"    corr(Fz_R, Fz_L) = {corr:+.3f}   ({'AGREE' if corr > 0.9 else 'DISAGREE — bug!'})")

print("\n(B) added-mass character over a cycle (steady state):")
print(f"    mean  = {mean*1e6:+8.2f} uN   ({mean/WEIGHT*100:+.2f}% of weight)")
print(f"    rms   = {rms*1e6:8.2f} uN   ({rms/WEIGHT*100:.2f}% of weight)")
print(f"    peak  = {peak*1e6:8.2f} uN")
print(f"    -> mean/rms = {abs(mean)/rms:.3f}  "
      f"({'~0 mean, conservative as expected' if abs(mean)/rms < 0.2 else 'mean not small'})")

with open(OUT/"e06_added_mass_trace.csv","w",newline="") as fh:
    w = csv.writer(fh); w.writerow(["t_s","stroke_deg","Fz_addedmass_uN"])
    for i in range(n):
        if m[i]: w.writerow([f"{ts[i]:.5f}", f"{stroke_log[i]:.3f}", f"{Fz[i]*1e6:.4f}"])

# figure: added-mass Fz vs stroke angle (peaks at reversals where stroke is extremal)
fig, ax = plt.subplots(figsize=(9, 4))
tt = (ts[m]-ts[m][0])*1e3
ax.plot(tt, Fz[m]*1e6, color="tab:green", label="added-mass lift")
ax.axhline(0, color="k", lw=0.5)
ax.axhline(mean*1e6, color="r", ls=":", label=f"mean = {mean*1e6:.0f} uN")
ax2 = ax.twinx()
ax2.plot(tt, stroke_log[m], color="tab:gray", lw=1, alpha=0.6, label="stroke angle")
ax2.set_ylabel("stroke angle (deg)", color="tab:gray")
ax.set_xlabel("time (ms)"); ax.set_ylabel("added-mass vertical force (uN)")
ax.set_title("Added mass: spikes at stroke reversals, ~zero mean")
ax.legend(loc="upper left", fontsize=8)
plt.tight_layout(); fig.savefig(OUT/"e06_added_mass.png", dpi=130)
print(f"\nsaved: {OUT/'e06_added_mass_trace.csv'}")
print(f"saved: {OUT/'e06_added_mass.png'}")