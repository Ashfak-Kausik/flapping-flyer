"""
e05_rotational_lift.py — STAGE 2 CHECK (piece 4): rotational (Kramer) lift.

Two things to verify:
  (A) NO mirror cancellation — both wings' rotational lift must share a sign
      (this is the trap that bit translational lift and the old prototype).
  (B) ROTATION TIMING gates it — with symmetric feather (flip exactly at
      stroke reversal, where speed ~ 0) the cycle-averaged rotational lift is
      ~0; advancing the rotation (feather leads the reversal) switches it on.
      That phase is the control knob we will use later.
Saves the phase sweep (CSV) and figures (PNG).
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
from src.aero import strips_for_wing, rotational_force

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "flyer.xml"))
data = mujoco.MjData(model)
DT = model.opt.timestep
WEIGHT = float(model.body_mass.sum()) * 9.81
wings = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"wing_{s}") for s in "RL"}
strips = {s: strips_for_wing(model, s, 20) for s in "RL"}
print(f"C_rot (from geometry) = {strips['R']['c_rot']:.4f}   (pi/2 = {np.pi/2:.4f})")

def jadr(n):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
    return model.jnt_qposadr[j], model.jnt_dofadr[j]
J = {n: jadr(n) for n in ("stroke_R","pitch_R","stroke_L","pitch_L")}
PHI = np.deg2rad(60); PSI = np.deg2rad(45); F_HZ = 40.0; W = 2*np.pi*F_HZ

def drive(t, d):
    s  =  PHI*np.cos(W*t);  ds = -PHI*W*np.sin(W*t)
    ps = -PSI*np.tanh(3*np.sin(W*t + d))
    dps= -PSI*3*W*np.cos(W*t + d)*(1 - np.tanh(3*np.sin(W*t + d))**2)
    return s, ds, ps, dps

def set_state(t, d):
    s, ds, ps, dps = drive(t, d)
    data.qpos[J["stroke_R"][0]] =  s; data.qvel[J["stroke_R"][1]] =  ds
    data.qpos[J["stroke_L"][0]] = -s; data.qvel[J["stroke_L"][1]] = -ds
    data.qpos[J["pitch_R"][0]]  = ps; data.qvel[J["pitch_R"][1]]  = dps
    data.qpos[J["pitch_L"][0]]  = ps; data.qvel[J["pitch_L"][1]]  = dps
    mujoco.mj_forward(model, data)

# ---------- (A) per-wing sign check, at a phase where rotation is active ----------
print("\n(A) per-wing rotational Fz (advanced rotation, mid-stroke):")
set_state(t=0.30/F_HZ, d=0.6)
fz = {}
for s_lr, bid in wings.items():
    F, _, _ = rotational_force(model, data, bid, strips[s_lr]); fz[s_lr] = F[2]
    print(f"    wing {s_lr}: Fz_rot = {F[2]*1e6:+8.2f} uN")
agree = np.sign(fz["R"]) == np.sign(fz["L"]) and abs(fz["R"]) > 1e-12
print(f"    -> wings {'AGREE (no cancellation)' if agree else 'CANCEL — bug!'}")

# ---------- (B) rotation-timing sweep ----------
def cycle_avg_rot(d):
    mujoco.mj_resetData(model, data)
    T_END = 4.0/F_HZ; n = int(T_END/DT); ts = np.empty(n); Fz = np.empty(n)
    for i in range(n):
        t = i*DT; set_state(t, d)
        f = sum(rotational_force(model, data, bid, strips[s])[0][2]
                for s, bid in wings.items())
        ts[i] = t; Fz[i] = f
    m = ts > (T_END - 2.0/F_HZ)
    return ts, Fz, Fz[m].mean()

print("\n(B) rotation-timing sweep (cycle-averaged rotational lift):")
print("   phase (rad) | mean rot lift (uN) | (% of weight)")
phases = [0.0, 0.2, 0.4, 0.6, 0.8]
rows = []
for d in phases:
    _, _, avg = cycle_avg_rot(d); rows.append((d, avg))
    print(f"      {d:4.1f}     |     {avg*1e6:8.2f}       |   {avg/WEIGHT*100:+5.1f}%")

with open(OUT/"e05_rotation_timing.csv","w",newline="") as fh:
    w = csv.writer(fh); w.writerow(["phase_rad","mean_rot_lift_uN","pct_weight"])
    for d, a in rows: w.writerow([d, f"{a*1e6:.3f}", f"{a/WEIGHT*100:.2f}"])

# figures: rotational Fz(t) symmetric vs advanced; and mean vs phase
fig, ax = plt.subplots(1, 2, figsize=(12, 4))
for d, lab in [(0.0, "symmetric (phase 0)"), (0.6, "advanced (phase 0.6)")]:
    ts, Fz, avg = cycle_avg_rot(d); m = ts > (4.0/F_HZ - 1.0/F_HZ)
    ax[0].plot((ts[m]-ts[m][0])*1e3, Fz[m]*1e6, label=f"{lab}, mean={avg*1e6:.0f} uN")
ax[0].axhline(0, color="k", lw=0.5)
ax[0].set_xlabel("time (ms)"); ax[0].set_ylabel("rotational lift (uN)")
ax[0].set_title("Rotational lift over a cycle"); ax[0].legend(fontsize=8)
pp = np.array([d for d, _ in rows]); mm = np.array([a for _, a in rows])*1e6
ax[1].plot(pp, mm, "o-")
ax[1].axhline(0, color="k", lw=0.5)
ax[1].set_xlabel("rotation-timing phase (rad)"); ax[1].set_ylabel("mean rotational lift (uN)")
ax[1].set_title("Rotation timing gates rotational lift (the control knob)")
plt.tight_layout(); fig.savefig(OUT/"e05_rotational_lift.png", dpi=130)
print(f"\nsaved: {OUT/'e05_rotation_timing.csv'}")
print(f"saved: {OUT/'e05_rotational_lift.png'}")