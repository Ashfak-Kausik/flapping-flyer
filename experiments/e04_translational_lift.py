"""
e04_translational_lift.py — STAGE 2 CHECK (piece 3): translational lift.

Drives a flapping motion (body clamped, kinematics prescribed) and checks the
two predictions the theory makes:
  (A) cycle-averaged vertical force is POSITIVE (up) and comparable to weight;
  (B) lift scales with the SQUARE of flapping frequency  (Part 5).
Saves the frequency sweep (CSV) and figures (PNG).
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
from src.aero import (wing_params_from_model, build_strips,
                      translational_force, strips_for_wing, RHO)

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "flyer.xml"))
data = mujoco.MjData(model)
DT = model.opt.timestep
WEIGHT = float(model.body_mass.sum()) * 9.81

wings = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"wing_{s}") for s in "RL"}
strips = {s: strips_for_wing(model, s, 20) for s in "RL"}   # per-wing!

def jadr(n):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
    return model.jnt_qposadr[j], model.jnt_dofadr[j]
J = {n: jadr(n) for n in ("stroke_R","pitch_R","stroke_L","pitch_L")}

PHI = np.deg2rad(60)        # stroke amplitude
PSI = np.deg2rad(45)       # feather amplitude

def drive(t, W):
    """Prescribed wing kinematics + analytic joint velocities."""
    s  =  PHI*np.cos(W*t);          ds = -PHI*W*np.sin(W*t)
    ps = -PSI*np.tanh(3*np.sin(W*t))    # feather sense -> lift UP
    dps= -PSI*3*W*np.cos(W*t)*(1-np.tanh(3*np.sin(W*t))**2)
    return s, ds, ps, dps

def cycle_avg_lift(f_hz):
    W = 2*np.pi*f_hz
    mujoco.mj_resetData(model, data)
    T_END = 4.0/f_hz                       # 4 cycles
    n = int(T_END/DT); Fz = np.empty(n); ts = np.empty(n)
    for i in range(n):
        t = i*DT
        s, ds, ps, dps = drive(t, W)
        data.qpos[J["stroke_R"][0]] =  s; data.qvel[J["stroke_R"][1]] =  ds
        data.qpos[J["stroke_L"][0]] = -s; data.qvel[J["stroke_L"][1]] = -ds
        data.qpos[J["pitch_R"][0]]  = ps; data.qvel[J["pitch_R"][1]]  = dps
        data.qpos[J["pitch_L"][0]]  = ps; data.qvel[J["pitch_L"][1]]  = dps
        mujoco.mj_forward(model, data)
        fz = 0.0
        for s_lr, bid in wings.items():
            F, _, _ = translational_force(model, data, bid, strips[s_lr])
            fz += F[2]
        Fz[i] = fz; ts[i] = t
    mask = ts > (T_END - 2.0/f_hz)          # average over last 2 cycles
    return ts, Fz, Fz[mask].mean()

print(f"body weight = {WEIGHT*1e6:.1f} uN\n")
print(" freq (Hz) | avg lift (uN) | lift/weight | lift/f^2 (uN/Hz^2)")
freqs = [20, 30, 40, 50, 60]
rows = []
for f in freqs:
    _, _, avg = cycle_avg_lift(f)
    rows.append((f, avg))
    print(f"   {f:5d}   |   {avg*1e6:9.2f}  |   {avg/WEIGHT:6.3f}    |   {avg*1e6/f**2:8.4f}")

# (B) f^2 test: lift/f^2 should be ~constant
ratios = np.array([avg*1e6/f**2 for f, avg in rows])
spread = ratios.std()/ratios.mean()
print(f"\n f^2 law: lift/f^2 spread = {spread*100:.2f}%  "
      f"({'CONSTANT -> lift ~ f^2 confirmed' if spread < 0.03 else 'NOT constant'})")

# save sweep CSV
with open(OUT/"e04_lift_vs_freq.csv","w",newline="") as fh:
    w = csv.writer(fh); w.writerow(["freq_hz","avg_lift_uN","lift_over_weight"])
    for f, avg in rows: w.writerow([f, f"{avg*1e6:.3f}", f"{avg/WEIGHT:.4f}"])

# figures: instantaneous Fz over a cycle @ 40 Hz, and the f^2 fit
ts, Fz, avg = cycle_avg_lift(40)
m = ts > (4.0/40 - 1.0/40)
fig, ax = plt.subplots(1, 2, figsize=(12, 4))
ax[0].plot((ts[m]-ts[m][0])*1e3, Fz[m]*1e6, color="tab:blue", label="vertical lift")
ax[0].axhline(WEIGHT*1e6, color="k", ls="--", label="body weight")
ax[0].axhline(avg*1e6, color="r", ls=":", label=f"cycle avg ({avg/WEIGHT:.2f} W)")
ax[0].set_xlabel("time (ms)"); ax[0].set_ylabel("force (uN)")
ax[0].set_title("Instantaneous translational lift @ 40 Hz"); ax[0].legend(fontsize=8)
ff = np.array(freqs); ll = np.array([a for _, a in rows])*1e6
ax[1].plot(ff, ll, "o-", label="measured avg lift")
ax[1].plot(ff, ratios.mean()*ff**2, "k--", label="$k\\,f^2$ fit")
ax[1].set_xlabel("flapping frequency (Hz)"); ax[1].set_ylabel("avg lift (uN)")
ax[1].set_title("Lift vs frequency: the $f^2$ law"); ax[1].legend(fontsize=8)
plt.tight_layout(); fig.savefig(OUT/"e04_translational_lift.png", dpi=130)
print(f"\nsaved: {OUT/'e04_lift_vs_freq.csv'}")
print(f"saved: {OUT/'e04_translational_lift.png'}")