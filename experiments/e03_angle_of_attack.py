"""
e03_angle_of_attack.py — STAGE 2 CHECK (piece 2): strip velocity & AoA.

This is the frame-sensitive piece, so we validate the angle of attack against
three cases we can predict BY HAND, before trusting it for any force:

  With the wing pointing sideways (stroke = 0) and given a stroke velocity, the
  strip moves through the air along a fixed world direction. Feathering the wing
  by an angle theta rotates the chord by theta relative to that flow, so the
  angle of attack MUST equal theta:
        pitch  0 deg -> wing slices edge-on   -> alpha ~  0 deg
        pitch 45 deg ->                         -> alpha ~ 45 deg
        pitch 90 deg -> wing moves face-on     -> alpha ~ 90 deg

Then we drive a slow flap and log alpha(t) for a mid-span strip, saving the
trace (CSV) and figure (PNG).
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
from src.aero import wing_params_from_model, build_strips, strip_kinematics

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)

model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "flyer.xml"))
data = mujoco.MjData(model)
WING = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "wing_R")
p = wing_params_from_model(model)
s = build_strips(p["c_max"], p["y_center"], p["span_semi"], 20)
r_mid = p["y_center"]            # a representative mid-span station

def jadr(name):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
    return model.jnt_qposadr[j], model.jnt_dofadr[j]
qs, ds = jadr("stroke_R")
qp, dp = jadr("pitch_R")

# ---------- Part A: the three known-angle checks ----------
print("="*56)
print(" AoA validation — known cases (expect alpha == pitch)")
print("="*56)
OMEGA_STROKE = 50.0            # rad/s, an arbitrary nonzero stroke rate
all_ok = True
for theta_deg in (0.0, 45.0, 90.0):
    mujoco.mj_resetData(model, data)
    data.qpos[qs] = 0.0                       # wing points sideways
    data.qpos[qp] = np.deg2rad(theta_deg)     # feather angle
    data.qvel[ds] = OMEGA_STROKE              # give it a stroke velocity
    data.qvel[dp] = 0.0
    mujoco.mj_forward(model, data)
    k = strip_kinematics(model, data, WING, r_mid)
    a = np.rad2deg(k["alpha"][0])
    sp = k["speed"][0]
    sp_expected = OMEGA_STROKE * r_mid        # |v| = omega * (dist from stroke axis)
    ok_a = abs(a - theta_deg) < 0.5
    ok_s = abs(sp - sp_expected) < 1e-4
    all_ok &= ok_a and ok_s
    print(f"  pitch = {theta_deg:5.1f} deg  ->  alpha = {a:6.2f} deg  "
          f"[{'PASS' if ok_a else 'FAIL'}]   "
          f"speed = {sp*1e3:6.1f} mm/s (expect {sp_expected*1e3:.1f}) "
          f"[{'PASS' if ok_s else 'FAIL'}]")
print("="*56)
print(f"  RESULT: {'ALL PASS — frames are correct' if all_ok else 'FAIL — frame/sign bug'}")
print("="*56)

# ---------- Part B: alpha(t) through a slow prescribed flap ----------
F = 5.0                          # slow flap, Hz (just to generate motion)
W = 2*np.pi*F
PHI = np.deg2rad(60)             # stroke amplitude
PSI = np.deg2rad(45)            # feather amplitude
DT = model.opt.timestep
def stroke(t):  return PHI*np.cos(W*t)
def dstroke(t): return -PHI*W*np.sin(W*t)
def pitch(t):   return PSI*np.tanh(3*np.sin(W*t))
def dpitch(t):  return PSI*3*W*np.cos(W*t)*(1-np.tanh(3*np.sin(W*t))**2)

mujoco.mj_resetData(model, data)
T_END = 2/F                       # two flap cycles
n = int(T_END/DT)
log = []
for i in range(n):
    t = i*DT
    data.qpos[qs] = stroke(t);  data.qvel[ds] = dstroke(t)
    data.qpos[qp] = pitch(t);   data.qvel[dp] = dpitch(t)
    mujoco.mj_forward(model, data)
    k = strip_kinematics(model, data, WING, r_mid)
    log.append((t, np.rad2deg(stroke(t)), np.rad2deg(pitch(t)),
                np.rad2deg(k["alpha"][0]), k["speed"][0]))
log = np.array(log)

with open(OUT / "e03_aoa_trace.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["t_s", "stroke_deg", "pitch_deg", "alpha_deg", "speed_mps"])
    for row in log:
        w.writerow([f"{row[0]:.5f}", f"{row[1]:.3f}", f"{row[2]:.3f}",
                    f"{row[3]:.3f}", f"{row[4]:.5f}"])

fig, ax = plt.subplots(2, 1, figsize=(9, 5), sharex=True)
ax[0].plot(log[:,0]*1e3, log[:,1], label="stroke angle")
ax[0].plot(log[:,0]*1e3, log[:,2], label="pitch (feather) angle")
ax[0].set_ylabel("deg"); ax[0].legend(loc="upper right")
ax[0].set_title("Mid-span strip: prescribed flap and resulting angle of attack")
ax[1].plot(log[:,0]*1e3, log[:,3], color="tab:red", label="angle of attack")
ax[1].set_ylabel("AoA (deg)"); ax[1].set_xlabel("time (ms)")
ax[1].legend(loc="upper right")
plt.tight_layout(); fig.savefig(OUT / "e03_aoa_trace.png", dpi=130)
print(f"\nsaved: {OUT/'e03_aoa_trace.csv'}")
print(f"saved: {OUT/'e03_aoa_trace.png'}")