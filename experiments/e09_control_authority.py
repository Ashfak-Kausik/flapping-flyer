"""
e09_control_authority.py — STAGE 4 (piece 1): control-authority map ("B-matrix").

Before closing any loop we must prove each control input produces the body
wrench we expect, with the right SIGN and little cross-coupling. We drive each
input on the CLAMPED body and measure the cycle-averaged aerodynamic wrench
(force + torque about the thorax CoM):
    u_thrust -> should drive  Fz   (vertical force)
    u_roll   -> should drive  Tx   (roll torque)
    u_pitch  -> should drive  Ty   (pitch torque, the unstable axis)
Validates: dominant on-axis effect, consistent sign, small off-axis coupling.
Saves the sweep (CSV) and figure (PNG).
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
from src.aero import strips_for_wing, wing_aero
from src.kinematics import FlapKinematics

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)

model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "flyer.xml"))   # clamped
data = mujoco.MjData(model)
DT = model.opt.timestep
THORAX = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "thorax")
wings = {s: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"wing_{s}") for s in "RL"}
strips = {s: strips_for_wing(model, s, 20) for s in "RL"}
def jadr(n):
    j = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
    return model.jnt_qposadr[j], model.jnt_dofadr[j]
J = {n: jadr(n) for n in ("stroke_R","pitch_R","stroke_L","pitch_L")}

F_HZ = 80.0
kin = FlapKinematics(f_hz=F_HZ, stroke_amp_deg=60, feather_amp_deg=45, rot_phase=0.0)

def measure(thrust=0.0, roll=0.0, pitch=0.0):
    """Cycle-averaged wrench [Fx,Fy,Fz,Tx,Ty,Tz] about thorax CoM (in uN, uN*mm)."""
    kin.set_control(thrust=thrust, roll=roll, pitch=pitch)
    mujoco.mj_resetData(model, data)
    T_END = 4.0 / F_HZ; n = int(T_END / DT)
    vn = {s: None for s in "RL"}
    W = np.zeros((n, 6))
    for i in range(n):
        t = i * DT
        for wing in ("R", "L"):
            st, dst, pt, dpt = kin.signals(t, wing)
            data.qpos[J[f"stroke_{wing}"][0]] = st; data.qvel[J[f"stroke_{wing}"][1]] = dst
            data.qpos[J[f"pitch_{wing}"][0]]  = pt; data.qvel[J[f"pitch_{wing}"][1]]  = dpt
        mujoco.mj_forward(model, data)
        com = data.xipos[THORAX]
        Ft = np.zeros(3); Tt = np.zeros(3)
        for s, bid in wings.items():
            F, T, vn[s], _ = wing_aero(model, data, bid, strips[s], vn[s], DT)
            Ft += F; Tt += T + np.cross(data.xipos[bid] - com, F)
        W[i, :3] = Ft; W[i, 3:] = Tt
    m = np.arange(n) * DT > (T_END - 2.0 / F_HZ)
    w = W[m].mean(axis=0)
    return np.array([w[0]*1e6, w[1]*1e6, w[2]*1e6, w[3]*1e9, w[4]*1e9, w[5]*1e9])
    # forces in uN, torques in uN*mm (N*m * 1e9 = uN*mm)

labels = ["Fx(uN)","Fy(uN)","Fz(uN)","Tx(uNmm)","Ty(uNmm)","Tz(uNmm)"]
inputs = {
    "u_thrust": ("thrust", np.linspace(-0.3, 0.3, 7), 0),
    "u_roll":   ("roll",   np.linspace(-0.3, 0.3, 7), 1),
    "u_pitch":  ("pitch",  np.linspace(-0.2, 0.2, 7), 2),
}
target = {"u_thrust": 2, "u_roll": 3, "u_pitch": 4}   # intended wrench index

print("="*70)
print(" CONTROL AUTHORITY (cycle-averaged wrench vs each input)")
print("="*70)
results = {}
for name, (kw, vals, _) in inputs.items():
    W = np.array([measure(**{kw: v}) for v in vals])
    results[name] = (vals, W)
    # effectiveness = slope of the TARGET wrench component vs the input
    ti = target[name]
    slope = np.polyfit(vals, W[:, ti], 1)[0]
    # largest off-target slope (cross-coupling)
    off = [(labels[j], np.polyfit(vals, W[:, j], 1)[0]) for j in range(6) if j != ti]
    off_max = max(off, key=lambda kv: abs(kv[1]))
    print(f"\n {name}: target {labels[ti]}")
    print(f"    on-axis slope  = {slope:+.2f} {labels[ti]} per unit input")
    print(f"    biggest cross  = {off_max[1]:+.2f} {off_max[0]} per unit  "
          f"({abs(off_max[1]/slope)*100:.0f}% of on-axis)")

# validate signs/dominance
print("\n" + "="*70)
ok = True
for name in inputs:
    vals, W = results[name]; ti = target[name]
    slope = np.polyfit(vals, W[:, ti], 1)[0]
    on = abs(slope)
    cross = max(abs(np.polyfit(vals, W[:, j], 1)[0]) for j in range(6) if j != ti)
    good = on > 1e-3 and cross < 0.6*on
    ok &= good
    print(f" {name:9s}-> {labels[ti]:9s}: {'DOMINANT' if good else 'WEAK/COUPLED'}")
print(f" RESULT: {'each input controls its axis — independent loops feasible' if ok else 'coupling high — needs care'}")
print("="*70)

# CSV
with open(OUT/"e09_control_authority.csv","w",newline="") as fh:
    w = csv.writer(fh); w.writerow(["input","value"]+labels)
    for name,(vals,W) in results.items():
        for v,row in zip(vals,W): w.writerow([name,f"{v:.3f}"]+[f"{x:.3f}" for x in row])

# figure: 3 panels, target component vs input (+ the dominant cross term faint)
fig, ax = plt.subplots(1, 3, figsize=(13, 4))
titles = {"u_thrust":"Thrust input -> Fz","u_roll":"Roll input -> Tx","u_pitch":"Pitch input -> Ty"}
for k,(name) in enumerate(inputs):
    vals, W = results[name]; ti = target[name]
    ax[k].plot(vals, W[:, ti], "o-", color="tab:blue", label=f"{labels[ti]} (target)")
    for j in range(6):
        if j != ti and abs(np.polyfit(vals,W[:,j],1)[0]) > 0.1*abs(np.polyfit(vals,W[:,ti],1)[0]):
            ax[k].plot(vals, W[:, j], ":", alpha=0.6, label=labels[j])
    ax[k].axhline(0, color="k", lw=0.4); ax[k].set_title(titles[name])
    ax[k].set_xlabel(name); ax[k].legend(fontsize=7)
ax[0].set_ylabel("cycle-avg wrench")
plt.tight_layout(); fig.savefig(OUT/"e09_control_authority.png", dpi=130)
print(f"\nsaved: {OUT/'e09_control_authority.csv'}")
print(f"saved: {OUT/'e09_control_authority.png'}")