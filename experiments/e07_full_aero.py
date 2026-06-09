"""
e07_full_aero.py — STAGE 2 CAPSTONE: the full quasi-steady model.

Assembles translational + rotational + added-mass into wing_aero() and runs the
end-to-end checks on the real flyer:
  (A) force DECOMPOSITION over a cycle — each term's vertical contribution and
      the total, with cycle-mean of each (sanity vs the individual checks);
  (B) the f^2 law still holds for the TOTAL lift;
  (C) preview of the control knob: total lift at symmetric vs advanced rotation.
Saves decomposition trace (CSV), freq sweep (CSV), and figure (PNG).
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
PHI = np.deg2rad(60); PSI = np.deg2rad(45)

def set_state(t, W, d):
    s  =  PHI*np.cos(W*t);  ds = -PHI*W*np.sin(W*t)
    ps = -PSI*np.tanh(3*np.sin(W*t + d))
    dps= -PSI*3*W*np.cos(W*t + d)*(1 - np.tanh(3*np.sin(W*t + d))**2)
    data.qpos[J["stroke_R"][0]] =  s; data.qvel[J["stroke_R"][1]] =  ds
    data.qpos[J["stroke_L"][0]] = -s; data.qvel[J["stroke_L"][1]] = -ds
    data.qpos[J["pitch_R"][0]]  = ps; data.qvel[J["pitch_R"][1]]  = dps
    data.qpos[J["pitch_L"][0]]  = ps; data.qvel[J["pitch_L"][1]]  = dps
    mujoco.mj_forward(model, data)

def run(f_hz, d=0.0):
    """Return time, per-term vertical force arrays, and total."""
    W = 2*np.pi*f_hz
    mujoco.mj_resetData(model, data)
    n = int(4.0/f_hz / DT)
    ts = np.empty(n); comp = {k: np.zeros(n) for k in ("trans","rot","added","total")}
    vn = {s: None for s in "RL"}
    for i in range(n):
        t = i*DT; set_state(t, W, d)
        tot = 0.0; per = {"trans":0.0,"rot":0.0,"added":0.0}
        for s_lr, bid in wings.items():
            F, _, vn[s_lr], info = wing_aero(model, data, bid, strips[s_lr], vn[s_lr], DT)
            for k in per: per[k] += info[k][0][2]
            tot += F[2]
        for k in per: comp[k][i] = per[k]
        comp["total"][i] = tot; ts[i] = t
    return ts, comp

# ---------- (A) decomposition at 40 Hz, symmetric ----------
F_REF = 40.0
ts, comp = run(F_REF, d=0.0)
mask = ts > (4.0/F_REF - 2.0/F_REF)
print(f"body weight = {WEIGHT*1e6:.1f} uN")
print(f"\n(A) cycle-mean vertical force by term @ {F_REF:.0f} Hz (symmetric rotation):")
for k in ("trans","rot","added","total"):
    mu = comp[k][mask].mean()
    print(f"    {k:7s}: mean = {mu*1e6:+8.2f} uN  ({mu/WEIGHT*100:+6.2f}% W)   "
          f"rms = {np.sqrt((comp[k][mask]**2).mean())*1e6:7.2f} uN")

# ---------- (B) f^2 law for the TOTAL ----------
print("\n(B) f^2 law for TOTAL lift:")
freqs = [20,30,40,50,60]; tot_means = []
for f in freqs:
    _, c = run(f, d=0.0); m = None
    tsf = np.arange(len(c["total"]))*DT; m = tsf > (4.0/f - 2.0/f)
    tot_means.append(c["total"][m].mean())
ratios = np.array([tm*1e6/f**2 for tm,f in zip(tot_means,freqs)])
print("    freq:", freqs)
print("    lift/f^2 spread = {:.2f}%  ({})".format(
    ratios.std()/ratios.mean()*100,
    "f^2 holds for full model" if ratios.std()/ratios.mean()<0.05 else "deviates"))

# ---------- (C) control-knob preview: total lift vs rotation timing ----------
print("\n(C) total cycle-mean lift vs rotation timing @ 40 Hz:")
for d in (0.0, 0.3, 0.6):
    _, c = run(F_REF, d=d); m = (np.arange(len(c["total"]))*DT) > (4.0/F_REF-2.0/F_REF)
    mu = c["total"][m].mean()
    print(f"    phase {d:.1f}:  total lift = {mu*1e6:7.1f} uN  ({mu/WEIGHT:.3f} W)")

# save decomposition CSV
with open(OUT/"e07_decomposition.csv","w",newline="") as fh:
    w = csv.writer(fh); w.writerow(["t_ms","trans_uN","rot_uN","added_uN","total_uN"])
    idx = np.where(mask)[0]
    for i in idx:
        w.writerow([f"{(ts[i]-ts[idx[0]])*1e3:.4f}"]+[f"{comp[k][i]*1e6:.4f}" for k in
                    ("trans","rot","added","total")])

# figure
fig, ax = plt.subplots(1, 2, figsize=(13, 4.2))
tt = (ts[mask]-ts[mask][0])*1e3
for k,col in [("trans","tab:blue"),("rot","tab:orange"),("added","tab:green"),("total","k")]:
    ax[0].plot(tt, comp[k][mask]*1e6, color=col, lw=1.8 if k=="total" else 1.3, label=k)
ax[0].axhline(WEIGHT*1e6, color="gray", ls="--", lw=1, label="weight")
ax[0].axhline(0, color="k", lw=0.4)
ax[0].set_xlabel("time (ms)"); ax[0].set_ylabel("vertical force (uN)")
ax[0].set_title(f"Full quasi-steady model @ {F_REF:.0f} Hz — decomposition")
ax[0].legend(fontsize=8, ncol=2)
ff=np.array(freqs); ll=np.array(tot_means)*1e6
ax[1].plot(ff, ll, "o-", label="total avg lift")
ax[1].plot(ff, ratios.mean()*ff**2, "k--", label="$k f^2$ fit")
ax[1].set_xlabel("flapping frequency (Hz)"); ax[1].set_ylabel("total avg lift (uN)")
ax[1].set_title("Total lift obeys the $f^2$ law"); ax[1].legend(fontsize=8)
plt.tight_layout(); fig.savefig(OUT/"e07_full_aero.png", dpi=130)
print(f"\nsaved: {OUT/'e07_decomposition.csv'}")
print(f"saved: {OUT/'e07_full_aero.png'}")