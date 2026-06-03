"""
e02_strips.py — STAGE 2 CHECK (piece 1): wing discretization.

Proves the strip discretisation faithfully represents the real wing:
  (1) strip areas must sum to the analytic ellipse area (convergence),
  (2) the area moments — which set WHERE lift acts and HOW force scales —
      must match theory.
Saves the strip table (CSV) and the chord-distribution figure (PNG) so the
numbers are on disk for the paper the moment they are produced.
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
from src.aero import wing_params_from_model, chord, build_strips

OUT = ROOT / "outputs"
OUT.mkdir(exist_ok=True)

model = mujoco.MjModel.from_xml_path(str(ROOT / "models" / "flyer.xml"))
p = wing_params_from_model(model)
a, b = p["c_max"] / 2.0, p["span_semi"]
R = p["r_tip"]                       # hinge -> tip (the aerodynamic radius)
area_true = np.pi * a * b            # analytic ellipse planform area

print("wing params (read from model):")
for k, v in p.items():
    print(f"   {k:10s} = {v*1e3:8.4f} mm")
print(f"   analytic planform area = {area_true*1e6:.4f} mm^2\n")

print(" n_strips |  area (mm^2) |  error")
for n in (4, 8, 16, 32, 64, 128):
    area = build_strips(p["c_max"], p["y_center"], p["span_semi"], n)["dS"].sum()
    print(f"   {n:5d}  |  {area*1e6:9.4f}   | {(area/area_true-1)*100:+6.2f} %")

s = build_strips(p["c_max"], p["y_center"], p["span_semi"], 2000)
S = s["dS"].sum()
r1 = (s["r"] * s["dS"]).sum() / S
r2 = np.sqrt((s["r"]**2 * s["dS"]).sum() / S)
print(f"\n radial centroid   r1_hat = {r1/R:.4f}   (0.5 = mid-span; >0.5 = outboard)")
print(f" 2nd-moment radius r2_hat = {r2/R:.4f}   (typical insect 0.50-0.60)")

# save strip table (CSV) at a working resolution
n_use = 20
s = build_strips(p["c_max"], p["y_center"], p["span_semi"], n_use)
with open(OUT / "e02_strip_table.csv", "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["strip", "r_mm", "dr_mm", "chord_mm", "area_mm2"])
    for i in range(n_use):
        w.writerow([i, f"{s['r'][i]*1e3:.4f}", f"{s['dr']*1e3:.4f}",
                    f"{s['c'][i]*1e3:.4f}", f"{s['dS'][i]*1e6:.5f}"])

# figure: chord distribution + strips
rr = np.linspace(p["r_root"], p["r_tip"], 400)
cc = chord(rr, p["c_max"], p["y_center"], p["span_semi"])
fig, ax = plt.subplots(figsize=(8, 3.6))
ax.fill_between(rr*1e3, cc*1e3/2, -cc*1e3/2, alpha=0.25, label="wing planform")
ax.bar(s["r"]*1e3, s["c"]*1e3, width=s["dr"]*1e3*0.9, alpha=0.45,
       bottom=-s["c"]*1e3/2, edgecolor="k", linewidth=0.4, label=f"{n_use} strips")
ax.axvline(r1*1e3, color="r", ls="--", lw=1, label=f"area centroid = {r1/R:.2f} R")
ax.set_xlabel("spanwise station r from hinge (mm)")
ax.set_ylabel("chord (mm)")
ax.set_title("Elliptical wing — chord distribution & blade-element strips")
ax.legend(fontsize=8, loc="upper left"); ax.set_aspect("equal")
plt.tight_layout(); fig.savefig(OUT / "e02_chord_distribution.png", dpi=130)
print(f"\nsaved: {OUT/'e02_strip_table.csv'}")
print(f"saved: {OUT/'e02_chord_distribution.png'}")