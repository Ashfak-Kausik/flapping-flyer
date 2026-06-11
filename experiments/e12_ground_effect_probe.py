"""
e12_ground_effect_probe.py — STAGE 5/6 bridge: is there a usable flow-proximity signal?

Feasibility probe for the project's core novelty: can a hovering flapping flyer
sense a nearby surface from its OWN aerodynamics? We clamp the flyer hovering at
a range of standoffs above a floor and measure how the cycle-averaged aero wrench
shifts versus free air, using the literature-grounded ground-effect model in
src/ground_effect.py (added, not derived — see that file's header).

Reports, per standoff: the lift shift dFz (the candidate signal), the wingbeat
ripple in Fz (context), and whether the symmetric floor produces any false
directional torque. A short second part tilts the flyer near the floor to show a
DIRECTIONAL torque appears when the surface is approached asymmetrically — the
physics the wall-sensing / avoidance loop will use.

HONEST SCOPE: this shows that a known effect of conservative magnitude yields a
signal our stack could exploit, and over what range; it does not prove the
effect's real-world magnitude (that needs a wind tunnel). K_GE is the parameter
to calibrate for sim-to-real.
"""
import sys
from pathlib import Path
import csv
import numpy as np
import mujoco
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.kinematics import FlapKinematics
from src import aero, ground_effect as ge

OUT = ROOT / "outputs"; OUT.mkdir(exist_ok=True)
AMP = 72.63                       # validated hover amplitude at 80 Hz (e10)
fly = Flyer(ROOT / "models" / "flyer.xml")
R = aero.wing_params_from_model(fly.model)["r_tip"]
WEIGHT = fly.M * 9.81
kin = FlapKinematics(f_hz=80, stroke_amp_deg=AMP, feather_amp_deg=45)


def _sys_com():
    c = np.zeros(3)
    for i in range(1, fly.model.nbody):
        c += fly.model.body_mass[i] * fly.data.xipos[i]
    return c / fly.M


def measure(height, surface, pitch_deg=0.0, roll_deg=0.0, ncyc=12, navg=8):
    """Cycle-averaged aero wrench about the system CoM with the body clamped level
    (or tilted) at `height`, with and without `surface`. Also returns Fz ripple."""
    m, d = fly.model, fly.data
    period = 2 * np.pi / kin.W; N = int(ncyc * period / fly.dt); t0 = (ncyc - navg) * period
    qp = np.array([np.cos(np.deg2rad(pitch_deg) / 2), 0, np.sin(np.deg2rad(pitch_deg) / 2), 0])
    qr = np.array([np.cos(np.deg2rad(roll_deg) / 2), np.sin(np.deg2rad(roll_deg) / 2), 0, 0])
    from src.flyer import _qmul
    q = _qmul(qp, qr); q /= np.linalg.norm(q)
    vn_g = {s: None for s in "RL"}; vn_f = {s: None for s in "RL"}
    acc_g = []; acc_f = []; fz_inst = []
    for i in range(N):
        t = i * fly.dt
        d.qpos[fly._qf:fly._qf + 3] = [0.0, 0.0, height]
        d.qpos[fly._qf + 3:fly._qf + 7] = q
        d.qvel[fly._vf:fly._vf + 6] = 0.0
        fly._prescribe_wings(kin, t)
        mujoco.mj_forward(m, d)
        com = _sys_com()
        Fg = np.zeros(3); Tg = np.zeros(3); Ff = np.zeros(3); Tf = np.zeros(3)
        for s, bid in fly.wings.items():
            fg, tg, vn_g[s], _ = ge.wing_aero_ge(m, d, bid, fly.strips[s], vn_g[s], fly.dt, surface, R)
            ff, tf, vn_f[s], _ = aero.wing_aero(m, d, bid, fly.strips[s], vn_f[s], fly.dt)
            Fg += fg; Tg += tg + np.cross(d.xipos[bid] - com, fg)
            Ff += ff; Tf += tf + np.cross(d.xipos[bid] - com, ff)
        if t >= t0:
            acc_g.append(np.concatenate([Fg, Tg])); acc_f.append(np.concatenate([Ff, Tf]))
            fz_inst.append(Fg[2])
    Wg = np.mean(acc_g, 0); Wf = np.mean(acc_f, 0)
    fz_inst = np.array(fz_inst)
    return Wg, Wf, dict(fz_pp=fz_inst.max() - fz_inst.min(), fz_std=fz_inst.std())


# ---- Part 1: lift signal vs standoff above a floor (z=0) ----
floor = dict(axis=2, sign=+1, pos=0.0)
standoffs_mm = np.array([8, 10, 12, 15, 18, 22, 26, 32, 40, 50, 60])
rows = []
print("=" * 72)
print(" STAGE 5/6 bridge — flow-proximity feasibility probe (FLOOR / ground effect)")
print("=" * 72)
print(f" wing length R = {R*1e3:.2f} mm,  weight = {WEIGHT*1e6:.1f} uN,  K_GE = {ge.K_GE}")
print(f" {'standoff':>10} {'d/R':>6} {'dFz':>10} {'dFz/W':>8} {'Fz ripple(pp)':>14} {'dFz/ripple':>11}")
for dmm in standoffs_mm:
    h = dmm / 1e3
    Wg, Wf, rip = measure(h, floor)
    dFz = Wg[2] - Wf[2]
    dFz_pct = dFz / WEIGHT * 100
    ratio = dFz / (rip["fz_pp"] + 1e-30)
    dTx, dTy = Wg[3] - Wf[3], Wg[4] - Wf[4]          # ground-effect-INDUCED torque shift
    rows.append((dmm, dmm / 1e3 / R, Wf[2], Wg[2], dFz, dFz_pct, rip["fz_pp"], rip["fz_std"],
                 dTx, dTy))
    print(f" {dmm:8d}mm {dmm/1e3/R:6.2f} {dFz*1e6:9.2f}uN {dFz_pct:7.2f}% {rip['fz_pp']*1e6:12.1f}uN {ratio:10.4f}")

# detection range: where dFz exceeds a conservative 0.5% of weight (a plausible
# resolvable lift change for an onboard sensor integrating over one wingbeat)
THR = 0.005 * WEIGHT
detect = [r for r in rows if abs(r[4]) >= THR]
range_mm = max(r[0] for r in detect) if detect else 0
print("-" * 72)
print(f" wingbeat ripple AVERAGES OUT over a full cycle (12.5 ms), so the cycle-mean")
print(f" dFz is recoverable; the real-world floor is sensor resolution + 1-cycle latency.")
print(f" CHECK (symmetric floor -> no false ROLL signal): max|ΔTx| = "
      f"{max(abs(r[8]) for r in rows)*1e9:.3f} uNmm  (≈0 ok; ΔTy is a real symmetric pitch effect)")
print(f" -> lift signal >= 0.5% of weight out to ~{range_mm} mm = {range_mm/1e3/R:.1f} wing-lengths")

# ---- Part 2: directional preview — tilt near the floor, watch a torque appear ----
print("-" * 72)
print(" directional preview: roll the flyer 15 deg at 14 mm standoff (one wing closer)")
Wg_t, Wf_t, _ = measure(0.014, floor, roll_deg=15.0)
dTx = (Wg_t[3] - Wf_t[3])
print(f"   ground-effect roll torque dTx = {dTx*1e9:+.2f} uNmm  (nonzero => directional signal)")
print(f"   (level floor gives ΔTx ≈ {max(abs(r[8]) for r in rows)*1e9:.3f} uNmm — symmetric, no roll)")
print("=" * 72)

# ---- save ----
with open(OUT / "e12_ground_effect.csv", "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["standoff_mm", "d_over_R", "Fz_free_uN", "Fz_ge_uN", "dFz_uN",
                "dFz_pct_weight", "Fz_ripple_pp_uN", "Fz_ripple_std_uN", "Tx_ge_uNmm", "Ty_ge_uNmm"])
    for r in rows:
        w.writerow([r[0], f"{r[1]:.3f}", f"{r[2]*1e6:.3f}", f"{r[3]*1e6:.3f}", f"{r[4]*1e6:.4f}",
                    f"{r[5]:.4f}", f"{r[6]*1e6:.3f}", f"{r[7]*1e6:.3f}", f"{r[8]*1e9:.4f}", f"{r[9]*1e9:.4f}"])

dR = np.array([r[1] for r in rows]); dpct = np.array([r[5] for r in rows])
fig, ax = plt.subplots(1, 2, figsize=(12, 4.8))
ax[0].plot(dR, dpct, "o-", color="tab:blue")
ax[0].axhline(0.5, color="tab:red", ls="--", lw=1, label="0.5% of weight (detection threshold)")
ax[0].set_xlabel("standoff  d / R  (wing-lengths)"); ax[0].set_ylabel("lift increase  ΔFz  (% of weight)")
ax[0].set_title("Ground-effect lift signal vs standoff"); ax[0].grid(alpha=0.3); ax[0].legend()
ax[1].loglog(np.array([r[0] for r in rows]), np.abs(dpct), "o-", color="tab:blue", label="ΔFz / W")
ax[1].axhline(0.5, color="tab:red", ls="--", lw=1)
ax[1].set_xlabel("standoff (mm)"); ax[1].set_ylabel("|ΔFz| (% of weight)")
ax[1].set_title("Same, log–log (slope ≈ −2: the (R/4d)² law)"); ax[1].grid(alpha=0.3, which="both")
plt.tight_layout(); fig.savefig(OUT / "e12_ground_effect.png", dpi=130)
print(f"saved: {OUT/'e12_ground_effect.csv'}")
print(f"saved: {OUT/'e12_ground_effect.png'}")