"""
paper_regen_data.py — regenerate the three traces the paper figures need that were never
persisted to a committed CSV. Nothing committed is overwritten; outputs go to
outputs/paper_figures/data/. Each uses the existing experiment code unchanged.

  e41      e41.sweep() (static aero, deterministic): roll torque / roll accel vs wall distance.
           The committed e41 CSVs hold only the derived fits and resolution, not this curve.
  e11_ol   the OPEN-LOOP half of e11 (same kick, no control); e11's CSV holds only the closed loop.
           The simulate() loop is copied from e11_hover_control.py (importing e11 would re-run it
           and overwrite its outputs).
  mission  one e48 mission on the e47 reference course on the CURRENT (true-wall) harness,
           level 1.0, seed 1, fuse=True — the same call as e48's __main__, with the trajectory kept.

Run: python experiments/report_figs/paper_regen_data.py {e41|e11_ol|mission}
"""
import sys; sys.path.insert(0, '.')
import csv, json, os, time
import numpy as np

OUT = "outputs/paper_figures/data"


def do_e41():
    import experiments.e41_wingwash_sensor as e41
    rows = e41.sweep()
    with open(f"{OUT}/e41_sweep.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["d_mm", "T_wall_nNm", "roll_acc_rad_s2", "kappa"]); w.writerows(rows)


def do_e11_ol():
    from src.flyer import Flyer
    from src.controller import design
    T_END = 1.2; KICK = dict(pitch_deg=10.0, roll_deg=10.0)          # e11_hover_control.py constants
    fly = Flyer("models/flyer.xml"); ctrl, kin, info = design(fly); n = int(T_END / fly.dt)
    fly.reset(kin=kin, height=0.05, **KICK); ctrl.reset()
    with open(f"{OUT}/e11_open_loop.csv", "w", newline="") as fh:
        w = csv.writer(fh); w.writerow(["t_ms", "pitch_deg", "roll_deg", "height_mm"])
        for i in range(n):
            fly.step(kin, i * fly.dt); s = fly.sense()
            w.writerow([f"{i * fly.dt * 1e3:.3f}", f"{np.rad2deg(s['pitch']):.4f}", f"{np.rad2deg(s['roll']):.4f}",
                        f"{s['height'] * 1e3:.4f}"])


def do_mission():
    import subprocess
    import experiments.e47_realistic_course as e47
    import experiments.e48_mission as e48
    geo = e47.build_geometry(e47.LEGS, e47.BREACHES)
    r = e48.run(geo, level=1.0, seed=1, fuse=True, model_path=f"{OUT}/_mission_model.xml")
    os.remove(f"{OUT}/_mission_model.xml")
    json.dump(dict(course="e47.LEGS / e47.BREACHES", legs=e47.LEGS, breaches=e47.BREACHES, level=1.0, seed=1, fuse=True,
                   harness_commit=subprocess.check_output(["git", "rev-parse", "--short", "HEAD"]).decode().strip(),
                   reached=r["reached"], crashed=bool(r["crashed"]), min_clear_mm=float(r["min_clear_mm"]), t=float(r["t"]),
                   traj=[[float(v) for v in p[:2]] for p in r["traj"]],
                   detected=[[d[0], [float(v) for v in d[1]], [float(v) for v in d[2]]] for d in r["detected"]]),
              open(f"{OUT}/mission_truewall.json", "w"))
    print(f"outcome={r['reached']} crashed={r['crashed']} minClear={r['min_clear_mm']:.1f}mm t={r['t']:.1f}s "
          f"detected={len(r['detected'])}")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True); t0 = time.time()
    {"e41": do_e41, "e11_ol": do_e11_ol, "mission": do_mission}[sys.argv[1]]()
    print(f"done {sys.argv[1]} in {time.time() - t0:.0f}s")
