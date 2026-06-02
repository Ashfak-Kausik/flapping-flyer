"""
e01_load_model.py  —  STAGE 1 CHECK (bio-inspired body)

Loads models/flyer.xml and prints the numbers we verify before moving on:
masses, wing planform (read back from geometry), aspect ratio, centre of mass
(must sit below the wing hinges), and a headless stability check that
replicates the viewer's auto-run so we never get surprised by a NaN again.

Usage:
    python experiments/e01_load_model.py
    python experiments/e01_load_model.py --view
"""
import argparse
from pathlib import Path
import numpy as np
import mujoco

MODEL_PATH = Path(__file__).resolve().parents[1] / "models" / "flyer.xml"


def main(view: bool):
    model = mujoco.MjModel.from_xml_path(str(MODEL_PATH))
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    def bid(n): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, n)
    def gid(n): return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, n)

    total_mass = float(model.body_mass.sum())
    wing_m = float(model.body_mass[bid("wing_R")])
    body_m = float(model.body_mass[bid("thorax")])

    # wing ellipsoid semi-axes: (chord/2 in x, span/2 in y, thick/2 in z)
    s = model.geom_size[gid("wing_R_g")]
    R = 2 * s[1]                 # span, root->tip
    c_max = 2 * s[0]             # max chord
    c_mean = np.pi / 4 * c_max   # mean chord of an ellipse
    aspect = R / c_mean

    # centre of mass (subtree CoM of root body = whole flyer) vs wing hinge
    com = data.subtree_com[bid("thorax")].copy()
    hinge = data.xpos[bid("stroke_R")].copy()

    print("=" * 60)
    print(" STAGE 1 CHECK — bio-inspired flyer body")
    print("=" * 60)
    print(f" bodies : {model.nbody}   joints : {model.njnt}   "
          f"actuators : {model.nu}   DOF : {model.nv}")
    print("-" * 60)
    print(f" total mass         : {total_mass*1e6:8.2f} mg")
    print(f"   body (thorax+...) : {body_m*1e6:8.2f} mg")
    print(f"   each wing         : {wing_m*1e6:8.2f} mg")
    print("-" * 60)
    print(f" wing span     (R)  : {R*1e3:8.2f} mm")
    print(f" max chord          : {c_max*1e3:8.2f} mm")
    print(f" mean chord (ellipse): {c_mean*1e3:7.2f} mm")
    print(f" aspect ratio R/c   : {aspect:8.2f}")
    print("-" * 60)
    print(f" centre of mass (mm): x {com[0]*1e3:+.3f}   y {com[1]*1e3:+.3f}   z {com[2]*1e3:+.3f}")
    print(f" wing-hinge height  : z {hinge[2]*1e3:.3f} mm")
    print(f"  -> CoM is {'BELOW' if com[2] < hinge[2] else 'ABOVE'} the hinges "
          f"({'pendulum-stable' if com[2] < hinge[2] else 'top-heavy!'})")
    print("-" * 60)

    # stability: replicate the viewer 'Run' headless
    steps = int(0.2 / model.opt.timestep)
    ok = True
    for i in range(steps):
        data.ctrl[:] = 0.0
        mujoco.mj_step(model, data)
        if not (np.all(np.isfinite(data.qpos)) and np.all(np.isfinite(data.qvel))):
            ok = False
            print(f" STABILITY : !! NaN at step {i}, t={data.time:.4f}")
            break
    if ok:
        print(f" STABILITY : {steps} steps @ ctrl=0, all finite — no NaN.")
    print("=" * 60)

    if view:
        from mujoco import viewer as mj_viewer
        mujoco.mj_resetData(model, data)
        print("opening viewer — close the window to exit")
        mj_viewer.launch(model, data)


if __name__ == "__main__":
    p = argparse.ArgumentParser()
    p.add_argument("--view", action="store_true", help="open interactive viewer")
    main(p.parse_args().view)