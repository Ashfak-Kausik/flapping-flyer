"""
view_corridor.py — interactive MuJoCo viewer: the flyer centering in a corridor.

Run on a machine with a display:  python experiments/view_corridor.py

The flyer hovers under the LQG controller between two walls (drawn as the blue/red
slabs; aerodynamically they are injected via the wall ground-effect model, not
MuJoCo collisions). It senses the net roll disturbance of the two walls from its
own control residual (the disturbance observer, controller.roll_dist) and commands
a lateral velocity to null it -> it finds the centreline. Every few seconds it is
nudged off-centre so you can watch it re-centre. Runs in slow motion so the 80 Hz
wingbeat is visible.

Honest note: centring is an underdamped proof-of-concept (it wobbles in, settles
loosely); robust tight centring is the open control problem. See docs/05 §9.
"""
import sys, time
from pathlib import Path
import numpy as np
import mujoco
import mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design

W = 0.020                                   # corridor half-width (m)
SLOWMO = 0.04                               # 1.0 = real time; smaller = slower

# build a corridor model (two visual, non-colliding walls) from flyer.xml
xml = (ROOT / "models" / "flyer.xml").read_text()
walls = (f'    <geom name="wall_R" type="box" pos="0 {W:.4f} 0.05" size="0.10 0.0008 0.06"'
         f' rgba="0.85 0.45 0.45 0.22" contype="0" conaffinity="0"/>\n'
         f'    <geom name="wall_L" type="box" pos="0 {-W:.4f} 0.05" size="0.10 0.0008 0.06"'
         f' rgba="0.45 0.55 0.85 0.22" contype="0" conaffinity="0"/>\n')
key = '<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
(ROOT / "models" / "flyer_corridor.xml").write_text(xml.replace(key, key + walls))

fly = Flyer(ROOT / "models" / "flyer_corridor.xml")
print("designing LQG + wall observer (feed-forward ON)...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                         Q=(5, 150, 20, 2, 2, 250, 250, 6e4))
SURF = [dict(axis=1, sign=-1, pos=+W), dict(axis=1, sign=+1, pos=-W)]
K, VMAX = 3e-5, 0.10
fly.reset(kin=kin, height=0.05, y=0.005); ctrl.reset()

with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.lookat[:] = [0, 0, 0.05]; viewer.cam.distance = 0.085
    viewer.cam.azimuth = 180; viewer.cam.elevation = -8
    t = 0.0; next_nudge = 3.0
    while viewer.is_running():
        frame_start = time.time()
        for _ in range(int(0.01 / fly.dt)):            # 10 ms of sim per drawn frame
            vy = np.clip(-K * ctrl.roll_dist, -VMAX, VMAX)
            u = ctrl.update(fly.sense(), fly.dt, vy_ref=vy)
            kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
            fly.step(kin, t, surface=SURF); t += fly.dt
            if t >= next_nudge:                        # periodic nudge off-centre
                fly.v[1] += np.random.choice([-1, 1]) * 0.10; next_nudge = t + 3.0
        viewer.sync()
        dt_wall = 0.01 / SLOWMO - (time.time() - frame_start)
        if dt_wall > 0: time.sleep(dt_wall)