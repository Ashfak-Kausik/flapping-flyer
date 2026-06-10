"""
view_hover.py — watch the closed-loop hover live in the MuJoCo viewer.

Runs the same LQG controller as e11 in an interactive window. The flyer starts
with a 10/10 deg kick (so you see it catch itself), then holds hover; a gust is
injected every couple of seconds so you can watch it recover. Wing-beat is 80 Hz,
so we play slowed down (SLOWDOWN) to make the motion visible.

Run on a machine with a display:  python experiments/view_hover.py
(If the window doesn't open, your OpenGL/GLFW may need MUJOCO_GL=glfw.)
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

SLOWDOWN = 12.0          # 1.0 = real time; higher = slower (wings more visible)
SYNC_EVERY = 12          # render every N sim steps
GUST_PERIOD = 2.0        # seconds between injected gusts
KICK = dict(pitch_deg=10.0, roll_deg=10.0)

fly = Flyer(ROOT / "models" / "flyer.xml")
print("identifying model + designing controller...")
ctrl, kin, info = design(fly)
fly.reset(kin=kin, height=0.05, **KICK); ctrl.reset()

with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.distance = 0.04; viewer.cam.azimuth = 50; viewer.cam.elevation = -18
    i = 0; gust_steps = int(GUST_PERIOD / fly.dt)
    while viewer.is_running():
        if i > 0 and i % gust_steps == 0:                      # periodic gust
            axis = np.array([20.0, 0, 0]) if (i // gust_steps) % 2 else np.array([0, 14.0, 0])
            fly.w = fly.w + axis
        u = ctrl.update(fly.sense(), fly.dt)
        kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
        fly.step(kin, i * fly.dt)
        if i % SYNC_EVERY == 0:
            viewer.cam.lookat[:] = fly.data.xpos[fly.thorax]   # keep flyer centred
            viewer.sync()
            time.sleep(SYNC_EVERY * fly.dt * SLOWDOWN)
        i += 1