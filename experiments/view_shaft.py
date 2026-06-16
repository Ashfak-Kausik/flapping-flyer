"""
view_shaft.py — interactive MuJoCo viewer: the flyer centering in a circular shaft.

Run on a machine with a display:  python3 experiments/view_shaft.py

A vertical round shaft (drawn as a ring of posts; aerodynamically it is the cyl_in
ground-effect surface, not MuJoCo collisions). The flyer reads the 2D proximity
vector from its own control residuals — roll_dist (left/right) and pitch_dist
(fore/aft) — and commands lateral + forward velocity to null it, so it finds the
centre of the cross-section. Every few seconds it is nudged so you can watch it
re-centre. Viewed top-down; runs in slow motion.

Honest notes (see docs/06): centring is an underdamped proof-of-concept, and it is
ANISOTROPIC — tight left/right (strong roll axis) but loose fore/aft (the pitch axis
is ~11x weaker), so a small residual sits in the fore/aft direction. This is
cross-section centring, NOT forward traversal (that needs Arc B: cruise + yaw).
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

Rc = 0.030          # shaft radius (m)
N = 32              # number of posts drawn
SLOWMO = 0.04

# build the shaft model (ring of visual, non-colliding posts) from flyer.xml
xml = (ROOT / "models" / "flyer.xml").read_text(); posts = ""
for k in range(N):
    a = 2 * np.pi * k / N
    posts += (f'    <geom name="post{k}" type="box" pos="{Rc*np.cos(a):.4f} {Rc*np.sin(a):.4f} 0.05"'
              f' size="0.0012 0.0012 0.035" rgba="0.40 0.70 0.75 0.5" contype="0" conaffinity="0"/>\n')
key = '<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
(ROOT / "models" / "flyer_shaft.xml").write_text(xml.replace(key, key + posts))

fly = Flyer(ROOT / "models" / "flyer_shaft.xml")
print("designing LQG + roll+pitch observer (feed-forward ON)...")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3, 4), feedforward=True,
                         Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
SURF = dict(type="cyl_in", center=[0, 0], radius=Rc)

# measure the free-air disturbance biases to subtract (roll~0, pitch~+42)
ctrl.reset(); fly.reset(kin=kin, height=0.05)
for i in range(int(0.6 / fly.dt)):
    u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
    fly.step(kin, i*fly.dt, surface=None)
bx, by = ctrl.roll_dist, ctrl.pitch_dist
Ky, Kx, VMAX = 3e-5, 3e-5*11, 0.10

ctrl.reset(); fly.reset(kin=kin, height=0.05); fly.x_com[0] = 0.009
with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.lookat[:] = [0, 0, 0.05]; viewer.cam.distance = 0.105
    viewer.cam.azimuth = 90; viewer.cam.elevation = -84
    t = 0.0; nxt = 2.5
    while viewer.is_running():
        f0 = time.time()
        for _ in range(int(0.01 / fly.dt)):
            vy = np.clip(-Ky * (ctrl.roll_dist - bx), -VMAX, VMAX)
            vx = np.clip(+Kx * (ctrl.pitch_dist - by), -VMAX, VMAX)
            u = ctrl.update(fly.sense(), fly.dt, vy_ref=vy, vx_ref=vx)
            kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
            fly.step(kin, t, surface=SURF); t += fly.dt
            if t >= nxt:
                a = np.random.uniform(0, 2*np.pi)
                fly.v[0] += 0.11*np.cos(a); fly.v[1] += 0.11*np.sin(a); nxt = t + 2.5
        viewer.sync()
        dtw = 0.01 / SLOWMO - (time.time() - f0)
        if dtw > 0: time.sleep(dtw)