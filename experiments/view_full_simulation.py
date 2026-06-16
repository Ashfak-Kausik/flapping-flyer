"""
view_full_simulation.py — one continuous run of everything the flyer can do (validated).

Run with a display:  python3 experiments/view_full_simulation.py
Phases: HOVER -> CRUISE through a straight round tunnel (centering to sub-mm) ->
CLIMB (altitude change) -> YAW TURN (re-aim heading).

Honest note: the yaw turn re-aims the nose; it does NOT yet follow a bend (the velocity
crabs — coordinated turning is the next development). So the tunnel here is STRAIGHT.
"""
import sys, time
from pathlib import Path
import numpy as np
import mujoco, mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design

Rc, Z0, NPH, TUN_END, SLOWMO = 0.032, 0.05, 16, 0.30, 0.05
xml = (ROOT/"models"/"flyer.xml").read_text(); g = ""
for xi in np.arange(0.0, 0.31, 0.030):
    for k in range(NPH):
        a = 2*np.pi*k/NPH
        g += (f'    <geom type="box" pos="{xi:.4f} {Rc*np.cos(a):.4f} {Z0+Rc*np.sin(a):.4f}"'
              f' size="0.0015 0.0012 0.0012" rgba="0.40 0.70 0.78 0.6" contype="0" conaffinity="0"/>\n')
key = '<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
(ROOT/"models"/"flyer_tour.xml").write_text(xml.replace(key, key+g))

fly = Flyer(ROOT/"models"/"flyer_tour.xml")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                         Q=(150, 150, 20, 2, 2, 250, 250, 6e4)); ctrl.K[:, 0] = 0.0
ROLL_FF = 175.0/10399.0; SURF = dict(type="cyl_in_x", center=[0.0, Z0], radius=Rc)
ctrl.reset(); fly.reset(kin=kin, height=Z0)
for i in range(int(0.5/fly.dt)):
    u = ctrl.update(fly.sense(), fly.dt); kin.set_control(thrust=u[0], roll=u[1], pitch=u[2])
    fly.step(kin, i*fly.dt, surface=None)
bx = ctrl.roll_dist


def phase(t):
    if t < 0.8: return 0.0, 0.0, Z0
    if t < 4.5: return 0.07, 0.0, Z0
    if t < 6.0: return 0.03, 0.0, Z0 + min((t-4.5)/1.5, 1)*0.013
    return 0.03, np.radians(25), Z0 + 0.013


ctrl.reset(); fly.reset(kin=kin, height=Z0)
psi_f = psi_prev = pref = I_y = I_s = 0.0; t = 0.0
with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.azimuth = 205; viewer.cam.elevation = -13; viewer.cam.distance = 0.115
    while viewer.is_running():
        f0 = time.time()
        for _ in range(int(0.01/fly.dt)):
            s = fly.sense(); Vc, htgt, href = phase(t); ctrl.h_ref = href
            psi_f += (s['yaw']-psi_f)*fly.dt/0.04; rate = (psi_f-psi_prev)/fly.dt; psi_prev = psi_f
            pref += np.clip(htgt-pref, -np.radians(25)*fly.dt, np.radians(25)*fly.dt)
            I_y = np.clip(I_y+(psi_f-pref)*fly.dt, -1.5, 1.5)
            uy = np.clip(0.08*(psi_f-pref)+0.02*rate+0.10*I_y, -0.3, 0.3)
            vfwd = s['vx']*np.cos(psi_f)+s['vy']*np.sin(psi_f); e = Vc-vfwd
            I_s = np.clip(I_s+e*fly.dt, -0.5, 0.5); pr = np.clip(0.25*e+0.9*I_s, -np.radians(8), np.radians(8))
            surf = SURF if fly.x_com[0] < TUN_END else None
            vy = np.clip(-1e-4*(ctrl.roll_dist-bx)-3.0*s['vy'], -0.10, 0.10)
            u = ctrl.update(s, fly.dt, vy_ref=vy, pitch_ref=pr)
            kin.set_control(thrust=u[0], roll=u[1]-ROLL_FF*uy, pitch=u[2], yaw=uy)
            fly.step(kin, t, surface=surf); t += fly.dt
            if t > 8.3: t = 0.0; ctrl.reset(); fly.reset(kin=kin, height=Z0); psi_f = psi_prev = pref = I_y = I_s = 0.0
        viewer.cam.lookat[:] = [fly.x_com[0]+0.02, fly.x_com[1], fly.x_com[2]]
        viewer.sync()
        dtw = 0.01/SLOWMO - (time.time()-f0)
        if dtw > 0: time.sleep(dtw)