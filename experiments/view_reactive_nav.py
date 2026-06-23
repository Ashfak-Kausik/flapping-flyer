"""
view_reactive_nav.py — watch the REACTIVE navigator LIVE in the MuJoCo viewer.
Run WITH a display:  python experiments/view_reactive_nav.py

The flyer feels its way: ANTENNA rays detect walls ahead and steer the nose toward the
opening; WING-WASH (roll_dist) keeps it centered between the side walls. No memorized
route. Here the course is a single L-bend; it rounds it by sensing. Loops; close to quit.
"""
import sys, time
from pathlib import Path
import numpy as np
import mujoco, mujoco.viewer

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna, BIG
from experiments.e31_corner import bodyframe
from experiments.e35_reactive_nav import WALLS, planes

Vc = 0.06; Kc = 1.5e-5; Kd = 3.0; KLAT = 3.0; ROLL_FF = 175.0 / 10399.0
YAWRATE = 0.5; Ksteer = 4.0; STOP = 0.045; CLEAR = 0.095; FMAX = 0.12; SLOWMO = 0.12

xml = (ROOT / "models" / "flyer.xml").read_text()
g = "".join(f'    <geom name="{n}" type="box" pos="{p}" size="{s}" group="3" '
            f'rgba="0.62 0.66 0.72 1" contype="0" conaffinity="0"/>\n' for n, p, s in WALLS)
key = '<geom name="floor" type="plane" size="0 0 0.01" material="groundplane"/>\n'
(ROOT / "models" / "flyer_reactive.xml").write_text(xml.replace(key, key + g))

fly = Flyer(ROOT / "models" / "flyer_reactive.xml")
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                         Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
ctrl.K[:, 0] = 0.0; ctrl.K[:, 1] = 0.0
ant = Antenna(fly)


class M: pass

def reset_nav():
    ctrl.reset(); fly.reset(kin=kin, height=0.05); ctrl.h_ref = 0.05
    m = M(); m.t = 0.0; m.I_s = 0.0; m.pref = 0.0
    m.nose_f = m.nose_prev = m.I_y = 0.0; m.rd_f = 0.0
    m.state = "CRUISE"; m.tdir = 0; m.turn0 = None
    return m

m = reset_nav()


def step():
    s = fly.sense(); psi = s['yaw']; x, y, z = fly.x_com; spd = np.hypot(s['vx'], s['vy'])
    b = bodyframe(s); u_fwd = b['vx']; v_lat = b['vy']
    m.nose_f += ((psi - m.nose_f + np.pi) % (2 * np.pi) - np.pi) * fly.dt / 0.04
    nrate = (m.nose_f - m.nose_prev) / fly.dt; m.nose_prev = m.nose_f
    fwd, fL, fR, dL, dR = ant.feel([0, 30, -30, 90, -90])
    rd = ctrl.roll_dist; m.rd_f += (rd - m.rd_f) * fly.dt / 0.10
    pl = planes(psi, fly.x_com, dL, dR)
    if m.state == "CRUISE":
        Vcmd = Vc * np.clip((fwd - STOP) / 0.04, 0.0, 1.0)
        m.pref += np.clip(Ksteer * (min(fL, FMAX) - min(fR, FMAX)), -0.5, 0.5) * fly.dt
        roll_ref = np.clip(-Kc * m.rd_f - Kd * v_lat, -np.radians(2.5), np.radians(2.5))
        if fwd < STOP and spd < 0.03:
            m.tdir = +1 if fL >= fR else -1; m.state = "TURN"; m.turn0 = m.nose_f; m.I_y = 0.0
    else:
        Vcmd = 0.0; m.pref += m.tdir * YAWRATE * fly.dt
        roll_ref = np.clip(-KLAT * v_lat, -np.radians(2), np.radians(2))
        turned = abs(((m.nose_f - m.turn0 + np.pi) % (2 * np.pi) - np.pi))
        if fwd > CLEAR and turned > np.radians(20): m.state = "CRUISE"; m.I_s = 0.0; m.rd_f = 0.0
        elif turned > np.radians(175): m.tdir = -m.tdir; m.turn0 = m.nose_f
    er = Vcmd - u_fwd; m.I_s = np.clip(m.I_s + er * fly.dt, -0.6, 0.6)
    pr = np.clip(0.30 * er + 1.1 * m.I_s, -np.radians(8), np.radians(8))
    eyaw = ((m.nose_f - m.pref + np.pi) % (2 * np.pi) - np.pi); m.I_y = np.clip(m.I_y + eyaw * fly.dt, -1.0, 1.0)
    uy = np.clip(0.14 * eyaw + 0.03 * nrate + 0.10 * m.I_y, -0.3, 0.3)
    u = ctrl.update(b, fly.dt, pitch_ref=pr, roll_ref=roll_ref, vy_ref=0.0)
    kin.set_control(thrust=u[0], roll=u[1] - ROLL_FF * uy, pitch=u[2], yaw=uy)
    fly.step(kin, m.t, surface=(pl + [dict(axis=2, sign=1, pos=0.0)])); m.t += fly.dt
    return y > 0.34 or m.t > 20.0          # reached end of corridor-1 -> loop


with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.azimuth = -50; viewer.cam.elevation = -40; viewer.cam.distance = 0.22
    viewer.cam.lookat[:] = [0.07, 0.05, 0.0]
    viewer.opt.geomgroup[3] = 1            # draw the sensing-group walls (hidden by default)
    while viewer.is_running():
        f0 = time.time()
        for _ in range(int(0.01 / fly.dt)):
            if step():
                time.sleep(0.5); m = reset_nav(); break
        viewer.sync()
        dtw = 0.01 / SLOWMO - (time.time() - f0)
        if dtw > 0: time.sleep(dtw)