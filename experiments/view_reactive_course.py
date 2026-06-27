"""
view_reactive_course.py — watch the REACTIVE COURSE run LIVE in the MuJoCo viewer.
Run WITH a display:   python experiments/view_reactive_course.py

The flyer feels its way through the winding corridor (START -> L90 -> R90 -> 30L ->
sharp RIGHT -> FINISH pad): ANTENNA rays detect walls ahead and steer toward the opening,
WING-WASH keeps it centered. It navigates the WHOLE corridor purely by sensing (no homing,
no shortcut), LANDS on the finish pad, pauses, then loops. Close the window to quit.

Course length is set by SCALE in e36_reactive_course.py (e.g. SCALE=3.0 tripls it); the
viewer's timeout scales with it automatically.
"""
import sys, time
from pathlib import Path
import numpy as np, mujoco, mujoco.viewer
ROOT = Path(__file__).resolve().parents[1]; sys.path.insert(0, str(ROOT))
from src.flyer import Flyer
from src.controller import design
from src.antenna import Antenna, BIG
from experiments.e31_corner import bodyframe
import experiments.e36_reactive_course as e

SAFE_BUF=0.020; KVEER=0.9
SLOWMO = 0.12          # 0.12 = ~8x slower than real time
PAUSE_AT_FINISH = 2.5  # seconds to sit landed on the pad before looping
TIMEOUT = e.run_timeout()   # scales with course length

fly = Flyer(Path(e.build_model(str(ROOT / "models" / "_reactive_course.xml"))))
ctrl, kin, info = design(fly, dist_obs=True, dist_states=(3,), feedforward=True,
                         Q=(150, 150, 20, 2, 2, 250, 250, 6e4))
ctrl.K[:, 0] = 0.0; ctrl.K[:, 1] = 0.0
ant = Antenna(fly)


class S: pass

def reset_nav():
    ctrl.reset(); fly.reset(kin=kin, height=e.CRUISE); ctrl.h_ref = e.CRUISE
    m = S(); m.t = 0.0; m.I_s = 0.0; m.pref = 0.0
    m.nose_f = m.nose_prev = m.I_y = 0.0; m.rd_f = 0.0
    m.state = "CRUISE"; m.tdir = 0; m.turn0 = None
    m.phase = "NAV"; m.land_t = 0.0
    return m

m = reset_nav()


def step():
    """Advance one sim step. Returns True when the run is complete (time to loop)."""
    s = fly.sense(); psi = s['yaw']; x, y, z = fly.x_com; spd = np.hypot(s['vx'], s['vy'])
    b = bodyframe(s); u_fwd = b['vx']; v_lat = b['vy']
    m.nose_f += ((psi - m.nose_f + np.pi) % (2 * np.pi) - np.pi) * fly.dt / 0.04
    nrate = (m.nose_f - m.nose_prev) / fly.dt; m.nose_prev = m.nose_f
    f0, fLp, fRp, f50p, f50m, dL, dR = ant.feel([0, 30, -30, 50, -50, 90, -90]); fwd, fL, fR = f0, fLp, fRp
    rd = ctrl.roll_dist; m.rd_f += (rd - m.rd_f) * fly.dt / 0.10
    pl = e.planes(psi, fly.x_com, dL, dR)
    dist_fin = np.hypot(e.FINISH[0] - x, e.FINISH[1] - y)

    if m.phase == "NAV":
        left_near = min(f50p, dL); right_near = min(f50m, dR)
        safe = 0.0
        if right_near < SAFE_BUF: safe += KVEER * (SAFE_BUF - right_near) / SAFE_BUF
        if left_near  < SAFE_BUF: safe -= KVEER * (SAFE_BUF - left_near) / SAFE_BUF
        slow = 0.35 if min(left_near, right_near) < SAFE_BUF else 1.0
        if m.state == "CRUISE":
            Vcmd = e.Vc * np.clip((fwd - e.STOP) / 0.04, 0.0, 1.0) * slow
            m.pref += (np.clip(e.Ksteer * (min(fL, e.FMAX) - min(fR, e.FMAX)), -0.5, 0.5) + safe) * fly.dt
            roll_ref = np.clip(-e.Kc * m.rd_f - e.Kd * v_lat, -np.radians(2.5), np.radians(2.5))
            if fwd < e.STOP and min(fL, fR) < e.STOP and spd < 0.03:
                m.tdir = +1 if fL >= fR else -1; m.state = "TURN"; m.turn0 = m.nose_f; m.I_y = 0.0
        else:
            Vcmd = 0.0; m.pref += m.tdir * e.YAWRATE * fly.dt
            roll_ref = np.clip(-e.KLAT * v_lat, -np.radians(2), np.radians(2))
            turned = abs(((m.nose_f - m.turn0 + np.pi) % (2 * np.pi) - np.pi))
            if fwd > e.CLEAR and turned > np.radians(20): m.state = "CRUISE"; m.I_s = 0.0; m.rd_f = 0.0
            elif turned > np.radians(175): m.tdir = -m.tdir; m.turn0 = m.nose_f
        ctrl.h_ref = e.CRUISE
        if dist_fin < e.FIN_ZONE:           # reactively reached the finish pad -> land
            m.phase = "LAND"; m.land_t = 0.0
    else:                                   # LAND: settle on the pad, then pause
        Vcmd = 0.0
        roll_ref = np.clip(-e.KLAT * v_lat, -np.radians(2), np.radians(2))
        ctrl.h_ref = max(ctrl.h_ref - e.DESC_RATE * fly.dt, 0.004)
        m.land_t += fly.dt
        if z < 0.013 and m.land_t > PAUSE_AT_FINISH:
            return True                     # done -> loop

    er = Vcmd - u_fwd; m.I_s = np.clip(m.I_s + er * fly.dt, -0.6, 0.6)
    pr = np.clip(0.30 * er + 1.1 * m.I_s, -np.radians(8), np.radians(8))
    eyaw = ((m.nose_f - m.pref + np.pi) % (2 * np.pi) - np.pi); m.I_y = np.clip(m.I_y + eyaw * fly.dt, -1.0, 1.0)
    uy = np.clip(0.14 * eyaw + 0.03 * nrate + 0.10 * m.I_y, -0.3, 0.3)
    u = ctrl.update(b, fly.dt, pitch_ref=pr, roll_ref=roll_ref, vy_ref=0.0)
    kin.set_control(thrust=u[0], roll=u[1] - e.ROLL_FF * uy, pitch=u[2], yaw=uy)
    fly.step(kin, m.t, surface=(pl + [dict(axis=2, sign=1, pos=0.0)])); m.t += fly.dt
    return m.t > TIMEOUT                     # safety cap (scales with course length)


with mujoco.viewer.launch_passive(fly.model, fly.data) as viewer:
    viewer.cam.azimuth = -60; viewer.cam.elevation = -55
    viewer.cam.distance = 0.34 * max(1.0, e.SCALE ** 0.6)
    viewer.cam.lookat[:] = [e.FINISH[0] * 0.5, e.FINISH[1] * 0.5 + 0.03, 0.0]
    viewer.opt.geomgroup[3] = 1            # draw the sensing-group walls (hidden by default)
    while viewer.is_running():
        f0 = time.time()
        for _ in range(int(0.01 / fly.dt)):
            if step():
                time.sleep(0.4); m = reset_nav(); break
        viewer.sync()
        dtw = 0.01 / SLOWMO - (time.time() - f0)
        if dtw > 0: time.sleep(dtw)