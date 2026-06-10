"""
src/sysid.py — system identification of the cycle-averaged body dynamics.

We linearise the hovering flyer about its trim. Two pieces:
  * B (control authority): measured separately in e09.
  * A (open-loop dynamics): measured here. We hold the body level at the hover
    pose, impose a small body twist (one of vx,vy,vz, wx,wy,wz), flap at the
    hover kinematics, and read the cycle-averaged aero wrench about the system
    CoM. Finite-differencing gives the 6x6 aero stability-derivative matrix D.
    A is then assembled from D, the rigid-body mass/inertia, the thrust-tilt
    (gravity) coupling, and the attitude kinematics.

State x = [vx, vy, vz, wx, wy, wz, roll, pitch, yaw]  (world-frame velocities &
rates at the level trim; attitude as small tilt angles). eig(A) gives the
open-loop modes (the instability we must control).
"""
import numpy as np
import mujoco
from src.aero import wing_aero
from src.kinematics import FlapKinematics

G = 9.81


def _sys_com(fly):
    c = np.zeros(3)
    for i in range(1, fly.model.nbody):
        c += fly.model.body_mass[i] * fly.data.xipos[i]
    return c / fly.M


def cycle_avg_wrench(fly, kin, twist, ncyc=8, navg=4):
    """Cycle-averaged aero wrench [Fx,Fy,Fz,Tx,Ty,Tz] about the system CoM (world
    frame), body held level at the hover pose while carrying `twist`
    = [vx,vy,vz, wx,wy,wz] (linear world, angular body; identical at level)."""
    m, d = fly.model, fly.data
    DT = fly.dt
    period = 2 * np.pi / kin.W
    N = int(ncyc * period / DT); t0 = (ncyc - navg) * period
    vn = {s: None for s in "RL"}; acc = []
    for i in range(N):
        t = i * DT
        d.qpos[fly._qf:fly._qf + 3] = [0.0, 0.0, 0.05]
        d.qpos[fly._qf + 3:fly._qf + 7] = [1.0, 0.0, 0.0, 0.0]
        d.qvel[fly._vf:fly._vf + 6] = twist
        for w in "RL":
            st, dst, pt, dpt = kin.signals(t, w)
            d.qpos[fly._j[f"stroke_{w}"][0]] = st; d.qvel[fly._j[f"stroke_{w}"][1]] = dst
            d.qpos[fly._j[f"pitch_{w}"][0]]  = pt; d.qvel[fly._j[f"pitch_{w}"][1]]  = dpt
        mujoco.mj_forward(m, d)
        com = _sys_com(fly); F = np.zeros(3); T = np.zeros(3)
        for s, bid in fly.wings.items():
            Fa, Ta, vn[s], _ = wing_aero(m, d, bid, fly.strips[s], vn[s], DT)
            F += Fa; T += Ta + np.cross(d.xipos[bid] - com, Fa)
        if t >= t0:
            acc.append(np.concatenate([F, T]))
    return np.mean(acc, 0)


def hover_amplitude(fly, f_hz=80, feather=45, lo=50.0, hi=95.0, iters=22):
    """Stroke amplitude (deg) whose cycle-averaged vertical force equals weight."""
    mid = 0.5 * (lo + hi)
    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        k = FlapKinematics(f_hz=f_hz, stroke_amp_deg=mid, feather_amp_deg=feather)
        if cycle_avg_wrench(fly, k, np.zeros(6))[2] > fly.M * G:
            hi = mid
        else:
            lo = mid
    return mid


def stability_derivatives(fly, kin, dv=0.05, dw=5.0):
    """6x6 aero derivative matrix D[wrench, twist] by central difference, plus
    the baseline (trim) wrench w0 at zero twist."""
    pert = [dv, dv, dv, dw, dw, dw]
    D = np.zeros((6, 6))
    for c in range(6):
        tp = np.zeros(6); tp[c] = pert[c]
        tm = np.zeros(6); tm[c] = -pert[c]
        D[:, c] = (cycle_avg_wrench(fly, kin, tp) - cycle_avg_wrench(fly, kin, tm)) / (2 * pert[c])
    w0 = cycle_avg_wrench(fly, kin, np.zeros(6))
    return D, w0


def assemble_A(D, M, I):
    """Assemble the 9x9 open-loop dynamics matrix from the aero derivatives D,
    mass M, inertia I (about CoM). State [vx vy vz wx wy wz roll pitch yaw]."""
    Iinv = np.linalg.inv(I)
    A = np.zeros((9, 9))
    A[0:3, 0:6] = D[0:3, 0:6] / M           # v_dot from aero force derivatives
    A[3:6, 0:6] = Iinv @ D[3:6, 0:6]        # w_dot from aero torque derivatives
    A[0, 7] += G                            # +pitch tilts trim thrust -> +ax
    A[1, 6] += G                            # +roll  tilts trim thrust -> +ay
    # attitude kinematics from d(up)/dt = omega x up: pitch_dot=+wy, roll_dot=-wx
    A[6, 3] = -1.0; A[7, 4] = 1.0; A[8, 5] = 1.0   # roll/pitch/yaw_dot = -wx/+wy/+wz
    return A


def control_derivatives(fly, kin, du=0.05):
    """6x3 wrench-vs-control matrix Bw at the hover trim (columns: u_thrust,
    u_roll, u_pitch), measured the same way as the stability derivatives."""
    Bw = np.zeros((6, 3))
    for j, knob in enumerate(("thrust", "roll", "pitch")):
        kin.set_control(**{knob: +du}); wp = cycle_avg_wrench(fly, kin, np.zeros(6))
        kin.set_control(**{knob: -du}); wm = cycle_avg_wrench(fly, kin, np.zeros(6))
        Bw[:, j] = (wp - wm) / (2 * du)
    kin.set_control()
    return Bw


def assemble_B(Bw, M, I):
    """Map the control-wrench derivatives into state space (9x3), state order
    [vx vy vz wx wy wz roll pitch yaw]."""
    Iinv = np.linalg.inv(I)
    B = np.zeros((9, 3))
    B[0:3, :] = Bw[0:3, :] / M
    B[3:6, :] = Iinv @ Bw[3:6, :]
    return B