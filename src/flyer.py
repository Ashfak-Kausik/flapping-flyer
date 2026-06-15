"""
src/flyer.py — the simulatable flyer: a free rigid body + injected aerodynamics.

Single rigid body (mass M, inertia I about the CoM); wings are massless aero
surfaces whose motion is prescribed only to compute the quasi-steady force, which
is applied to the body and advanced with our own semi-implicit integrator. See
docs/03 for why (the prescribed-wings-on-free-body inertial-torque artifact).

Stage 6 addition: step() takes an optional `surface`. With surface=None the aero
is exactly the validated free-air model (Stages 2-5 unchanged). With a surface
provided, the translational lift gets the ground/wall-effect factor from
src/ground_effect.py, so the flyer can fly near surfaces and feel them.
"""
import numpy as np
import mujoco
from src.aero import strips_for_wing, wing_aero, wing_params_from_model
from src.ground_effect import wing_aero_ge

G = 9.81


def make_free_model(model_path):
    spec = mujoco.MjSpec.from_file(str(model_path))
    spec.body("thorax").add_freejoint()
    return spec.compile()


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


class Flyer:
    def __init__(self, model_path, n_strips=20):
        self.model = make_free_model(model_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.wings = {s: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"wing_{s}")
                      for s in "RL"}
        self.thorax = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "thorax")
        self.strips = {s: strips_for_wing(self.model, s, n_strips) for s in "RL"}
        self.R = wing_params_from_model(self.model)["r_tip"]      # wing length (ground effect)
        self._j = {}
        for n in ("stroke_R", "pitch_R", "stroke_L", "pitch_L"):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            self._j[n] = (self.model.jnt_qposadr[jid], self.model.jnt_dofadr[jid])
        jadr = self.model.body_jntadr[self.thorax]
        self._qf = self.model.jnt_qposadr[jadr]
        self._vf = self.model.jnt_dofadr[jadr]
        self.vn = {s: None for s in "RL"}
        self._compute_inertial()

    def _compute_inertial(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self._qf:self._qf + 3] = [0.0, 0.0, 0.05]
        d.qpos[self._qf + 3:self._qf + 7] = [1.0, 0.0, 0.0, 0.0]
        for n in ("stroke_R", "pitch_R", "stroke_L", "pitch_L"):
            d.qpos[self._j[n][0]] = 0.0
        mujoco.mj_forward(m, d)
        self.M = float(sum(m.body_mass[1:]))
        com = np.zeros(3)
        for i in range(1, m.nbody):
            com += m.body_mass[i] * d.xipos[i]
        com /= self.M
        I = np.zeros((3, 3))
        for i in range(1, m.nbody):
            R = d.ximat[i].reshape(3, 3)
            Ii = R @ np.diag(m.body_inertia[i]) @ R.T
            r = d.xipos[i] - com
            I += Ii + m.body_mass[i] * (r @ r * np.eye(3) - np.outer(r, r))
        self.I = I; self.Iinv = np.linalg.inv(I)
        self.offset_b = com - d.xpos[self.thorax]

    def _quat2mat(self):
        R = np.zeros(9); mujoco.mju_quat2Mat(R, self.q); return R.reshape(3, 3)

    def _set_body_state(self):
        d = self.data; R = self._quat2mat()
        origin = self.x_com - R @ self.offset_b
        v_origin = self.v + np.cross(R @ self.w, origin - self.x_com)
        d.qpos[self._qf:self._qf + 3] = origin
        d.qpos[self._qf + 3:self._qf + 7] = self.q
        d.qvel[self._vf:self._vf + 3] = v_origin
        d.qvel[self._vf + 3:self._vf + 6] = self.w

    def reset(self, kin=None, height=0.05, pitch_deg=0.0, roll_deg=0.0, y=0.0):
        mujoco.mj_resetData(self.model, self.data)
        qp = np.array([np.cos(np.deg2rad(pitch_deg) / 2), 0, np.sin(np.deg2rad(pitch_deg) / 2), 0])
        qr = np.array([np.cos(np.deg2rad(roll_deg) / 2), np.sin(np.deg2rad(roll_deg) / 2), 0, 0])
        self.q = _qmul(qp, qr); self.q /= np.linalg.norm(self.q)
        R = self._quat2mat()
        self.x_com = np.array([0.0, y, height]) + R @ self.offset_b
        self.v = np.zeros(3); self.w = np.zeros(3)
        self.vn = {s: None for s in "RL"}
        if kin is not None:
            self._set_body_state(); self._prescribe_wings(kin, 0.0)
            mujoco.mj_forward(self.model, self.data)

    def _prescribe_wings(self, kin, t):
        d = self.data
        for wing in ("R", "L"):
            st, dst, pt, dpt = kin.signals(t, wing)
            d.qpos[self._j[f"stroke_{wing}"][0]] = st;  d.qvel[self._j[f"stroke_{wing}"][1]] = dst
            d.qpos[self._j[f"pitch_{wing}"][0]]  = pt;  d.qvel[self._j[f"pitch_{wing}"][1]]  = dpt

    def step(self, kin, t, surface=None):
        d = self.data
        self._set_body_state()
        self._prescribe_wings(kin, t)
        mujoco.mj_forward(self.model, self.data)
        R = d.xmat[self.thorax].reshape(3, 3)
        F_w = np.zeros(3); T_w = np.zeros(3)
        for s, bid in self.wings.items():
            if surface is None:
                F, T, self.vn[s], _ = wing_aero(self.model, d, bid, self.strips[s], self.vn[s], self.dt)
            else:
                F, T, self.vn[s], _ = wing_aero_ge(self.model, d, bid, self.strips[s],
                                                   self.vn[s], self.dt, surface, self.R)
            F_w += F
            T_w += T + np.cross(d.xipos[bid] - self.x_com, F)
        a = (F_w + np.array([0.0, 0.0, -self.M * G])) / self.M
        T_b = R.T @ T_w
        wdot = self.Iinv @ (T_b - np.cross(self.w, self.I @ self.w))
        self.v = self.v + a * self.dt
        self.w = self.w + wdot * self.dt
        self.x_com = self.x_com + self.v * self.dt
        self.q = self.q + 0.5 * _qmul(self.q, np.array([0.0, *self.w])) * self.dt
        self.q /= np.linalg.norm(self.q)
        self._set_body_state(); self._prescribe_wings(kin, t + self.dt)
        mujoco.mj_forward(self.model, self.data)

    def attitude(self):
        R = self.data.xmat[self.thorax].reshape(3, 3)
        up = R[:, 2]
        return np.arctan2(up[0], up[2]), np.arctan2(up[1], up[2]), self.data.xpos[self.thorax][2]

    def sense(self):
        pitch, roll, height = self.attitude()
        R = self.data.xmat[self.thorax].reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])          # heading from the body forward axis
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.thorax, vel6, 0)
        return dict(pitch=pitch, roll=roll, height=height, yaw=yaw,
                    wx=vel6[0], wy=vel6[1], wz=vel6[2],
                    vx=vel6[3], vy=vel6[4], vz=vel6[5])
"""
src/flyer.py — the simulatable flyer: a free rigid body + injected aerodynamics.

Single rigid body (mass M, inertia I about the CoM); wings are massless aero
surfaces whose motion is prescribed only to compute the quasi-steady force, which
is applied to the body and advanced with our own semi-implicit integrator. See
docs/03 for why (the prescribed-wings-on-free-body inertial-torque artifact).

Stage 6 addition: step() takes an optional `surface`. With surface=None the aero
is exactly the validated free-air model (Stages 2-5 unchanged). With a surface
provided, the translational lift gets the ground/wall-effect factor from
src/ground_effect.py, so the flyer can fly near surfaces and feel them.
"""
import numpy as np
import mujoco
from src.aero import strips_for_wing, wing_aero, wing_params_from_model
from src.ground_effect import wing_aero_ge

G = 9.81


def make_free_model(model_path):
    spec = mujoco.MjSpec.from_file(str(model_path))
    spec.body("thorax").add_freejoint()
    return spec.compile()


def _qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1*w2 - x1*x2 - y1*y2 - z1*z2,
                     w1*x2 + x1*w2 + y1*z2 - z1*y2,
                     w1*y2 - x1*z2 + y1*w2 + z1*x2,
                     w1*z2 + x1*y2 - y1*x2 + z1*w2])


class Flyer:
    def __init__(self, model_path, n_strips=20):
        self.model = make_free_model(model_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.wings = {s: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"wing_{s}")
                      for s in "RL"}
        self.thorax = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "thorax")
        self.strips = {s: strips_for_wing(self.model, s, n_strips) for s in "RL"}
        self.R = wing_params_from_model(self.model)["r_tip"]      # wing length (ground effect)
        self._j = {}
        for n in ("stroke_R", "pitch_R", "stroke_L", "pitch_L"):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            self._j[n] = (self.model.jnt_qposadr[jid], self.model.jnt_dofadr[jid])
        jadr = self.model.body_jntadr[self.thorax]
        self._qf = self.model.jnt_qposadr[jadr]
        self._vf = self.model.jnt_dofadr[jadr]
        self.vn = {s: None for s in "RL"}
        self._compute_inertial()

    def _compute_inertial(self):
        m, d = self.model, self.data
        mujoco.mj_resetData(m, d)
        d.qpos[self._qf:self._qf + 3] = [0.0, 0.0, 0.05]
        d.qpos[self._qf + 3:self._qf + 7] = [1.0, 0.0, 0.0, 0.0]
        for n in ("stroke_R", "pitch_R", "stroke_L", "pitch_L"):
            d.qpos[self._j[n][0]] = 0.0
        mujoco.mj_forward(m, d)
        self.M = float(sum(m.body_mass[1:]))
        com = np.zeros(3)
        for i in range(1, m.nbody):
            com += m.body_mass[i] * d.xipos[i]
        com /= self.M
        I = np.zeros((3, 3))
        for i in range(1, m.nbody):
            R = d.ximat[i].reshape(3, 3)
            Ii = R @ np.diag(m.body_inertia[i]) @ R.T
            r = d.xipos[i] - com
            I += Ii + m.body_mass[i] * (r @ r * np.eye(3) - np.outer(r, r))
        self.I = I; self.Iinv = np.linalg.inv(I)
        self.offset_b = com - d.xpos[self.thorax]

    def _quat2mat(self):
        R = np.zeros(9); mujoco.mju_quat2Mat(R, self.q); return R.reshape(3, 3)

    def _set_body_state(self):
        d = self.data; R = self._quat2mat()
        origin = self.x_com - R @ self.offset_b
        v_origin = self.v + np.cross(R @ self.w, origin - self.x_com)
        d.qpos[self._qf:self._qf + 3] = origin
        d.qpos[self._qf + 3:self._qf + 7] = self.q
        d.qvel[self._vf:self._vf + 3] = v_origin
        d.qvel[self._vf + 3:self._vf + 6] = self.w

    def reset(self, kin=None, height=0.05, pitch_deg=0.0, roll_deg=0.0, y=0.0):
        mujoco.mj_resetData(self.model, self.data)
        qp = np.array([np.cos(np.deg2rad(pitch_deg) / 2), 0, np.sin(np.deg2rad(pitch_deg) / 2), 0])
        qr = np.array([np.cos(np.deg2rad(roll_deg) / 2), np.sin(np.deg2rad(roll_deg) / 2), 0, 0])
        self.q = _qmul(qp, qr); self.q /= np.linalg.norm(self.q)
        R = self._quat2mat()
        self.x_com = np.array([0.0, y, height]) + R @ self.offset_b
        self.v = np.zeros(3); self.w = np.zeros(3)
        self.vn = {s: None for s in "RL"}
        if kin is not None:
            self._set_body_state(); self._prescribe_wings(kin, 0.0)
            mujoco.mj_forward(self.model, self.data)

    def _prescribe_wings(self, kin, t):
        d = self.data
        for wing in ("R", "L"):
            st, dst, pt, dpt = kin.signals(t, wing)
            d.qpos[self._j[f"stroke_{wing}"][0]] = st;  d.qvel[self._j[f"stroke_{wing}"][1]] = dst
            d.qpos[self._j[f"pitch_{wing}"][0]]  = pt;  d.qvel[self._j[f"pitch_{wing}"][1]]  = dpt

    def step(self, kin, t, surface=None):
        d = self.data
        self._set_body_state()
        self._prescribe_wings(kin, t)
        mujoco.mj_forward(self.model, self.data)
        R = d.xmat[self.thorax].reshape(3, 3)
        F_w = np.zeros(3); T_w = np.zeros(3)
        for s, bid in self.wings.items():
            if surface is None:
                F, T, self.vn[s], _ = wing_aero(self.model, d, bid, self.strips[s], self.vn[s], self.dt)
            else:
                F, T, self.vn[s], _ = wing_aero_ge(self.model, d, bid, self.strips[s],
                                                   self.vn[s], self.dt, surface, self.R)
            F_w += F
            T_w += T + np.cross(d.xipos[bid] - self.x_com, F)
        a = (F_w + np.array([0.0, 0.0, -self.M * G])) / self.M
        T_b = R.T @ T_w
        wdot = self.Iinv @ (T_b - np.cross(self.w, self.I @ self.w))
        self.v = self.v + a * self.dt
        self.w = self.w + wdot * self.dt
        self.x_com = self.x_com + self.v * self.dt
        self.q = self.q + 0.5 * _qmul(self.q, np.array([0.0, *self.w])) * self.dt
        self.q /= np.linalg.norm(self.q)
        self._set_body_state(); self._prescribe_wings(kin, t + self.dt)
        mujoco.mj_forward(self.model, self.data)

    def attitude(self):
        R = self.data.xmat[self.thorax].reshape(3, 3)
        up = R[:, 2]
        return np.arctan2(up[0], up[2]), np.arctan2(up[1], up[2]), self.data.xpos[self.thorax][2]

    def sense(self):
        pitch, roll, height = self.attitude()
        R = self.data.xmat[self.thorax].reshape(3, 3)
        yaw = np.arctan2(R[1, 0], R[0, 0])          # heading from the body forward axis
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.thorax, vel6, 0)
        return dict(pitch=pitch, roll=roll, height=height, yaw=yaw,
                    wx=vel6[0], wy=vel6[1], wz=vel6[2],
                    vx=vel6[3], vy=vel6[4], vz=vel6[5])