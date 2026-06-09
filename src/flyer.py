"""
src/flyer.py — the simulatable flyer: free body + injected aerodynamics.

make_free_model() derives a FREE-FLYING model from the (clamped) flyer.xml by
adding a free joint to the thorax via the MuJoCo spec API — so flyer.xml stays
the single source of truth for the body.

Flyer.step() prescribes the wing motion, computes the quasi-steady aero with
wing_aero(), injects it into MuJoCo via xfrc_applied on the wing bodies (MuJoCo
then transmits it through the joints to the free body), and advances one step.
The wings are re-prescribed each step (override) so they follow the commanded
flap exactly while the body responds to aero + gravity.
"""
import numpy as np
import mujoco
from src.aero import strips_for_wing, wing_aero


def make_free_model(model_path):
    spec = mujoco.MjSpec.from_file(str(model_path))
    spec.body("thorax").add_freejoint()
    return spec.compile()


class Flyer:
    def __init__(self, model_path, n_strips=20):
        self.model = make_free_model(model_path)
        self.data = mujoco.MjData(self.model)
        self.dt = self.model.opt.timestep
        self.wings = {s: mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"wing_{s}")
                      for s in "RL"}
        self.thorax = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "thorax")
        self.strips = {s: strips_for_wing(self.model, s, n_strips) for s in "RL"}
        self._j = {}
        for n in ("stroke_R", "pitch_R", "stroke_L", "pitch_L"):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)
            self._j[n] = (self.model.jnt_qposadr[jid], self.model.jnt_dofadr[jid])
        self.vn = {s: None for s in "RL"}

    def reset(self, kin=None, height=0.05):
        mujoco.mj_resetData(self.model, self.data)
        # free joint qpos = [x y z, quat wxyz]; set start height, level attitude
        self.data.qpos[0:3] = [0.0, 0.0, height]
        self.data.qpos[3:7] = [1.0, 0.0, 0.0, 0.0]
        if kin is not None:
            self._prescribe_wings(kin, 0.0)     # start wings on-trajectory (no jump)
        self.vn = {s: None for s in "RL"}
        mujoco.mj_forward(self.model, self.data)

    def _prescribe_wings(self, kin, t):
        d = self.data
        for wing in ("R", "L"):
            st, dst, pt, dpt = kin.signals(t, wing)
            d.qpos[self._j[f"stroke_{wing}"][0]] = st;  d.qvel[self._j[f"stroke_{wing}"][1]] = dst
            d.qpos[self._j[f"pitch_{wing}"][0]]  = pt;  d.qvel[self._j[f"pitch_{wing}"][1]]  = dpt

    def step(self, kin, t):
        self._prescribe_wings(kin, t)
        mujoco.mj_forward(self.model, self.data)        # refresh kinematics for aero
        # sum each wing's aero into ONE wrench on the thorax CoM. Applying force
        # to the near-massless wing bodies gives them huge accelerations (F/I);
        # carrying the wrench to the body is both stabler and the net effect we want.
        thorax_com = self.data.xipos[self.thorax]
        F_tot = np.zeros(3); T_tot = np.zeros(3)
        for s, bid in self.wings.items():
            F, T, self.vn[s], _ = wing_aero(self.model, self.data, bid,
                                            self.strips[s], self.vn[s], self.dt)
            wing_com = self.data.xipos[bid]
            F_tot += F
            T_tot += T + np.cross(wing_com - thorax_com, F)
        self.data.xfrc_applied[:] = 0.0
        self.data.xfrc_applied[self.thorax, :3] = F_tot
        self.data.xfrc_applied[self.thorax, 3:] = T_tot
        mujoco.mj_step(self.model, self.data)
        self._prescribe_wings(kin, t + self.dt)         # keep wings on trajectory

    def attitude(self):
        """Return (pitch, roll, yaw_tilt) of the body in radians, plus height.
        pitch/roll = tilt of the body up-axis toward +x / +y."""
        R = self.data.xmat[self.thorax].reshape(3, 3)
        up = R[:, 2]                                    # body up-axis in world
        pitch = np.arctan2(up[0], up[2])
        roll = np.arctan2(up[1], up[2])
        height = self.data.xpos[self.thorax][2]
        return pitch, roll, height

    def sense(self):
        """Body-state sensor (stand-in for the halteres + an altimeter).
        Returns attitude (pitch, roll), body angular rates (wx, wy, wz) in the
        WORLD frame, height, and vertical speed."""
        pitch, roll, height = self.attitude()
        vel6 = np.zeros(6)
        mujoco.mj_objectVelocity(self.model, self.data, mujoco.mjtObj.mjOBJ_BODY,
                                 self.thorax, vel6, 0)
        wx, wy, wz = vel6[:3]
        vz = vel6[5]
        return dict(pitch=pitch, roll=roll, height=height,
                    wx=wx, wy=wy, wz=wz, vz=vz)