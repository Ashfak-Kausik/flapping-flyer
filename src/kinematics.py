"""
src/kinematics.py — prescribed wing flapping, with control inputs.

Base flap = stroke (fore/aft sweep) + feather (flip). On top sit three control
inputs the controller will modulate (Stage 4):
    u_thrust : symmetric stroke-amplitude change   -> vertical force
    u_roll   : differential amplitude (R vs L)      -> roll torque (about +x)
    u_pitch  : symmetric mean-stroke offset          -> pitch torque (about +y)
signals(t, wing) returns the JOINT angle/rate for that wing, control included.
The left wing's sweep is mirrored (-cos) so symmetric flapping is symmetric.
"""
import numpy as np


class FlapKinematics:
    def __init__(self, f_hz, stroke_amp_deg=60.0, feather_amp_deg=45.0,
                 rot_phase=0.0, feather_sign=-1.0):
        self.W = 2 * np.pi * f_hz
        self.PHI = np.deg2rad(stroke_amp_deg)
        self.PSI = np.deg2rad(feather_amp_deg)
        self.d = rot_phase
        self.fs = feather_sign
        self.u_thrust = 0.0
        self.u_roll = 0.0
        self.u_pitch = 0.0

    def set_control(self, thrust=0.0, roll=0.0, pitch=0.0):
        self.u_thrust, self.u_roll, self.u_pitch = thrust, roll, pitch

    def _amp(self, wing):                       # stroke amplitude for this wing
        s = 1.0 if wing == "R" else -1.0
        return self.PHI * (1.0 + self.u_thrust + s * self.u_roll)

    def _offset(self, wing):                    # mean-stroke offset (pitch knob)
        s = 1.0 if wing == "R" else -1.0
        return s * self.u_pitch

    def signals(self, t, wing):
        """Return (stroke_angle, stroke_rate, pitch_angle, pitch_rate) for the
        named wing ('R' or 'L'), as JOINT values (mirroring already applied)."""
        mirror = 1.0 if wing == "R" else -1.0
        A = self._amp(wing)
        off = self._offset(wing)
        stroke = mirror * A * np.cos(self.W * t) + off
        dstroke = -mirror * A * self.W * np.sin(self.W * t)
        a = self.W * t + self.d
        pitch = self.fs * self.PSI * np.tanh(3 * np.sin(a))
        dpitch = self.fs * self.PSI * 3 * self.W * np.cos(a) * (1 - np.tanh(3 * np.sin(a)) ** 2)
        return stroke, dstroke, pitch, dpitch