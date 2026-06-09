"""
src/kinematics.py — prescribed wing-flapping trajectories.

Pulls the stroke/feather motion (previously inlined in the experiments) into one
place. stroke = fore/aft sweep; pitch = feather/flip. rot_phase advances the
feather relative to the stroke (the rotation-timing control knob from Part 7).
"""
import numpy as np


class FlapKinematics:
    def __init__(self, f_hz, stroke_amp_deg=60.0, feather_amp_deg=45.0,
                 rot_phase=0.0, feather_sign=-1.0):
        self.W = 2 * np.pi * f_hz
        self.PHI = np.deg2rad(stroke_amp_deg)
        self.PSI = np.deg2rad(feather_amp_deg)
        self.d = rot_phase
        self.fs = feather_sign            # -1 -> lift points up (see e04)

    def stroke(self, t):   return self.PHI * np.cos(self.W * t)
    def dstroke(self, t):  return -self.PHI * self.W * np.sin(self.W * t)

    def pitch(self, t):
        return self.fs * self.PSI * np.tanh(3 * np.sin(self.W * t + self.d))

    def dpitch(self, t):
        a = self.W * t + self.d
        return self.fs * self.PSI * 3 * self.W * np.cos(a) * (1 - np.tanh(3 * np.sin(a)) ** 2)