"""
src/antenna.py — forward tactile sensing (insect antennae), modelled as ray-casts.

Aerodynamic wing-wash sensing is blind straight ahead (wings span sideways), so a
wall in FRONT is nearly invisible to it. Real insects feel ahead with antennae; we
do the same with a small fan of feeler rays cast from the head against the wall geoms
(collision group 3). Each feeler returns the distance to the nearest wall along it,
or "clear" (BIG) if nothing is within max_range. This is the ONLY thing that lets the
flyer know "wall ahead" and "which way is open"; the wing-wash still does the sides.
"""
import numpy as np, mujoco

BIG = 0.999

class Antenna:
    def __init__(self, fly, group=3, max_range=0.16):
        self.fly = fly; self.m = fly.model
        self.mask = np.zeros(6, dtype=np.uint8); self.mask[group] = 1
        self.max_range = max_range

    def feel(self, angles_deg):
        """Distances (m) to the nearest wall along each feeler angle (deg, relative to
        the nose; +left). Returns BIG for a clear feeler."""
        psi = self.fly.sense()['yaw']
        p = self.fly.x_com.copy().astype(np.float64); p[2] = max(p[2], 0.04)
        out = []
        for a in angles_deg:
            ang = psi + np.radians(a); vec = np.array([np.cos(ang), np.sin(ang), 0.0])
            gid = np.zeros(1, dtype=np.int32)
            d = mujoco.mj_ray(self.m, self.fly.data, p, vec, self.mask, 1, -1, gid)
            out.append(BIG if (d < 0 or d > self.max_range) else d)
        return out