"""
src/safety.py — wall clearance & crash detection for sense-only walls.

MuJoCo rigid-contact does not fire reliably for our ~50 um flyer geoms (it falls
through the floor), so walls are sense-only and the flyer can't physically crash.
For honest pass/fail testing we instead MEASURE clearance: cast a ring of rays
around the centre of mass and take the nearest wall. If the nearest wall is closer
than the wingtip reach, a wing would strike it -> we log a CRASH. This makes a
wall-hit a real, recorded failure even without rigid contact.
"""
import numpy as np, mujoco

WINGREACH = 0.0133          # wingtip reach from CoM (m); contact if a wall is nearer than this

class Clearance:
    def __init__(self, fly, group=3, n_rays=36):
        self.fly = fly; self.m = fly.model
        self.mask = np.zeros(6, dtype=np.uint8); self.mask[group] = 1
        self.angles = np.linspace(0.0, 2*np.pi, n_rays, endpoint=False)

    def min_clearance(self):
        """Distance (m) from CoM to the nearest wall in the horizontal plane."""
        p = self.fly.x_com.copy().astype(np.float64); p[2] = max(p[2], 0.02)
        best = 1e3
        for a in self.angles:
            vec = np.array([np.cos(a), np.sin(a), 0.0]); gid = np.zeros(1, dtype=np.int32)
            d = mujoco.mj_ray(self.m, self.fly.data, p, vec, self.mask, 1, -1, gid)
            if 0 <= d < best: best = d
        return best

    def crashed(self):
        return self.min_clearance() < WINGREACH