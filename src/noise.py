"""
src/noise.py — realistic sensor-noise model for sim-to-real honesty.

The flyer's real sensors are noisy. We add Gaussian noise to:
  * the ANTENNA range readings (a real feeler / short-range rangefinder isn't exact),
  * the IMU state from sense() — attitude, angular rates, velocity, yaw, height.
The wing-wash proximity signal (ctrl.roll_dist) is NOT noised directly: it is derived by
the controller's disturbance observer from the IMU state, so noising the IMU makes roll_dist
noisy through the observer dynamics -- the honest propagation path.

One scalar `level` scales every sigma together so we can sweep noise vs. navigation quality.
The TRUE flyer state (position, dynamics) is never touched -- only what the flyer *measures*.
"""
import numpy as np

class NoiseModel:
    # baseline (level=1.0) one-sigma values -- credible MEMS-class for a small flyer
    S_RANGE = 0.002    # antenna range            (m)
    S_ATT   = 0.005    # pitch / roll estimate    (rad,  ~0.3 deg)
    S_GYRO  = 0.02     # body rates wx,wy,wz       (rad/s, ~1.1 deg/s)
    S_VEL   = 0.005    # velocity vx,vy,vz         (m/s,  5 mm/s)
    S_YAW   = 0.005    # heading estimate          (rad)
    S_H     = 0.001    # height                    (m)

    def __init__(self, level=1.0, seed=0):
        self.L = float(level); self.rng = np.random.default_rng(seed)

    def _n(self, sigma):
        return self.rng.normal(0.0, sigma * self.L) if self.L > 0 else 0.0

    def sense(self, s):
        """Return a noisy copy of the sense() dict (IMU)."""
        s = dict(s)
        s['pitch'] += self._n(self.S_ATT); s['roll'] += self._n(self.S_ATT); s['yaw'] += self._n(self.S_YAW)
        s['wx'] += self._n(self.S_GYRO);   s['wy'] += self._n(self.S_GYRO);  s['wz'] += self._n(self.S_GYRO)
        s['vx'] += self._n(self.S_VEL);    s['vy'] += self._n(self.S_VEL);   s['vz'] += self._n(self.S_VEL)
        s['height'] += self._n(self.S_H)
        return s

    def feel(self, dists, BIG=0.999):
        """Add range noise to finite feeler readings; a clear feeler stays clear."""
        return [d if d >= BIG else max(0.0, d + self._n(self.S_RANGE)) for d in dists]