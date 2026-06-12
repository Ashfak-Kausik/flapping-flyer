"""
src/avoidance.py — flow-based wall avoidance: an outer loop on the LQG.

The flyer senses a wall with NO dedicated proximity sensor: the wall's aerodynamic
ground/wall effect disturbs roll, the LQG cancels it, and the cancelling effort
shows up as a steady bias in the controller's own u_roll command (validated in
e14). This module reads that residual (low-passed over ~a wingbeat) and commands a
lateral velocity into the LQG to hold a chosen standoff — i.e. regulate the
residual magnitude to a setpoint. Sign of the residual = which side the wall is
on, so the retreat/approach direction is automatic.

CAVEAT (honest): while the flyer is actively translating, u_roll also contains the
roll used to *make* that translation, so the residual is partly contaminated by
the manoeuvre. The residual->standoff map is therefore monotonic and repeatable
but not 1:1 with the setpoint; a disturbance observer would decouple them. The
proximity SENSING itself is validated quasi-statically (e14); this is the reactive
layer built on top.
"""
import numpy as np


class WallStandoff:
    def __init__(self, r_setpoint, dt, gain=8.0, v_max=0.12, tau=0.025):
        self.rstar = r_setpoint
        self.K = gain
        self.vmax = v_max
        self.a = dt / tau              # EMA coefficient for the residual low-pass
        self.r = 0.0                   # low-passed u_roll residual (the proximity signal)

    def command(self):
        """Lateral-velocity command for the LQG (m/s). Retreat if closer than the
        setpoint, approach if farther; direction set by the residual's sign."""
        if abs(self.r) < 1e-5:
            return 0.0
        return float(np.clip(np.sign(self.r) * self.K * (abs(self.r) - self.rstar),
                             -self.vmax, self.vmax))

    def observe(self, u_roll):
        """Feed in this step's roll command to update the proximity estimate."""
        self.r += self.a * (u_roll - self.r)

    @property
    def proximity(self):
        return abs(self.r)            # larger => closer to the wall