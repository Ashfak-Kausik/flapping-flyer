"""
src/controller.py — LQG hover controller (Kalman estimator + LQR), with an
optional disturbance observer for flow-proximity sensing.

The hovering flyer is open-loop unstable (e10). We stabilise it with full-state
feedback, but two of the relevant states are not measured (vx, vy) and the body
rates carry a large wingbeat ripple. A model-based Kalman filter estimates the
hidden states and rejects the ripple by trusting the cycle-averaged model over
the noisy rate measurements. The LQR feeds back the clean estimate.

Reduced design state  z = [vx vy vz wx wy roll pitch h]. Measured outputs
y = [vz wx wy roll pitch h]. Control u = [u_thrust u_roll u_pitch].

DISTURBANCE OBSERVER (Stage 6 refinement): augment z with a roll angular-
acceleration disturbance delta entering the wx equation (wx is index 3):
    wx_dot = (model) + delta,   delta_dot = 0 + noise.
The Kalman filter then estimates delta from the part of the roll dynamics that
the known control u and the model do not explain. Because it already accounts for
the commanded u, delta_hat captures the WALL torque, not the manoeuvre roll — a
clean, decontaminated proximity signal. It is observable through the trusted roll-
angle channel even though wx itself is distrusted for ripple rejection. Setting
feedforward=True also cancels delta_hat directly (u_roll_ff = -delta_hat/B_roll),
rejecting the wall disturbance without waiting for the slow feedback loop.
"""
import numpy as np
from src.kinematics import FlapKinematics
from src import sysid


def care(A, B, Q, R):
    """Continuous-time algebraic Riccati solve via the Hamiltonian-eigenvector
    method (no scipy). Also yields the Kalman solution via (A^T, C^T, Qk, Rk)."""
    n = A.shape[0]
    H = np.block([[A, -B @ np.linalg.inv(R) @ B.T],
                  [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    U = V[:, np.argsort(w.real)[:n]]
    P = (U[n:, :] @ np.linalg.inv(U[:n, :])).real
    return 0.5 * (P + P.T)

KEEP = [0, 1, 2, 3, 4, 6, 7]          # vx vy vz wx wy roll pitch  (drop wz, yaw)
MEAS = [2, 3, 4, 5, 6, 7]             # measured rows of the 8-state z
WX = 3                                # roll-rate index in the 8-state z


class HoverController:
    def __init__(self, A, B, dt, pitch_trim=0.0, h_ref=0.05,
                 Q=(5, 5, 20, 2, 2, 250, 250, 6e4), R=(50, 50, 50),
                 Qk=(1, 1, 0.05, 30, 30, 0.02, 0.02, 1e-6),
                 Rk=(1e-3, 100, 100, 1e-4, 1e-4, 1e-7), u_limit=0.6,
                 dist_obs=True, dist_states=(3,), dist_q=2e8, feedforward=False):
        nz = len(KEEP)
        Aa = np.zeros((nz + 1, nz + 1)); Aa[:nz, :nz] = A[np.ix_(KEEP, KEEP)]; Aa[nz, 2] = 1.0
        Ba = np.zeros((nz + 1, 3)); Ba[:nz, :] = B[KEEP, :]
        C = np.zeros((len(MEAS), nz + 1))
        for r, idx in enumerate(MEAS):
            C[r, idx] = 1.0
        # LQR on the 8-state (unchanged): u = -K z
        P = care(Aa, Ba, np.diag(Q), np.diag(R))
        self.K = np.linalg.inv(np.diag(R)) @ Ba.T @ P
        self.na = Aa.shape[0]                       # 8 physical states
        self.B_roll_accel = Ba[WX, 1]               # roll authority in rad/s^2 per unit
        self.Ba_phys = Ba.copy()                    # 8x3, for feed-forward B lookups
        self.CTRL_COL = {2: 0, 3: 1, 4: 2}          # vz<-thrust, wx<-roll, wy<-pitch
        self.dist_obs = dist_obs
        self.dist_states = list(dist_states)
        self.ff = 1.0 if feedforward else 0.0

        if dist_obs:                                # augment estimator with a disturbance per axis
            na = self.na; ds = self.dist_states; nd = len(ds)
            Aaug = np.zeros((na + nd, na + nd)); Aaug[:na, :na] = Aa
            for k, idx in enumerate(ds): Aaug[idx, na + k] = 1.0
            Baug = np.zeros((na + nd, 3)); Baug[:na, :] = Ba
            Caug = np.zeros((C.shape[0], na + nd)); Caug[:, :na] = C
            Qk_aug = np.concatenate([np.array(Qk, float), np.full(nd, dist_q)])
            Pf = care(Aaug.T, Caug.T, np.diag(Qk_aug), np.diag(Rk))
            self.L = Pf @ Caug.T @ np.linalg.inv(np.diag(Rk))
            self.Aa, self.Ba, self.C = Aaug, Baug, Caug
            self.dist_idx = {idx: na + k for k, idx in enumerate(ds)}
        else:
            Pf = care(Aa.T, C.T, np.diag(Qk), np.diag(Rk))
            self.L = Pf @ C.T @ np.linalg.inv(np.diag(Rk))
            self.Aa, self.Ba, self.C = Aa, Ba, C

        self.pitch_trim = pitch_trim; self.h_ref = h_ref; self.u_limit = u_limit
        self.reset()

    def reset(self):
        self.xh = np.zeros(self.Aa.shape[0])

    def update(self, sense, dt, vy_ref=0.0):
        """One control step. `vy_ref` commands a lateral velocity (m/s)."""
        y = np.array([sense['vz'], sense['wx'], sense['wy'],
                      sense['roll'], sense['pitch'], sense['height'] - self.h_ref])
        z_ref = np.zeros(self.na); z_ref[1] = vy_ref
        u_ff = np.zeros(3)
        if self.dist_obs:
            for idx, ai in self.dist_idx.items():
                col = self.CTRL_COL[idx]; u_ff[col] += -self.ff * self.xh[ai] / self.Ba_phys[idx, col]
        u = np.clip(-self.K @ (self.xh[:self.na] - z_ref)
                    + np.array([0.0, 0.0, self.pitch_trim]) + u_ff,
                    -self.u_limit, self.u_limit)
        self.xh = self.xh + dt * (self.Aa @ self.xh + self.Ba @ u + self.L @ (y - self.C @ self.xh))
        return u

    @property
    def vy_est(self):
        return self.xh[1]

    @property
    def roll_dist(self):
        """Estimated WALL roll-acceleration disturbance (rad/s^2); sign = side."""
        return self.xh[self.dist_idx[3]] if self.dist_obs and 3 in self.dist_idx else 0.0

    @property
    def pitch_dist(self):
        """Estimated fore/aft surface pitch-acceleration disturbance (rad/s^2)."""
        return self.xh[self.dist_idx[4]] if self.dist_obs and 4 in self.dist_idx else 0.0

    @property
    def floor_dist(self):
        """Estimated FLOOR vertical-acceleration disturbance (m/s^2): ground-effect
        lift as an acceleration, = dFz/M. Larger => closer to a surface below."""
        return self.xh[self.dist_idx[2]] if self.dist_obs and 2 in self.dist_idx else 0.0


def design(fly, f_hz=80, feather=45, h_ref=0.05, **kw):
    amp = sysid.hover_amplitude(fly, f_hz=f_hz, feather=feather)
    kin = FlapKinematics(f_hz=f_hz, stroke_amp_deg=amp, feather_amp_deg=feather)
    D, w0 = sysid.stability_derivatives(fly, kin); A = sysid.assemble_A(D, fly.M, fly.I)
    Bw = sysid.control_derivatives(fly, kin); B = sysid.assemble_B(Bw, fly.M, fly.I)
    trim = -w0[4] / Bw[4, 2]
    ctrl = HoverController(A, B, fly.dt, pitch_trim=trim, h_ref=h_ref, **kw)
    return ctrl, kin, dict(amp=amp, A=A, B=B, w0=w0, Bw=Bw)