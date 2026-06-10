"""
src/controller.py — LQG hover controller (Kalman estimator + LQR).

The hovering flyer is open-loop unstable (e10). We stabilise it with full-state
feedback, but two of the relevant states are not measured: horizontal velocity
vx, vy (no airspeed sensor). And the measured body rates carry a large wingbeat
ripple (~+/-17 rad/s). A model-based Kalman filter solves both at once: it
estimates vx, vy from the dynamics and rejects the ripple by trusting the
cycle-averaged model over the noisy rate measurements. The LQR then feeds back
the clean full-state estimate.

Reduced design state  z = [vx vy vz wx wy roll pitch h]   (yaw dropped: it is
uncontrollable with our three knobs and only drifts neutrally). Measured outputs
y = [vz wx wy roll pitch h]. Control u = [u_thrust u_roll u_pitch].
"""
import numpy as np
from src.kinematics import FlapKinematics
from src import sysid


def care(A, B, Q, R):
    """Solve the continuous-time algebraic Riccati equation
        A^T P + P A - P B R^-1 B^T P + Q = 0
    for the stabilising symmetric P, using the Hamiltonian-eigenvector method
    (no scipy needed). P is read off the stable invariant subspace of the
    2n x 2n Hamiltonian H; the same routine gives the Kalman solution by passing
    (A^T, C^T, Qk, Rk). Matches scipy.solve_continuous_are to ~1e-9 here."""
    n = A.shape[0]
    H = np.block([[A, -B @ np.linalg.inv(R) @ B.T],
                  [-Q, -A.T]])
    w, V = np.linalg.eig(H)
    U = V[:, np.argsort(w.real)[:n]]          # n eigenvectors with Re(lambda) < 0
    P = (U[n:, :] @ np.linalg.inv(U[:n, :])).real
    return 0.5 * (P + P.T)

KEEP = [0, 1, 2, 3, 4, 6, 7]          # vx vy vz wx wy roll pitch  (drop wz, yaw)
MEAS = [2, 3, 4, 5, 6, 7]             # measured rows of the 8-state z


class HoverController:
    def __init__(self, A, B, dt, pitch_trim=0.0, h_ref=0.05,
                 Q=(5, 5, 20, 2, 2, 250, 250, 6e4), R=(50, 50, 50),
                 Qk=(1, 1, 0.05, 30, 30, 0.02, 0.02, 1e-6),
                 Rk=(1e-3, 100, 100, 1e-4, 1e-4, 1e-7), u_limit=0.6):
        nz = len(KEEP)
        Aa = np.zeros((nz + 1, nz + 1)); Aa[:nz, :nz] = A[np.ix_(KEEP, KEEP)]; Aa[nz, 2] = 1.0
        Ba = np.zeros((nz + 1, 3)); Ba[:nz, :] = B[KEEP, :]
        C = np.zeros((len(MEAS), nz + 1))
        for r, idx in enumerate(MEAS):
            C[r, idx] = 1.0
        # LQR: u = -K z
        P = care(Aa, Ba, np.diag(Q), np.diag(R))
        self.K = np.linalg.inv(np.diag(R)) @ Ba.T @ P
        # Kalman: dz_hat = Aa z_hat + Ba u + L (y - C z_hat)
        Pf = care(Aa.T, C.T, np.diag(Qk), np.diag(Rk))
        self.L = Pf @ C.T @ np.linalg.inv(np.diag(Rk))
        self.Aa, self.Ba, self.C = Aa, Ba, C
        self.pitch_trim = pitch_trim; self.h_ref = h_ref; self.u_limit = u_limit
        self.reset()

    def reset(self):
        self.xh = np.zeros(self.Aa.shape[0])

    def update(self, sense, dt):
        """One control step. `sense` is Flyer.sense(). Returns (thrust, roll, pitch)."""
        y = np.array([sense['vz'], sense['wx'], sense['wy'],
                      sense['roll'], sense['pitch'], sense['height'] - self.h_ref])
        u = np.clip(-self.K @ self.xh + np.array([0.0, 0.0, self.pitch_trim]),
                    -self.u_limit, self.u_limit)
        self.xh = self.xh + dt * (self.Aa @ self.xh + self.Ba @ u + self.L @ (y - self.C @ self.xh))
        return u

    @property
    def vy_est(self):
        return self.xh[1]


def design(fly, f_hz=80, feather=45, h_ref=0.05, **kw):
    """Identify the hover model (A, B) for `fly` and return a tuned controller,
    the hover kinematics, and the identified pieces."""
    amp = sysid.hover_amplitude(fly, f_hz=f_hz, feather=feather)
    kin = FlapKinematics(f_hz=f_hz, stroke_amp_deg=amp, feather_amp_deg=feather)
    D, w0 = sysid.stability_derivatives(fly, kin); A = sysid.assemble_A(D, fly.M, fly.I)
    Bw = sysid.control_derivatives(fly, kin); B = sysid.assemble_B(Bw, fly.M, fly.I)
    trim = -w0[4] / Bw[4, 2]
    ctrl = HoverController(A, B, fly.dt, pitch_trim=trim, h_ref=h_ref, **kw)
    return ctrl, kin, dict(amp=amp, A=A, B=B, w0=w0, Bw=Bw)