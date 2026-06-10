# Stages 3–4 — Free Flight, Instability, and Hover Control
 
*A full account: the theory, the mathematics, and — deliberately — every wrong
turn, what broke, how we found the cause, what we tried next and why, and what
finally worked. This is the methods-and-results spine of the paper, written at
lab-notebook resolution so nothing is lost.*
 
---
 
## 0. Where we were, and what this covers
 
By the end of Stage 2 we had a clamped flyer: a bio-inspired body defined in
`models/flyer.xml`, a validated quasi-steady blade-element aerodynamics module
(`src/aero.py`), and prescribed wing kinematics (`src/kinematics.py`). Every
aerodynamic coefficient and force had been checked against a hand-predictable
case. The wings flapped, the aero produced sensible forces, and at the right
stroke amplitude the cycle-averaged vertical force equalled the weight.
 
What we had **not** done is let the body go. Everything was bolted to the world.
Stages 3 and 4 are about free flight:
 
- **Stage 3** — cut the body loose, fly it open-loop on the validated aero, and
 characterise how it (mis)behaves without control.
- **Stage 4** — identify the linearised flight dynamics, then design a
 controller that makes it hover, hold altitude, and recover from disturbances.
The short version of the journey: the first "free flight" tumbled violently and
defeated every controller for many sessions — and it turned out the tumble was a
**simulation artifact**, not physics. Fixing that exposed the *true* (mild)
instability. Identifying that instability surfaced a **sign bug** in our own
linear model. And making the controller actually hold roll required moving from
naive feedback to a **model-based estimator**. Each of these is documented below
in full.
 
A recurring practical fact worth stating up front, because it shapes everything:
**MuJoCo has no aerodynamics.** It integrates rigid-body mechanics and contacts.
Every aerodynamic force in this project is computed by our code and injected. So
"flying" here means: we compute the aero each step and we are responsible for how
it enters the equations of motion. That responsibility is where Stage 3 went
wrong.
 
---
 
## 1. Stage 3 — cutting it loose, and the tumble that wasn't real
 
### 1.1 The setup and the prediction
 
To make a free flyer we take the clamped `flyer.xml` and add a free joint to the
thorax (`make_free_model()` does this through the MuJoCo spec API, so the XML
stays the single source of truth for the body). A free joint gives the body six
degrees of freedom: position `(x, y, z)` and orientation (a unit quaternion).
 
The wings still need to flap. In the first design we did the obvious thing: each
step we *prescribed* the wing joint angles and velocities to follow the commanded
flap, then called `mj_step` to advance the whole tree (body + wings) under the
injected aero plus gravity.
 
The physical prediction was modest and correct in spirit: a hovering flapping
flyer is passively **unstable** — it will not just sit there; it will slowly
diverge in attitude (pitch first, by analogy with insects), and a controller will
be needed to tame it. We expected a slow divergence over tens to hundreds of
milliseconds.
 
### 1.2 The failure
 
What we got instead was a **violent tumble**: pitch reached 10° in about
**10.5 ms**, and the body cartwheeled within a couple of wingbeats. No amount of
control authority helped. We built attitude controller after attitude
controller; each was overwhelmed almost instantly. For several sessions the
working assumption was "the flyer is just extremely unstable, our controllers are
too weak or too slow." That assumption was wrong, and it cost us a lot of time.
 
### 1.3 The smoking gun
 
The breakthrough came from refusing to accept the instability at face value and
instrumenting the very first integration step. We printed the body's angular
acceleration at step 0 and, separately, the aerodynamic torque actually being
applied at step 0.
 
The result was stark: the body's **pitch angular acceleration was about
−25,697 rad/s²** while the **applied aerodynamic torque was essentially zero**.
 
That is impossible if the only torques on the body are aero and gravity. Gravity
applies no torque about the centre of mass; aero was ~0 at that instant. An
angular acceleration of 2.6×10⁴ rad/s² with no corresponding applied torque means
a torque was entering the body from somewhere we were not accounting for.
 
### 1.4 Root cause: prescribing wings on a free body injects inertial torque
 
The culprit was the way we drove the wings. Overwriting the wing joint
velocities every step, on a body that is itself free to move, forces the wings
through a trajectory *regardless of the reaction that motion implies*. The wings
have small but nonzero mass (~1 mg each), and at flapping speed they carry real
angular momentum. When you prescribe that motion kinematically on a free base,
MuJoCo must satisfy the constraint, and the centrifugal/Coriolis reaction of the
flapping wings is transmitted straight into the free base as a spurious torque.
 
A back-of-envelope check makes the scale believable. A wing of mass `m ≈ 1 mg` at
radius `r ≈ 3 mm` flapping at `ω ≈ 2π·80 ≈ 500 rad/s` has a centripetal term of
order `m r ω² ≈ (1e-6)(3e-3)(500²) ≈ 7.5e-4 N` — i.e. hundreds of micronewtons of
oscillating internal force, and the associated oscillating torque about the body
is of order `10³ µN·mm`. The body's pitch inertia is only `I_yy ≈ 6.0×10⁻¹⁰
kg·m²`, so even a fraction of that spurious torque produces angular accelerations
in the **10⁴ rad/s²** range. That is exactly what we measured. The "instability"
was the integrator fighting the kinematic constraint, not aerodynamics.
 
> **Lesson.** Kinematically prescribing the motion of massive sub-bodies on a free
> base is not free. The reaction has to go somewhere, and on a free body it goes
> into the base as a torque you did not intend. If the sub-body inertia matters at
> the relevant frequency, this dominates.
 
### 1.5 The masking fix that wasn't enough
 
Our first response (the original Stage 3 patch) was to stop applying aero to the
wing bodies and instead **lump the entire aero wrench onto the thorax** — sum each
wing's force and torque, carry it to the body centre of mass, and apply that. This
helped the *force* accounting, and it removed one pathology (applying large forces
to near-massless wing bodies gives them absurd accelerations). But it did **not**
cure the disease: we were still calling `mj_step` on the full tree with
kinematically prescribed wings, so the spurious inertial reaction was still being
injected. The tumble was reduced but the dynamics were still contaminated. It
masked the symptom without removing the cause.
 
### 1.6 The real fix: integrate one rigid body, treat wings as massless aero surfaces
 
The clean solution is to change *what we integrate*. The flyer is modelled as a
**single rigid body** with the total mass `M` and the total inertia `I` about the
system centre of mass. The wings become **massless aerodynamic surfaces**: their
motion is still prescribed, but only so that `wing_aero()` can compute the
quasi-steady force. That aero force is then applied to the single rigid body,
which we advance with **our own rigid-body integrator** — not `mj_step` on the
full tree.
 
Concretely, `src/flyer.py` now:
 
1. Computes `M`, `I` (about the CoM), and the constant CoM offset once at
  construction.
2. Tracks the rigid-body state directly: CoM world position `x_com`, world
  velocity `v`, body-frame angular velocity `w`, and orientation quaternion `q`.
3. Each `step()`: writes the current state into MuJoCo's data, prescribes the
  wings, calls `mj_forward` (kinematics only — **no** integration of the tree),
  reads each wing's aero force, sums the wrench about the CoM, then integrates
  the single rigid body and writes the new state back.
Because we never integrate the wing joints dynamically, there is no spurious
inertial reaction. The wings contribute exactly one thing — aerodynamic force —
which is all we ever wanted from them. Their mass still counts (it is in `M` and
`I`); what we drop is the *dynamic* wobble of the flapping mass, a small
high-frequency perturbation. This is the standard "frozen wing inertia"
approximation and it is entirely defensible for body-scale flight dynamics.
 
The rigid-body mathematics we integrate are given in full in §2.
 
### 1.7 Validation, and the honest open-loop result
 
The validation was a hand-predictable case: release the rewritten flyer from a 2°
pitch with symmetric flapping. The old model tumbled in under 20 ms. The new model
**stays within a few degrees over 400 ms** and matches the linear prediction. The
artifact was gone.
 
With an honest model, `e08_free_flight.py` gives the honest answer. From a small
2° pitch + 2° roll seed at the hover stroke amplitude:
 
- pitch crosses 10° at **≈ 78 ms**,
- roll crosses 10° at **≈ 126 ms**,
- the flyer tumbles and falls over **≈ 1 s**.
This is a **mild, slow** instability — the kind a controller can comfortably
handle — not the violent tumble we had been fighting. Pitch crosses first not
because it is the faster mode (it is not, as the eigenvalues later show) but
because of a constant pitch-bias torque that gives it a head start (§4). The
crucial point for Stage 4: the thing we are about to control is gentle.
 
---
 
## 2. The rigid-body dynamics we integrate (math reference)
 
State: CoM position `x ∈ ℝ³` (world), CoM velocity `v ∈ ℝ³` (world), orientation
quaternion `q` (body→world), body-frame angular velocity `ω ∈ ℝ³`. Total mass
`M`, inertia `I` about the CoM (body frame, constant). Rotation matrix `R(q)`.
 
**Translation (Newton, world frame).** With aerodynamic force `F_aero` (sum over
wings) and gravity:
 
$$ \dot v = \frac{1}{M}\left( F_\text{aero} + \begin{bmatrix}0\\0\\-Mg\end{bmatrix}\right), \qquad \dot x = v. $$
 
**Rotation (Euler, body frame).** With aerodynamic torque about the CoM
`T_aero` (world), transported to body frame as `T_b = Rᵀ T_aero`:
 
$$ I\,\dot\omega = T_b - \omega \times (I\,\omega), \qquad \dot\omega = I^{-1}\big(T_b - \omega\times(I\omega)\big). $$
 
The `ω×(Iω)` term is the gyroscopic coupling; it is small here because the body
spins slowly, but we keep it for correctness.
 
**Orientation.** For a body-frame angular velocity, the quaternion derivative is
 
$$ \dot q = \tfrac12\, q \otimes (0,\ \omega), $$
 
integrated and then renormalised each step.
 
**Integration.** We use **semi-implicit (symplectic) Euler**: update velocities
first, then positions/orientation with the *new* velocities. This is markedly
more stable than explicit Euler for oscillatory mechanical systems at our time
step (`dt = 1e-4 s`).
 
**CoM offset.** The system CoM sits a fraction of a millimetre from the thorax
body origin (~0.05 mm). We carry this constant body-frame offset so that when we
write the body into MuJoCo for the wing kinematics, the wings sit in exactly the
right place, and the torque is taken consistently about the true CoM. It is tiny
but it is free to do correctly, and doing it wrong would inject a small constant
bias torque (`F × 0.05 mm ≈ 40 µN·mm`, comparable to the real pitch bias — not
negligible).
 
**Aero wrench about the CoM.** Each wing returns a force `F_i` and a torque `T_i`
about its own centre of pressure; we transport to the system CoM:
 
$$ F_\text{aero}=\sum_i F_i,\qquad T_\text{aero}=\sum_i \big(T_i + (x_{\text{cop},i}-x_\text{com})\times F_i\big). $$
 
This integrator is the foundation everything else stands on; it is the thing that
had to be right before any controller could possibly work.
 
---
 
## 3. Stage 4 — system identification
 
### 3.1 Why linearise, and cycle-averaging
 
The flyer is a strongly time-varying system: the wings beat at 80 Hz, so the
instantaneous forces oscillate enormously within every 12.5 ms cycle. We do not
want to control the wingbeat; we want to control the **slow body dynamics** that
ride on top of it. The standard tool is the **cycle-averaged linear model**: hold
the body in a state, flap for several cycles, average the aero wrench over whole
cycles, and treat that average as a smooth function of the body state and the
control inputs. Differentiating that function about the hover trim gives a linear
state-space model `ẋ = A x + B u`.
 
The hover trim itself is found by bisection: the stroke amplitude whose
cycle-averaged vertical force equals the weight. At 80 Hz this is
**72.63°** (`Fz = 759.3 µN = Mg` exactly).
 
### 3.2 Control authority `B` (experiment e09)
 
`B` answers: what does each control knob do, on cycle-average? Our three knobs are
`u_thrust` (symmetric stroke-amplitude change), `u_roll` (differential amplitude
between wings), and `u_pitch` (mean stroke-angle offset). We perturb each in turn
about hover and read the cycle-averaged wrench.
 
The result is clean and near-diagonal — each knob predominantly drives its
intended axis, with cross-coupling at the few-percent level:
 
| input | dominant effect (at hover) | cross-coupling |
|---|---|---|
| `u_thrust` | `Fz` ≈ 1520 µN / unit | ≤ 8 % |
| `u_roll`   | `Tx` ≈ 10399 µN·mm / unit | ~1 % |
| `u_pitch`  | `Ty` ≈ 5493 µN·mm / unit | ~1 % |
 
(Reported here at the 72.63° hover amplitude; e09 also reports the same structure
measured at a 60° reference, 1041 / 7958 / 4081, confirming the *shape* is
amplitude-robust.) The near-diagonality is what makes the system controllable
with three knobs and is why even simple per-axis control is conceptually sound.
 
### 3.3 Open-loop dynamics `A` (experiment e10)
 
`A` is measured the same way, but perturbing **body twist** instead of controls.
We impose a small `vx, vy, vz, ωx, ωy, ωz` on the (otherwise level, hovering)
body, flap, average the wrench, and finite-difference. That gives a 6×6 matrix `D`
of aerodynamic **stability derivatives** (how the cycle-averaged wrench responds
to body motion). Physical sanity checks all pass:
 
- heave damping `∂Fz/∂vz < 0` (sinking makes more lift — restoring),
- pitch-rate damping `∂Ty/∂ωy < 0`, roll-rate damping `∂Tx/∂ωx < 0`,
- longitudinal/lateral cross terms ≈ 0 (the two planes decouple, as expected for
 a symmetric flyer).
From `D`, the rigid-body `M`, `I`, the thrust-tilt (gravity) coupling, and the
attitude kinematics, we assemble the 9-state matrix `A` for
 
$$ x = [\,v_x\ v_y\ v_z\ \ \omega_x\ \omega_y\ \omega_z\ \ \phi_\text{roll}\ \theta_\text{pitch}\ \psi_\text{yaw}\,]. $$
 
### 3.4 The attitude-kinematics sign — and the bug we shipped
 
The attitude block of `A` couples body rates to the tilt angles. We define pitch
and roll as the tilt of the body up-axis toward world +x and +y. For small tilts,
roll ≈ `up_y`, pitch ≈ `up_x`. The up-axis evolves as
 
$$ \dot{\mathbf{up}} = \omega \times \mathbf{up}, \qquad \mathbf{up}\approx(0,0,1) \;\Rightarrow\; \dot{\mathbf{up}} = (\,\omega_y,\ -\omega_x,\ 0\,). $$
 
So the correct kinematics are
 
$$ \dot\theta_\text{pitch} = +\omega_y, \qquad \boxed{\dot\phi_\text{roll} = -\omega_x}, \qquad \dot\psi_\text{yaw} = +\omega_z. $$
 
The cross product **flips the sign on roll**. We had originally written
`roll_dot = +ωx`. Pitch and yaw were right; roll was inverted.
 
This is a textbook trap, and it bit us twice. First, it made the **lateral
eigenvalues wrong** in the version of e10 we initially shipped (the lateral mode
came out as a benign oscillation when it is really a fast divergence — see §3.5).
Second, and more painfully, it made the **roll feedback come out backwards** in
the first LQR (§5.5): the controller "corrected" a roll error by driving it
further. We caught it by probing the *clean* flyer directly — apply `+u_roll`,
observe the roll sign; apply a `+ωx` kick, observe the roll sign — and both
contradicted the linear model. The one-line fix (`A[6,3] = -1`) reconciled the
model with the plant.
 
> **Lesson.** Derive attitude kinematics from `d(up)/dt = ω×up` and *check the
> signs against the real plant* with a direct probe before trusting any model
> built on top of them. A single inverted sign hid for an entire stage because
> the pitch axis, which worked, masked it.
 
### 3.5 Eigenvalues and the modes
 
With the corrected `A`, the open-loop eigenvalues (1/s) are:
 
| eigenvalue | character | meaning |
|---|---|---|
| **+15.98** | unstable, real, doubling **43 ms** | fast lateral/roll divergence |
| **+4.33 ± 11.99 j** | unstable, oscillatory, 1.9 Hz, doubling 160 ms | longitudinal/pitch mode |
| 0.00 | neutral | yaw drift (no restoring, no actuator) |
| −1.67 | stable, τ 598 ms | slow heave-coupled mode |
| −17.45, −22.84 | stable | fast aerodynamically-damped modes |
| −27.96 ± 12.56 j | stable, 2.0 Hz | damped lateral oscillation |
 
The physical picture is the textbook hovering-insect one: an unstable, divergent
roll, an unstable oscillatory pitch, a stable heave, and a freely-drifting yaw.
Note this **squares with e08**: the lateral mode is actually the *faster* of the
two unstable modes, yet pitch crosses 10° first in free flight — because there is
a constant **+41 µN·mm pitch-bias torque** at trim that gives pitch a head start
from rest, while roll has no bias and must grow purely from its (faster) mode.
Both are slow compared to a wingbeat, so a cycle-averaged controller is valid.
 
### 3.6 Controllability — why yaw is dropped
 
The controllability matrix of `(A, B)` has rank **7 of 9**: the system is *not*
fully controllable. The reason is concrete — none of our three knobs produces a
yaw torque `Tz` (there is no yaw actuator; that is the future halteres / stroke-
timing work). Yaw angle and yaw rate are therefore uncontrollable. Crucially they
are also **non-divergent** (yaw is neutral, its rate is self-damped), so leaving
them out of the design is safe: we drop yaw entirely, design on the controllable
7-state subspace, augment it with height for altitude hold, and let yaw drift.
This is why the flyer in the viewer holds attitude beautifully but slowly rotates
and translates — those are precisely the unactuated freedoms.
 
---
 
## 4. Stage 4 — control: the long road to hover
 
### 4.1 First idea: independent PID, and why it could not work yet
 
The near-diagonal `B` suggests three independent PID loops (thrust↔altitude,
roll↔roll, pitch↔pitch). We built that early. It failed — but at the time we could
not cleanly separate "the controller is bad" from "the plant is a fake tumble,"
because this predated the Stage 3 fix. Once the integrator was corrected, we chose
not to return to hand-tuned PID but to design from the identified `(A, B)` with
**LQR**, which is principled, handles the coupling, and gives a single coherent
gain. PID was abandoned, not committed.
 
### 4.2 LQR — the theory
 
LQR chooses the state-feedback `u = -Kx` minimising
 
$$ J = \int_0^\infty \big(x^\top Q\, x + u^\top R\, u\big)\,dt, $$
 
with `Q ⪰ 0` penalising state error and `R ≻ 0` penalising control effort. The
optimal gain is `K = R⁻¹ Bᵀ P`, where `P` solves the **continuous-time algebraic
Riccati equation (CARE)**
 
$$ A^\top P + P A - P B R^{-1} B^\top P + Q = 0. $$
 
We design on the reduced state `z = [vx, vy, vz, ωx, ωy, roll, pitch, h]` (yaw
dropped, height `h` appended with `ḣ = vz` for altitude hold). Larger `Q` entries
mean "hold this tighter"; larger `R` means "use less control / lower bandwidth."
 
### 4.3 First LQR test: instant divergence — the wingbeat ripple
 
Linearly, the closed loop was comfortably stable (slowest pole ≈ −1 /s). On the
*nonlinear* flyer it diverged in **~64 ms**. The discrepancy was the tell, so we
instrumented the first few milliseconds and found `u_pitch` slamming to its limit
within **0.8 ms** — driven not by pitch (which was ~0.006°) but by a pitch *rate*
of 0.6 rad/s. We then measured the open-loop rate ripple and found the body
genuinely wobbles at the wingbeat frequency with a **pitch-rate amplitude of about
±17 rad/s** (and, once rolling, a roll-rate ripple up to ±60 rad/s).
 
That is the root cause: LQR is a **cycle-averaged** design, but we were feeding it
the **instantaneous** rippling rates. The rate gains amplified the 80 Hz wobble
straight into actuator saturation, and a saturated controller chattering against a
fast unstable axis diverges. The body wobble is real physics (oscillating aero
torque on a tiny inertia); the controller simply must not chase it.
 
### 4.4 Filtering — what worked and what didn't
 
We needed to give the controller the slow (cycle-averaged) state, so we low-pass
filtered the sensed signals. This turned into a careful study:
 
- **Exponential moving average (EMA), uniform time constant.** Worked. With
 `R ≈ 50` and `τ ≈ 9 ms` (cutoff ~18 Hz, between the ~2 Hz dynamics and the 80 Hz
 ripple) the flyer **held hover from level and recovered pitch kicks up to 20°**,
 with a small residual ~3–4° pitch wobble from ripple leaking through.
- **One-period moving average (boxcar).** *Failed.* A boxcar over one wingbeat
 perfectly nulls the 80 Hz fundamental — but it adds ~6 ms of group delay (half
 the window), and that lag destabilised the loop. Cleaner ripple rejection, fatal
 phase lag.
- **Filtering rates harder than angles.** *Failed.* The rate feedback is the
 *damping*; lagging it more removed the damping and the fast axes went unstable.
The takeaway is a genuine engineering tension: ripple rejection wants strong
filtering (lag), stability of a fast unstable axis wants minimal lag. The EMA at
τ ≈ 9 ms is the compromise that survives — for pitch.
 
### 4.5 Pitch solved, roll not — and the second sign bug surfaces
 
With the EMA, **pitch became robust** (holds level, recovers 20° kicks). **Roll
did not.** From level, roll is fine (a symmetric flyer has nothing to excite it).
But any roll *kick* made roll oscillate with growing amplitude and tumble in
50–200 ms, no matter how we tuned the roll gain or filter.
 
The stubborn, tuning-independent ~60 ms failure was a structural tell, so we
watched a roll kick directly — and found the controller commanding `u_roll` in the
**wrong direction**: roll at −10°, controller pushing it to −12°. That is what led
us to the `roll_dot = −ωx` sign bug of §3.4. We fixed `A`, redesigned, and roll
*began* to recover correctly (−15° → −9° in 8 ms).
 
### 4.6 Roll still fails after the sign fix — the real diagnosis
 
The sign fix was necessary but not sufficient. With it, roll recovered *initially*
and then built into a **growing oscillation** that tumbled around 50–60 ms.
Watching the full trajectory showed three compounding problems, all specific to
the roll axis:
 
1. **Roll is the fast unstable mode** (+15.98 /s, doubling 43 ms) — it must be
  caught within tens of ms, leaving little tolerance for filter lag.
2. **Roll has ~10× less inertia and ~2× the authority** of pitch — it is twitchy,
  so the same relative ripple that pitch tolerates drives roll bang-bang.
3. **The sideslip velocity `vy` — the state the fast lateral mode rides on — is
  not measured** (we have no airspeed sensor) and so was not fed back. During a
  roll kick, `vy` drifted to ~150 mm/s, feeding the very mode we were trying to
  kill.
Gentle-roll gain sweeps and assorted filter combinations did not fix it. The
problem was not a gain; it was that the controller was both *blind to `vy`* and
*fighting ripple in `ωx`*. That diagnosis pointed directly at the solution.
 
### 4.7 The fix: LQG — a model-based estimator
 
The principled answer to "unmeasured states + noisy measurements" is a **Kalman
filter**, giving an **LQG** controller (LQR + Kalman estimator). The filter runs
the cycle-averaged model forward and corrects it with the measurements:
 
$$ \dot{\hat z} = A\,\hat z + B\,u + L\,(y - C\,\hat z), $$
 
where `y = [vz, ωx, ωy, roll, pitch, h]` are the measured outputs (`vx, vy`
hidden), `C` selects them, and `L` is the Kalman gain. `L` comes from the *dual*
Riccati equation — solve CARE with `(Aᵀ, Cᵀ, Q_k, R_k)` for `P_f`, then
`L = P_f Cᵀ R_k⁻¹`. The two covariances do the engineering:
 
- **`R_k` large on the rate channels `ωx, ωy`** tells the filter "the rate
 measurements are noisy" — so it trusts the smooth model prediction over the
 rippling rates. This **rejects the wingbeat ripple** as a *model-based* low-pass,
 with far better phase behaviour than a blind EMA.
- The filter **estimates `vx, vy`** from the dynamics, so the LQR can finally feed
 back the *full* state, including the sideslip that the fast roll mode needs.
The controller then uses `u = -K \hat z` on the clean full-state estimate. This
solves both failures at once: it kills the ripple *and* closes the loop on `vy`.
 
### 4.8 Success — hover hold and disturbance rejection (e11)
 
It worked, and not marginally. With the Kalman filter feeding clean estimates
(tracked `vy` to within ~15 mm/s of truth), the LQR commands smooth, *small*
control — the bang-bang was never about needing authority, it was about chasing
ripple. Results:
 
- **Holds hover** from level: |pitch| < 3°, roll flat, altitude locked to ~51 mm,
 control effort ~1 %.
- **Recovers large kicks**: a combined **30° pitch + 30° roll** kick returns to
 |roll| < 0.4°, |pitch| < 2.6°, altitude held, peak control **0.11 of 1.0**.
- **Rejects gusts**: mid-flight angular-rate puffs in roll and pitch are absorbed
 and recovered.
`e11_hover_control.py` is the money shot: the *same* flyer, *same* aero, *same*
10°/10° kick, run open-loop (tumbles past 45° at 124 ms and falls 3.7 m) versus
closed-loop (holds, and shrugs off two mid-flight gusts on **4 %** of control
authority). The only difference between tumble and hover is the loop.
 
### 4.9 Dropping scipy — a Riccati solver from scratch
 
The CARE solves initially used `scipy.linalg.solve_continuous_are`, but the
target environment did not have scipy. Rather than add a dependency, we wrote the
solver in ~10 lines of NumPy using the classical **Hamiltonian-eigenvector
method**. For CARE `AᵀP + PA − PBR⁻¹BᵀP + Q = 0`, form the `2n×2n` Hamiltonian
 
$$ H = \begin{bmatrix} A & -B R^{-1} B^\top \\ -Q & -A^\top \end{bmatrix}, $$
 
take its eigendecomposition, collect the `n` eigenvectors whose eigenvalues have
negative real part (the stable invariant subspace), partition that basis as
`[U₁; U₂]`, and read off `P = U₂ U₁⁻¹` (symmetrised). The same routine serves the
Kalman solve by duality. It reproduces scipy's gains to ~1e-9 on our matrices, so
the controller is now fully self-contained — one more black box turned into
readable code.
 
---
 
## 5. The control law, end to end (reference)
 
Per step, given the sensor reading `s = (pitch, roll, height, ωx, ωy, ωz, vz)`:
 
1. Form the measurement vector `y = [vz, ωx, ωy, roll, pitch, height − h_ref]`.
2. Compute control from the current estimate:
  `u = clip(−K ẑ + [0, 0, u_pitch_trim], ±0.6)`,
  where `u_pitch_trim = −(bias Ty)/(B: pitch→Ty)` cancels the constant +41 µN·mm
  pitch bias so there is no steady-state pitch offset.
3. Apply `u` as `(u_thrust, u_roll, u_pitch)` to the kinematics and step the flyer.
4. Advance the estimator: `ẑ ← ẑ + dt·(A ẑ + B u + L (y − C ẑ))`.
Design choices baked in: yaw dropped (uncontrollable, non-divergent); height
appended for altitude hold; feedback uses only physically measurable signals while
`vx, vy` are *estimated*; `Q`, `R` tuned for ~1 % hover effort with headroom; the
Kalman `R_k` large on rate channels to reject wingbeat ripple. Implemented in
`src/controller.py` as `HoverController` (+ a `design()` helper that runs the
system-ID and returns a tuned controller).
 
---
 
## 6. What the failures taught us (the meta-lessons)
 
- **A plausible-looking instability can be a numerical artifact.** We lost
 sessions treating a constraint-violation torque as physics. The cure was to
 *measure the applied torque and the resulting acceleration separately* and
 notice they were inconsistent. When a system misbehaves far more violently than
 first principles predict, suspect the integrator before the physics.
- **Prescribing massive sub-bodies on a free base injects reaction torques.**
 Integrate what you mean to integrate; if the wings are only there for aero, don't
 let the solver treat their prescribed motion as a dynamic constraint on the body.
- **Derive attitude kinematics and verify the signs against the real plant.** One
 inverted sign (`roll_dot = +ωx` instead of `−ωx`) corrupted the lateral
 eigenvalues and inverted the roll feedback, and it hid behind the working pitch
 axis.
- **A cycle-averaged controller must be fed cycle-averaged state.** Feeding raw,
 rippling rates to a cycle-averaged LQR saturates it instantly. The wobble is real
 physics; the controller must not chase it.
- **Filtering trades ripple rejection against phase lag.** Strong filters (boxcar,
 hard rate filtering) reject ripple but the lag destabilises fast unstable axes. A
 *model-based* estimator beats a blind filter precisely because it gets the phase
 right.
- **When a stubborn failure is independent of tuning, it is structural.** Roll
 failing at ~60 ms regardless of gain pointed first to a sign error, then to an
 unmeasured state — not to a gain that needed more sweeping.
- **The right state estimate makes the control easy.** The final controller uses
 ~1–11 % of its authority. The hard part was never force; it was *information* —
 knowing the true, clean, full state.
---
 
## 7. Validation ledger and file map
 
| stage / item | check | result |
|---|---|---|
| rigid-body rewrite | release from 2° pitch | stays within few° / 400 ms (was tumble < 20 ms) |
| e08 open-loop | time to 10° | pitch 78 ms, roll 126 ms — mild |
| e09 control authority `B` | each knob → its axis | dominant, cross-coupling ≤ 8 % |
| e10 dynamics `A` | sign sanity + eigenvalues | damping signs correct; modes as expected |
| roll sign | clean-plant probe | `+u_roll → −roll`, `+ωx → −roll` ✓ |
| CARE solver | vs scipy on real matrices | match ~1e-9 |
| LQG hover | hold + 30° kick + gusts | holds; recovers; ~4–11 % control |
| e11 money shot | open vs closed, same kick | tumble+fall vs hover+recover |
 
Code: `src/flyer.py` (rigid-body integrator + aero injection), `src/sysid.py`
(cycle-averaged `A`, `B` identification), `src/controller.py` (LQG: CARE, LQR,
Kalman, trim). Experiments: `e08` (open-loop), `e09` (`B`), `e10` (`A` +
eigenvalues), `e11` (closed-loop money shot), `view_hover.py` (live viewer). All
experiments save CSV + PNG to `outputs/`.
 
---
 
## 8. Open issues and future work
 
- **Yaw is open-loop.** No yaw actuator yet, so yaw drifts (visible as slow
 rotation). Adding yaw authority (stroke-plane deviation / split-cycle timing,
 i.e. the halteres-inspired mechanism) and closing the loop is the next control
 item.
- **Horizontal position is open-loop.** We stabilise attitude and altitude but not
 `(x, y)`; with no position sensor the flyer translates slowly (the gentle forward
 drift seen in the viewer). A position/outer loop needs an external reference and
 belongs to the commanded-motion stage.
- **The estimator uses the cycle-averaged model.** It treats the wingbeat wobble
 as measurement noise rather than modelling it. This is standard and works well,
 but it is an approximation; a wingbeat-resolved (e.g. harmonic-balance) estimator
 is a possible refinement.
- **Frozen wing inertia.** Body dynamics use a fixed mass/inertia; the small
 periodic shift of the CoM as the wings beat is neglected. Justified at body
 scale, but worth quantifying for the paper.
Next stage (Stage 5) builds directly on this loop: feed it **reference offsets**
(climb/descend via `h_ref`, translate via small commanded attitude) to get
commanded up/down/left/right motion — the flyer doing what it's told, on top of a
hover it can already hold.