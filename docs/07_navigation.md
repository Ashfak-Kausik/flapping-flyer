# 07 — Navigation: cruise, centering, and turning (Arc B)
 
Arc A established that the flyer can *sense* its surroundings — a calibrated 3D proximity
vector read from its own aerodynamic control residuals (docs/06). That is a reactive,
station-keeping capability: hover near a surface and feel where it is. **Arc B is about
going somewhere** — moving through a confined space under control, not just reacting in
place. Three capabilities are needed: hold a forward speed (cruise), stay off the walls
while moving (a lateral position loop), and change heading (turn). This note records
what is earned, and one honest open problem.
 
Summary figure: `outputs/arcB_summary.png`. Experiments: `e22_forward_cruise.py`
(cruise + tube fly-through), `e23_yaw_authority.py` (yaw actuator).
 
---
 
## B1 — Forward cruise via optic flow
 
**The problem.** The hover controller cannot cruise. Commanded to fly forward at
0.05 m/s, the flyer instead settled at ~0.24 m/s and, near a wall, ran away entirely.
The cause is not gains: forward velocity `vx` is a *hidden* state that the hover Kalman
filter infers from attitude, valid only near `vx ≈ 0`. Once the flyer actually
translates, the hover-linearised estimator stays frozen near zero (measured: estimate
−0.003 m/s while the true speed was 0.239). The speed loop was regulating a fiction.
 
**The physics is benign.** A cruise-trim sweep (cycle-averaged forces vs forward speed
and pitch) shows pitch has a strong, clean grip on horizontal force (~13 µN/deg) while
forward motion adds only a gentle drag (~25 µN at 0.2 m/s, ~3% of weight). Steady cruise
needs just a 1–2° pitch trim. Flight is easy; the missing piece was *sensing the speed*.
 
**The fix — optic flow + a speed-via-pitch cascade.** Real flying insects do not estimate
velocity from proprioception; they read the visual flow field. We add an optic-flow
analog: `sense()` returns the world horizontal velocity (`vx`, `vy`). An **outer loop**
reads `vx` directly and commands a pitch reference (PI on speed error); the **inner LQG**
holds that pitch. Crucially, `vx` is removed from the LQR (`K[:,0]=0`) so the broken
hover-frame `vx` estimate never enters control — the outer loop owns forward speed
entirely.
 
**Result (figure A).** Forward speed now tracks a command (0.05→0.064, 0.10→0.114,
0.15→0.164; a steady ~15% offset remains to trim out), with no lateral drift and altitude
held. Hover never needed a speed sense because `vx ≈ 0`; cruise demands optic flow. The
flyer's sensing suite grows with its behaviour — a concrete, bio-plausible reason.
 
---
 
## B2 — Lateral position loop (the recurring control gap, resolved)
 
Centring between walls had appeared throughout the project (corridor, shaft, tube) and was
always **underdamped**: velocity-nulling alone (`vy_ref = -Kp·δ`, δ the proximity
residual) centred the flyer, then let it slowly drift into the wall — about 5 s before
contact while cruising. The signal δ is a position-like quantity, so commanding velocity
from it is proportional position feedback with *no damping term*.
 
**The fix.** A proper PD: proportional on the proximity δ **plus rate damping from the
optic-flow lateral velocity** — `vy_ref = -Kp·δ - Kd·vy`, with Kp=1e-4, Kd=3.0. The
optic-flow velocity added for cruise (B1) is exactly the missing damping signal. Strong
enough proportional *and* derivative action locks the loop.
 
**Result (figure B).** Cruising down a round tube (R=32 mm), the flyer centres from
6 mm off-axis and then **holds the centreline to ~0.2–0.3 mm**, traversing >1 m with no
wall contact (16 s tested; clearance never below +12.7 mm). The same optic-flow velocity
closed two gaps at once — forward-speed control and the long-standing centring damping.
The fly-through video is `outputs/tube_flythrough.mp4`.
 
---
 
## B3 — A yaw actuator from split-cycle flapping
 
The controllability analysis (e10) found yaw uncontrollable: symmetric flapping with
amplitude/offset controls makes no net torque about the vertical axis — fatal for a tunnel
that bends. We add the standard flapping-MAV mechanism, **split-cycle**: distort the
stroke phase so a wing sweeps faster in one half-stroke than the other, so its
cycle-averaged fore/aft drag no longer cancels; applied oppositely on the two wings, the
fore/aft forces at the wings' lateral offsets form a yaw couple.
 
   stroke = A·cos(ξ),   ξ = ω·t + (K·u_yaw·mirror)·sin(ω·t)     (mirror = +1 R, −1 L)
 
**Result (figure C).** A clean, linear, sign-controllable yaw torque — authority
~49 rad/s² per unit `u_yaw` (enough in principle for a ~90° turn in ~0.5 s). Yaw is
intrinsically far weaker than roll because it is a second-order drag asymmetry rather than
a lift differential — as real insects yaw slowly. Cross-coupling is real but trivially
absorbed: a roll torque ~5× the yaw (cancelled by u_roll ≈ 0.017·u_yaw, against a roll
authority of ~60,900 rad/s²/unit) and a ~6% lift bump (trimmed by the altitude loop).
 
The capability the controllability analysis said we lacked, we can now generate.
 
---
 
## B4 — Closing the yaw heading loop (was the open problem; now resolved for moderate turns)
 
The yaw *actuator* (B3) was measured with the flyer **held fixed**; in early free-flight
tests the yaw rate oscillated and feeding heading back tumbled the flyer, so heading
control was first logged as unsolved. A yaw-dynamics diagnosis (`e24_yaw_heading.py`)
resolved it.
 
**Diagnosis — the key insight.** Imposing a steady body yaw rate on a level flapping flyer
and measuring the cycle-averaged torque gives dTz/dwz ≈ −3.85 µN·mm per rad/s: yaw is
**naturally well-damped** (time constant ~0.19 s), a stable first-order rate response. A
constant `u_yaw` settles to a steady turn rate; it does **not** run away. So the
oscillation was never the yaw physics — it was the controller feeding back a *noisy `wz`
rate term* for damping that the aerodynamics already provide.
 
**Controller — the fix.** Drop the noisy `wz` feedback and control a **low-pass filtered
heading** (removing the wingbeat wobble), with: proportional + derivative-of-filtered-
heading (a clean rate, with natural aero damping doing most of the work); **integral**
action to reject the small constant yaw cross-coupling the LQG's own roll/pitch commands
inject (the source of a steady-state error under proportional-only control); a
**slew-limited** heading reference so `u_yaw` and its roll coupling stay small; and the
e23 roll-coupling feed-forward (u_roll −= 0.0168·u_yaw).
 
**Result.** Moderate commanded turns track cleanly and stably: +30° → 30.4° (0.4° error),
max|roll| 0.7°, altitude held. The flyer can now turn to a commanded heading.
 
**Honest remaining edge.** Large sharp turns (90°) reach the neighbourhood but settle short
and disturb altitude — the bigger maneuver drives more coupling (roll excursion, the ~6%
lift bump). They need gentler slewing and altitude coordination during the turn. For tunnel
bends, which are typically gentle, the heading loop is sufficient as-is; sharp 90° turns are
a logged refinement.
 
---
 
## What Arc B earns
 
A flapping micro-flyer that, on a CPU-only simulation with classical control and no
proximity sensor:
 
- holds a commanded **forward cruise** speed using an optic-flow analog (the cue real
 insects use), having diagnosed *why* a hover controller is blind to forward speed;
- stays centred in a confined channel **while moving**, to sub-millimetre accuracy, via a
 PD position loop whose damping comes from the same optic-flow signal — retiring the
 underdamped centring seen since the corridor;
- can **generate a yaw torque** via split-cycle, removing the uncontrollability that would
 otherwise forbid turning.
- can **turn to a commanded heading** via split-cycle yaw plus a filtered-heading PI+D
 loop, after a diagnosis showed yaw is naturally damped and the earlier instability was a
 noisy-feedback artifact, not the physics.
It flies a *straight* round tube end-to-end, and it can execute moderate commanded turns.
The next step is to combine them — a *bending* tube flown by cruise + lateral centering +
commanded yaw at the bends — which is the demonstration that ties Arc B together and the
doorway to Arc C (a meshed devastated 3D scene with a start→goal path). Sharp 90° turns
with full altitude coordination remain a logged refinement.