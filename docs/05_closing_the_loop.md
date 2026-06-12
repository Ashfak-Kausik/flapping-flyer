# Stage 6 — Closing the Loop: Flow-Sensed Wall Avoidance
 
*The payoff stage. The feasibility section (docs/04) showed a real, directional,
usefully-ranged proximity signal lives in the flyer's own aerodynamics. Here we
close the loop: the flyer flies near a wall, reads the signal from its own control
commands — no dedicated proximity sensor — and reacts, holding a chosen standoff.
This is the lab notebook for that: the architecture, the two new code paths, the
two experiments (sensing validated in-loop; standoff controlled), and a frank
account of three limitations the simple loop exposes.*
 
---
 
## 0. The claim we are trying to earn
 
> A flapping flyer can hold a controlled standoff from a wall it senses **only**
> through self-generated unsteady aerodynamics, read off the controller's own
> command residuals, with classical control on a CPU — no extra sensor, no
> learning.
 
Everything below is in service of earning exactly that sentence, and of being
precise about its edges.
 
---
 
## 1. Architecture — three layers on the validated inner loop
 
The Stage 4 LQG hover controller is untouched as the inner loop. Two thin layers
wrap it:
 
1. **Surface-aware flight.** The flyer must actually *feel* the wall, or there is
  nothing to sense. `Flyer.step` gained an optional `surface=` argument; when set,
  the per-wing aero runs through the ground/wall-effect model (docs/04). With
  `surface=None` it is bit-identical to Stages 2-5, so nothing prior is disturbed.
2. **Sense.** The wall's roll disturbance is cancelled by the LQG; the cancelling
  effort is a steady bias in the controller's own `u_roll`. Low-passing `u_roll`
  over ~a wingbeat gives the proximity signal — magnitude = how close, sign =
  which side.
3. **React.** An outer loop (`src/avoidance.py`) maps that residual to a commanded
  lateral velocity fed into the LQG, regulating the standoff.
For (3) the controller needed a way to be *told* to translate. `controller.update`
gained a `vy_ref` argument: the reduced-state reference's lateral-velocity entry is
set, so `u = -K(ẑ - z_ref)` drives the flyer to a commanded sideways speed (it
leans via roll to achieve it). `vy_ref=0` recovers pure hover. This is also the
first commanded-translation capability in the project (a slice of Stage 5).
 
---
 
## 2. Sensing in the loop — experiment `e14`
 
**Question.** The open-loop probe (e13) predicted a wall roll torque. Does that
actually show up as a *control residual* when the flyer is flying and the LQG is
fighting it?
 
**Method.** Fly closed-loop with a wall at a range of standoffs; at each, read the
cycle-averaged steady `u_roll`. Compare to the open-loop prediction `−ΔTx/B_roll`
(the roll command needed to exactly cancel the measured wall torque).
 
**A wrinkle worth recording — passive repulsion.** On the first attempt the flyer
*drifted 40 mm off the wall* during the measurement. The wall's enhanced near-wing
lift rolls the flyer away, and the LQG (which regulates lateral *velocity*, not
*position*) lets it coast off. So the flyer has a built-in tendency to flee walls
— interesting and bio-plausible, but its **sign is the model-dependent part** (a
real wall's omitted lateral suction could weaken or reverse it), so we do not lean
on it. To characterise the *sensor* cleanly we held lateral position (as an outer
position loop would), isolating the residual at each fixed standoff.
 
**Result — the sensor works in-loop.** Held on station, the closed-loop residual
tracks the open-loop prediction across the range:
 
| wall gap | `<u_roll>` measured | predicted −ΔTx/B | final roll |
|---:|---:|---:|---:|
| 2.7 mm  | −0.0638 | −0.0833 | −5.9° |
| 4.7 mm  | −0.0370 | −0.0332 | −1.6° |
| 6.7 mm  | −0.0231 | −0.0216 | −0.7° |
| 10.7 mm | −0.0112 | −0.0104 | −0.3° |
| 16.7 mm | −0.0050 | −0.0047 | −0.1° |
| 26.7 mm | −0.0019 | −0.0018 | −0.0° |
 
The match is tight except at the closest gap, where the LQG shares the
cancellation between `u_roll` and a small steady roll angle (−5.9°), so `u_roll`
alone slightly under-reads — a benign, explainable deviation. **Directionality**
holds: wall at +y gives `u_roll = −0.023`, at −y gives `+0.023`. The wall is read
straight from the controller's own command, no extra sensor. (Files:
`experiments/e14_wall_sensing.py`, `outputs/e14_wall_sensing.{csv,png}`.)
 
---
 
## 3. The avoidance loop — `src/avoidance.py`
 
`WallStandoff` is deliberately tiny: it low-passes `u_roll` into a proximity
estimate `r` (EMA over ~a wingbeat) and commands
 
```
vy_ref = clip( sign(r) · K · (|r| − r*),  ±v_max )
```
 
If the flyer is closer than the setpoint (`|r| > r*`) it retreats; farther, it
approaches; the residual's sign sets the direction automatically. The setpoint
`r*` *is* the standoff command. No geometry, no map — just regulate the sensed
signal. That the "approach when too far" branch must overcome the passive
repulsion is what makes the loop demonstrably *active*, not passive coasting.
 
---
 
## 4. The demonstration — experiment `e15`
 
**(A) Convergence to a commanded standoff, from either side.** For two setpoints,
starting both near (y=0) and far (y=−30 mm), the flyer flies to the corresponding
standoff and holds it, never contacting the wall. The traces share the same
settled gap regardless of start — active regulation, not a transient.
 
**(B) Dial-a-standoff.** Sweeping the setpoint gives a clean monotone knob, with
the near/far spread as the error bar:
 
| residual setpoint | achieved gap (mean) | near / far spread |
|---:|---:|---:|
| 0.006 | 28.7 mm | 3.8 mm |
| 0.009 | 23.6 mm | 2.9 mm |
| 0.012 | 20.5 mm | 2.6 mm |
| 0.016 | 17.5 mm | 2.0 mm |
 
Larger setpoint → tighter standoff, reached from both sides, spread tightening as
the standoff tightens. The standoff is a usable control parameter, driven purely by
the `u_roll` residual. (Files: `experiments/e15_wall_avoidance.py`,
`outputs/e15_wall_avoidance.{csv,png}`.) **This is the headline result: autonomous,
sensor-free, flow-based wall standoff on a flapping MAV.**
 
---
 
## 5. Three honest limitations the simple loop exposes
 
These are real and we state them rather than tune them away silently:
 
1. **Maneuver contamination.** While the flyer is translating, `u_roll` contains
  the roll used to *make* that translation, not just the wall's disturbance. So
  the achieved residual is not equal to the setpoint, and the setpoint→standoff
  map is monotone and repeatable but **not 1:1**. The *sensing* is validated
  quasi-statically (e14, position held); the *acting* layer rides on a partly
  self-contaminated signal. The principled fix is a **disturbance observer** that
  estimates the wall torque as an exogenous input and subtracts the known
  commanded-roll contribution — deferred as a refinement.
2. **Limit cycle under a hard shove.** Given a strong velocity kick straight at the
  wall, the simple proportional loop overshoots in and out (a ~5–25 mm limit
  cycle) instead of settling, again because the retreat maneuver corrupts the
  reading. Gentler gains + a deadband shrink the ripple from ~22 mm to ~5 mm but
  do not eliminate it; the disturbance observer above is the real cure. Gentle
  convergence (§4) is clean; violent disturbance rejection is not yet.
3. **Passive repulsion, sign uncertain.** The model exhibits a passive aerodynamic
  wall repulsion strong enough to resist a commanded approach. It is plausibly
  real and bio-relevant, but its **sign** depends on the omitted lateral-suction
  term, so we present the *active* loop (robust to that sign) as the result and
  flag the passive effect as a model-dependent observation.
---
 
## 6. What Stage 6 is / is not entitled to claim
 
**Earned.** With the established surface-effect physics of docs/04: a flapping
flyer can sense a wall in closed loop from its own control residual (e14, matches
the open-loop prediction, correct directionality) and **actively hold a selectable
standoff** from it with no proximity sensor and classical CPU control (e15, monotone
dial-a-standoff, convergent from both sides, no contact).
 
**Not earned.** Precise residual→distance calibration (contaminated during motion),
clean rejection of violent disturbances (limit cycle), and the sign/magnitude of
the passive repulsion — all pending the disturbance observer and, ultimately,
experimental calibration of `K_GE`. As throughout, this is a *simulation*
demonstration on top of an *added* (not derived) surface-effect model.
 
---
 
## 7. Next
 
- **Disturbance observer** to decontaminate the residual — fixes limitations (1)
 and (2) together and turns the monotone map into a calibrated one.
- **Floor sensing via the thrust residual** — the vertical analogue of e14
 (`u_thrust` drops near a floor), then floor-standoff / safe-descent.
- **Corridor centering** — two walls, balance the left/right residuals.
- **A MuJoCo viewer clip** of the standoff hold for the paper's supplementary.
- Eventually: the disturbance-observer'd loop is the version to carry toward a
 physical build, where `K_GE` and the passive-force sign get pinned in a tunnel.
Files for this stage: `src/flyer.py` (surface-aware), `src/controller.py` (vy_ref),
`src/avoidance.py`, `experiments/e14_wall_sensing.py`, `experiments/e15_wall_avoidance.py`,
and the `outputs/e14_*`, `outputs/e15_*` artifacts.
 
---
 
## 8. Refinement — the disturbance observer: from proxy to calibrated distance sensor (`e16`)
 
Section 5 listed maneuver contamination as the core weakness: reading the wall off
`u_roll` mixes the wall's disturbance with the roll used to *maneuver*. The fix is
to estimate the wall torque **as a state** rather than infer it from control effort.
 
**Construction.** Augment the Kalman filter's reduced state with a roll angular-
acceleration disturbance δ entering the `wx` equation:
`wx_dot = (model)·z + (model)·u + δ`, `δ_dot = 0 + noise`
(`controller.py`, `dist_obs=True`). The filter estimates δ from the part of the
roll dynamics the *known* `u` and the model do not explain, so δ̂ tracks the wall,
not the maneuver. It stays observable through the trusted roll-angle channel even
though `wx` itself is distrusted for ripple rejection. The estimate also enables a
feed-forward `u_roll_ff = −δ̂/B_roll` that cancels the wall torque directly.
 
**(A) Calibration.** Held at a sweep of standoffs, δ̂ equals the independently
measured wall torque `T_d/Ixx` essentially 1:1:
 
| wall | gap | δ̂ (rad/s²) | T_d/Ixx | ratio |
|---:|---:|---:|---:|---:|
| 16 mm | 2.7 mm | 4270 | 4150 | 1.03 |
| 20 mm | 6.7 mm | 1407 | 1385 | 1.02 |
| 24 mm | 10.7 mm | 673 | 664 | 1.01 |
| 30 mm | 16.7 mm | 300 | 297 | 1.01 |
| 38 mm | 24.7 mm | 136 | 134 | 1.01 |
 
Fitted slope 1.03 — δ̂ is a *physically calibrated* estimate, not just a monotone
proxy.
 
**(B) Distance estimation.** Fitting the δ̂(gap) law on the training standoffs and
inverting it, the estimated gap on **held-out** test standoffs matches truth to
sub-millimetre accuracy:
 
| test wall | true gap | estimated gap | error |
|---:|---:|---:|---:|
| 17 mm | 3.7 mm | 3.9 mm | +0.2 mm |
| 22 mm | 8.7 mm | 8.4 mm | −0.3 mm |
| 27 mm | 13.7 mm | 13.2 mm | −0.5 mm |
| 34 mm | 20.7 mm | 20.6 mm | −0.1 mm |
 
RMSE 0.29 mm. The flyer reads its distance to a wall, to a fraction of a
millimetre, from its own flapping dynamics — no proximity sensor. (Files:
`experiments/e16_disturbance_observer.py`, `outputs/e16_disturbance_observer.{csv,png}`.)
 
**What this revises in §5.** Limitation (1), contamination, is resolved for
*sensing*: δ̂ is clean and calibrated. Two honest updates to the rest:
- The e15 **limit cycle** turned out to be a property of the weak, contaminated
 loop, not something only feed-forward can cure. With the clean δ̂ signal *and*
 more lateral authority (`Q[vy]` raised ~30×, lateral tracking 39%→86%), the
 loop is well-damped; feed-forward's *marginal* benefit in this regime is small
 (near-wall roll holds within ~0.2° either way). So we no longer credit feed-
 forward with eliminating the limit cycle — the clean signal plus lateral
 authority does it.
- A new honest edge: raising lateral authority and cancelling the wall torque
 removes the passive repulsion the e15 demo leaned on, so **fully-active**
 standoff control now rests entirely on the (sound) sensing — and close-range
 setpoints remain touchy because δ ∝ (R/4d)² steepens sharply near contact. The
 *distance sensing* (this section) is solid; tight closed-loop standoff
 regulation at <5 mm gaps is the remaining control problem.
**Net:** the contribution's sensing claim is now much stronger — not "a directional
proximity proxy" but "a **calibrated, sub-mm wall-distance estimate** from self-
generated aerodynamics, read through a disturbance observer on the existing
controller." The reactive demonstration (e15) stands as a proof-of-concept;
publication-grade closed-loop standoff is a control-tuning task on top of this
now-calibrated sensor.
 
---
 
## 9. Extension — corridor centering from the net wall disturbance (`e17`)
 
A single wall gives a signed standoff signal; two walls give a **centering** signal
for free. With the clean wall observer (§8) and `ground_effect` extended to sum
several surfaces (κ = 1 + Σᵢ K(R/4dᵢ)², backward-compatible), the estimate
δ̂_net is the *sum* of the two walls' roll disturbances.
 
**(A) The signal.** Held across lateral offsets in a 40 mm corridor (half-width
20 mm, 6.7 mm wing-tip gap to each wall at centre), δ̂_net is a clean **odd**
function through the origin: zero on the centreline (the walls cancel), ±1347 at
±2 mm, ±8357 at ±8 mm. So the centring target is δ̂_net → 0 — a setpoint that needs
**no calibration**, unlike the standoff's non-zero one. Which side am I closer to,
and by how much, read straight off the controller's own roll estimate.
 
**(B) Centring.** Null-seeking it (`vy_ref = −K·δ̂_net`) with feed-forward on (so
flying between two walls is stable — see below), the flyer converges toward the
centreline from both sides without touching a wall:
 
| start | settled |
|---:|---:|
| +5 mm | −1.8 mm |
| −5 mm | +1.8 mm |
| +3 mm | −3.0 mm |
 
**Two honest findings the loop forced out.**
- **Flying between two walls is open-loop unstable.** Without feed-forward, the
 position-dependent wall torque feeds the fast +15.98 roll mode and a slow
 oscillation grows until the flyer is ejected into a wall's near-field. Feed-
 forward (cancelling the wall torque) is what makes the corridor flyable; on top
 of it the active null-seeker centres.
- **Stability is width-dependent and the loop is underdamped.** Centring is clean
 only where the central signal is strong (tight corridors, ~20 mm half-width);
 wider corridors have a weak central δ̂_net and a slow drift wins. Even at the good
 width the loop is underdamped — it converges with decaying oscillations and a
 small residual offset (±1.8 mm), not a crisp lock. Starts must also begin inside
 the physical corridor (|y| < 6.7 mm here); larger "starts" begin inside a wall.
**Net.** The *centring signal* is as clean as the wall sensor it is built from —
odd, zero-valued at centre, calibration-free. Closed-loop centring is a working
**proof of concept** with the same character as the e15 standoff: solid sensing,
underdamped control that is robust only in the strong-signal regime. Tight,
width-robust centring is the same open control problem (a proper lateral position
loop / gain-scheduling against the (R/4d)² nonlinearity), not a sensing gap.
 
Files: `src/ground_effect.py` (multi-surface), `src/controller.py` (multi-axis
observer), `experiments/e17_corridor_centering.py`, `outputs/e17_corridor_centering.{csv,png}