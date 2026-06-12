# Stage 5/6 Bridge — Flow-Proximity Sensing: Feasibility
 
*The evidentiary base for the project's actual contribution. Before building an
avoidance loop, we ask a physics question we genuinely did not know the answer to:
can a hovering flapping flyer feel a nearby surface from its own aerodynamics —
how strongly, how far, and can it tell which way? This section records the
reasoning, the model (and one sign trap avoided), the two experiments, the
numbers, and — deliberately — exactly what we are and are not entitled to claim.*
 
---
 
## 0. Why this is the contribution, not a side-quest
 
Quadrotors navigate narrow spaces with lidar and cameras; that is solved and not
novel. What no rotorcraft can do is use its *own wake* as a proximity sensor —
because a rotor's downwash destroys the very flow signal it would need to read,
and stirs dust/contaminants in exactly the confined settings (greenhouse
pollination, reactor inspection) where a flapping flyer's low downwash is the
whole point. So the defensible, flapping-specific contribution is:
 
> **A flapping MAV can sense confined surroundings through self-generated unsteady
> aerodynamics — a sensing modality unavailable to rotorcraft — and a reactive
> controller can close the loop on it.**
 
Narrow-space navigation is the *demonstration*; flow-based proximity sensing is
the *contribution*. Everything in this section is about establishing that the
sensing signal physically exists, is large enough, reaches far enough, and is
directional — i.e. that the contribution is real before we build on it.
 
A second, pleasant consequence we establish here: the signal is readable from the
**existing controller's own residuals** (it works harder near a surface), so no
new sensor hardware is required in the model. That is a strong, simple story.
 
---
 
## 1. The honest modelling situation (read before trusting any number)
 
Our aerodynamics (`src/aero.py`) is quasi-steady blade-element: each wing strip's
force comes from its motion through **still air**. It has **no wake / induced-
velocity term**. The direct consequence, stated plainly:
 
> A bare geometric wall placed in the MuJoCo scene would change *nothing* in our
> forces. The simulation cannot *discover* ground or wall effect, because the
> physical mechanism (the wake) is not represented. We must **add** the effect.
 
So this is, by construction, a **simulation feasibility** study, not a discovery
of physics. We add a literature-grounded surface-effect term and ask whether an
effect *of that established form and conservative magnitude* yields a usable
signal. It does not prove the real-world magnitude — that needs a wind tunnel —
but "given established ground-effect physics, the signal is usable, directional,
and reaches ~N wing-lengths" is a legitimate and publishable claim, provided we
say exactly that. The single magnitude coefficient `K_GE` is the knob to calibrate
for sim-to-real.
 
### 1.1 A sign trap we avoided
 
The "obvious" way to add ground effect is the method of images on the wing's bound
vortex. We worked it through and it gives the **wrong sign**. A 2-D spanwise bound
vortex at height `d` above a plane has an image of opposite circulation at `−d`;
the velocity it induces back at the wing is *streamwise*, and carried through it
slightly *reduces* the effective flow speed — predicting a small lift **loss**.
That contradicts every measurement of hovering ground effect (lift rises). The
lesson: the bound-vortex image is not the mechanism.
 
The correct mechanism for hovering ground effect is a **wake effect**. The wing
drives a downward induced jet (its downwash), which lowers its own effective angle
of attack. A nearby surface blocks that downward wake, the induced downwash falls,
the effective angle of attack recovers, and **lift rises**. The leading-order
magnitude scales as `(R/4d)²` (Cheeseman & Bennett 1955; the same monotone trend
is reported for flapping hover by Gao & Lu 2008 and Truong et al. 2013), with
`R` the wing length and `d` the standoff.
 
---
 
## 2. The model (`src/ground_effect.py`)
 
We implement the effect as a **per-strip lift enhancement**, applied on top of the
validated aero and exactly neutral out of ground effect:
 
$$ \kappa(d) \;=\; 1 \;+\; K_\text{GE}\left(\frac{R}{4d}\right)^2, \qquad
\kappa \to 1 \ \text{as}\ d\to\infty, $$
 
capped at `κ_max` very close to the surface (where the `(R/4d)²` form diverges and
the model breaks down anyway). For each blade-element strip we compute its
distance `d` to the surface and multiply that strip's **circulatory lift** by
`κ(d)` (drag is left unchanged — ground effect acts through the lift/induced
mechanism). `K_GE = 1` is the Cheeseman–Bennett leading term; we keep it
conservative.
 
Two properties make this the right structural choice:
 
- **OGE-neutral.** `κ → 1` far away, so Stages 2–4 (all validated out of ground
 effect) are untouched. We verified `surface=None` reproduces `aero.wing_aero`
 bit-for-bit.
- **Directional for free.** Because `κ` is applied *per strip from each strip's
 own distance to the surface*, geometry produces direction without any extra
 modelling:
 - a **floor**, flyer level → all strips at ~equal height → symmetric lift rise
   → a pure `Fz` signal;
 - a **wall** to one side, or a tilted approach → near strips enhanced more →
   asymmetric lift → a **roll torque**, i.e. a *which-side* signal.
For the flyer used here, the wing length is **R = 12.6 mm** (hinge-to-tip,
RoboBee-class), which is large relative to body scale — so the effect reaches
centimetres, not millimetres.
 
---
 
## 3. Floor / ground effect (experiment `e12`)
 
**Method.** Clamp the flyer hovering level at a range of standoffs above a floor
(`z = 0`); at each standoff measure the cycle-averaged aero wrench *with* the
surface and *without* (free air); the difference is the signal. The free-air model
is height-independent, so the difference is purely the added ground effect.
 
**Result — a clean power law.** The lift rise `ΔFz` as a fraction of weight:
 
| standoff | d / R | ΔFz / weight |
|---:|---:|---:|
| 8 mm  | 0.63 | **+11.74 %** |
| 10 mm | 0.79 | +7.92 % |
| 12 mm | 0.95 | +5.70 % |
| 15 mm | 1.19 | +3.79 % |
| 18 mm | 1.43 | +2.70 % |
| 22 mm | 1.75 | +1.85 % |
| 26 mm | 2.06 | +1.34 % |
| 32 mm | 2.54 | +0.90 % |
| 40 mm | 3.17 | +0.59 % |
| 50 mm | 3.97 | +0.38 % |
| 60 mm | 4.76 | +0.27 % |
 
On log–log axes the slope is **−2** — the `(R/4d)²` law, i.e. the model behaving
as the physics dictates rather than as an arbitrary curve. Taking a conservative
0.5%-of-weight detection threshold (a plausible resolvable cycle-averaged lift
change), the signal is usable out to **~40 mm ≈ 3.2 wing-lengths**. Because `R` is
large, that is a *centimetres-scale* proximity envelope.
 
**The ripple caveat (important and honest).** The *instantaneous* vertical force
swings ~±800 µN across a wingbeat (peak-to-peak ≈ 2× the weight), so the signal is
**invisible moment-to-moment** — it lives entirely in the cycle average. The
saving grace: that ripple is periodic and averages to ~zero over one 12.5 ms
wingbeat, so the cycle mean cleanly recovers `ΔFz`. The real consequences are
therefore (i) a **~1-wingbeat latency floor** on detection (caps reaction speed,
fine for a flyer), and (ii) the true hardware limit is **sensor resolution**, not
ripple — can a real onboard sensor resolve a 0.5–12% cycle-averaged lift shift?
That is the sim-to-real question, flagged, not hidden.
 
**Directional sanity + preview.** A symmetric level floor produces **exactly
zero** roll torque (`max|ΔTx| = 0.000 µN·mm`) — so there are no false directional
signals. Tilt the flyer 15° near the floor (one wing closer) and a clean roll
torque of **−88 µN·mm** appears. Asymmetry → torque: the mechanism the wall case
relies on.
 
Files: `experiments/e12_ground_effect_probe.py`, `outputs/e12_ground_effect.{csv,png}`.
 
---
 
## 4. Wall effect — the directional case (experiment `e13`)
 
The narrow-space claim needs *which side*, so the wall is the real test: flyer
hovering **level at 50 mm** (vertically out of ground effect, so the floor is
irrelevant and we isolate the wall), with a vertical surface swept in from the
+y side. The wingtips reach |y| = 13.3 mm over a cycle, so the wall is swept down
to a ~1.7 mm gap.
 
**Model for the wall.** The same per-strip `κ` handles any surface orientation; a
vertical surface acts as a **reflection plane** for the near wing — an
endplate / image-wing effect that reduces its tip losses and raises its lift, with
the same `(R/4d)²` proximity law. The near wing gaining lift → a **roll torque**
whose sign encodes the wall's side.
 
**Result — directional roll signal.** Reported both as the raw torque `ΔTx` and as
a *roll-command-equivalent* (the fraction of roll authority the controller would
spend cancelling it — i.e. exactly the residual an avoidance loop reads):
 
| wall gap to tip | gap / R | ΔTx (roll) | roll-cmd equiv |
|---:|---:|---:|---:|
| 1.7 mm  | 0.13 | 1218.7 µN·mm | 11.7 % |
| 2.7 mm  | 0.21 | 865.7 | 8.3 % |
| 4.7 mm  | 0.37 | 432.1 | 4.2 % |
| 6.7 mm  | 0.53 | 257.4 | 2.5 % |
| 10.7 mm | 0.85 | 119.5 | 1.15 % |
| 14.7 mm | 1.17 | 67.0 | 0.64 % |
| 20.7 mm | 1.64 | 34.0 | 0.33 % |
| 28.7 mm | 2.28 | 16.9 | 0.16 % |
| 41.7 mm | 3.31 | 7.2 | 0.07 % |
| 56.7 mm | 4.50 | 3.4 | 0.03 % |
 
Same `(R/4d)²` fall-off. Taking a 1%-of-roll-authority threshold as comfortably
readable, the directional signal is usable out to a **~11 mm gap ≈ 0.8
wing-lengths** — shorter than the floor's 3.2, as expected, since this is a torque
from a *lift asymmetry* rather than a full lift change.
 
**Directionality — the robust result.** Put the wall on +y and `ΔTx = +257 µN·mm`;
put it on −y and `ΔTx = −257 µN·mm`. The sign tracks the side exactly. The flyer
does not merely feel "a wall," it feels *which way*.
 
**Two honest caveats (in the experiment header too).**
- **`ΔFy ≈ 0` exactly.** Our model only raises the near wing's (vertical) lift, so
 the wall signal is a *pure roll torque* with zero lateral force. A real wall
 almost certainly *also* exerts a lateral suction (the Coandă "wall-suck") this
 model omits — meaning the real signal is probably **larger and richer** than
 shown. Our roll-only result is therefore **conservative**.
- **Magnitude is model-dependent; the sign is not.** The endplate/reflection
 magnitude is less settled in the literature than ground effect, so the absolute
 numbers carry calibration uncertainty (`K_GE`). The **directionality**
 (sign-flips with side, monotone with proximity) is geometric and robust.
Files: `experiments/e13_wall_effect_probe.py`, `outputs/e13_wall_effect.{csv,png}`.
 
---
 
## 5. The sensing modality: control residuals, no new sensor
 
How would the flyer *read* these signals on hardware? The elegant answer the LQG
hands us: it already does. Near a floor, the extra lift makes the altitude loop
**reduce thrust** to hold height — so the thrust command drops below its hover
nominal. Near a wall, the roll disturbance makes the controller apply a **roll
bias** to stay level. Both are directly visible in the controller's own outputs
and in the Kalman estimator's residuals; the flyer "feels" a surface as an anomaly
it is already compensating for. The roll-command-equivalent column of §4 is
literally that residual.
 
So the proposed sensing channel needs **no dedicated proximity sensor** — it reads
the controller's commands/innovations. That is both a clean engineering story and
a strong, simple novelty framing: *sensorless flow-proximity sensing via control
residuals on a flapping MAV.*
 
---
 
## 6. What we are and are not entitled to claim
 
**Entitled to claim (from these probes):**
- Given established ground-effect physics of conservative magnitude, a hovering
 flapping flyer carries a proximity signal in its own aerodynamics: a lift rise
 for a surface below (usable to ~3 wing-lengths) and a directional roll torque
 for a wall beside it (usable to ~1 wing-length, **sign = side**).
- Both follow the expected `(R/4d)²` law (a model-consistency check, slope −2).
- Both are readable as deviations the existing controller already compensates for,
 so no extra sensor is required in the model.
- The directionality is robust to the magnitude uncertainty.
**Not entitled to claim (and we say so):**
- That the real-world magnitudes are as modelled — the simulation adds the effect,
 it does not derive it; `K_GE` (and the wall's omitted lateral suction) need
 experimental calibration. This is explicitly a *simulation feasibility* result.
- Anything about wing-compliance / contact recovery — a genuinely flapping-specific
 advantage, but our wings are rigid and we model no contact, so it stays
 *motivation*, not a demonstrated result, until a contact model exists.
---
 
## 7. What this justifies — Stage 6
 
The foundation is solid: a real, physics-grounded, directional, usefully-ranged
proximity signal, readable without new hardware, on CPU, no learning required.
That is exactly the substrate Stage 6 needs:
 
1. **Sense** — read the control residuals (thrust drop ⇒ surface below/closing;
  roll bias ⇒ wall, sign ⇒ side), and convert to a proximity + direction estimate.
2. **React** — feed an avoidance reference into the LQG (climb away from a floor,
  roll/translate away from a wall), reusing the validated inner loop.
3. **Demonstrate** — the flyer dodging a wall / holding a corridor: the headline
  result, of which this feasibility section is the physical justification.
Files for this section: `src/ground_effect.py`, `experiments/e12_ground_effect_probe.py`,
`experiments/e13_wall_effect_probe.py`, and the four `outputs/e12_*`, `outputs/e13_*`
CSV/PNG artifacts.