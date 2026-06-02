# Stage 1 — The Body Model (Design Record)
 
**File produced:** `models/flyer.xml`
**Check script:** `experiments/e01_load_model.py`
**Status:** built and validated (loads; numbers confirmed).
 
> This document defends every number in `flyer.xml`. The rule: no magic
> constants. If a value isn't justified here, it's a bug waiting to happen.
 
---
 
## 1. What Stage 1 is (and is not)
 
The body file is **geometry + mass + joints**, nothing else. It deliberately
does **not** contain:
 
- **aerodynamics** — MuJoCo has none; we inject it in Stage 2.
- **flapping frequency or amplitude** — those are *kinematics*, set by the
 driver in Stage 2, not properties of the body.
So Stage 1 answers exactly one question: *what does the flyer physically look
like, and how are its parts hinged together?*
 
---
 
## 2. The scale decision (the one real choice here)
 
True mosquito scale is **not** our build target, and we should be explicit about
why. Real insects span an enormous range:
 
| Flyer | Body mass | Wing length | Wingbeat | Stroke amp. | Reynolds |
|---|---|---|---|---|---|
| Fruit fly (*Drosophila*) | ~1 mg | ~2.5 mm | ~210 Hz | ~140° | ~150 |
| **Mosquito** (*Aedes/Culex*) | **~2 mg** | **~2.8 mm** | **~600 Hz** | **~40°** | **~70** |
| Honeybee | ~90 mg | ~9.5 mm | ~230 Hz | ~90° | ~1100 |
| Hawkmoth (*Manduca*) | ~1.6 g | ~50 mm | ~26 Hz | ~115° | ~4000 |
| **RoboBee** (Harvard) | **~80–100 mg** | **~15 mm** | **~120 Hz** | **~110°** | ~1500 |
| RoboFly (UW) | ~74 mg | ~13 mm | ~150 Hz | — | — |
 
*(Values are representative round figures from the literature, for orientation,
not precise species constants.)*
 
Two facts drive the decision:
 
1. **The mosquito is a physics outlier.** It is tiny, beats its wings
  *extraordinarily* fast (~600 Hz), and uses an unusually *low* stroke
  amplitude — which is exactly why it leans on rotational lift mechanisms
  rather than the translation-dominated lift of other insects. That makes it
  fascinating (and central to our novelty), but a brutal *first* build target.
2. **Wingbeat frequency sets the simulation timestep.** To integrate a flapping
  cycle accurately we need ~100–200 timesteps per wingbeat:
  ```
  dt  ≲  1 / (150 · f)
  ```
 
  | Target | f | required dt | steps for 1 s of sim |
  |---|---|---|---|
  | Mosquito | 600 Hz | ~1.1e-5 s | ~90,000 |
  | RoboBee | 120 Hz | ~5.6e-5 s | ~18,000 |
  | Our dev point | ~40–80 Hz | ~1e-4 s | ~10,000 |
  Control development means running *many seconds* of sim across *many*
  tuning iterations. A 10× smaller timestep is a 10× slower project. Speed of
  iteration matters more than biological fidelity while we are still learning
  to fly.
**Decision.** Build at a **RoboBee-class development scale** (~80 mg, ~12–15 mm
wing). Rationale:
 
- It is the scale of the only insect-robots that have *actually flown*, so our
 results are comparable to a real hardware lineage.
- Its frequency (~120 Hz, and we can develop even slower) keeps the timestep and
 iteration speed sane.
- **All parameters remain adjustable.** Once we can fly and stabilise at this
 scale, the *mosquito-specific regime* (high frequency, low amplitude,
 rotation-dominated) becomes a deliberate experiment for the novelty work —
 not a constraint we fight from day one.
> This is a choice, not a law. If you want to argue for a different scale, this
> is the section to argue with.
 
---
 
## 3. Every number in `flyer.xml`, defended
 
### Wing
| Parameter | Value | Justification |
|---|---|---|
| Wing length `R` | 12 mm | RoboBee class (15 mm); slightly smaller toward insect range. |
| Mean chord `c` | 3.5 mm | Gives aspect ratio R/c ≈ 3.4. |
| Aspect ratio | 3.43 | Single-wing insect aspect ratios run ~2.5–4; we sit mid-range. |
| Thickness | 50 µm | A thin flat plate — insect wings are membranes; the flat-plate model (Stage 2) assumes this. |
| Wing mass | 1.0 mg each | ~1.25 % of body mass; insect wings are ~0.5–2 %. Kept non-zero and well-conditioned for the solver. |
| **Pitch axis** | **quarter-chord** | Geom offset in y by −c/4 puts the hinge at the 1/4-chord line. This is the realistic wing torsion axis, and it sets the rotational-lift coefficient `C_rot = π(0.75 − 0.25) = π/2 ≈ 1.57` (Stage 2, Part 7). |
 
### Thorax (body)
| Parameter | Value | Justification |
|---|---|---|
| Mass | 80 mg | RoboBee original airframe mass. |
| Shape | capsule, 8 mm long, 1.8 mm radius | A simple, smooth proxy for an insect thorax; gives a reasonable moment of inertia for the control stage. |
 
### Joints (the hinge structure)
Each wing has **two** hinges in series — this is the minimum to reproduce insect
wing motion:
 
1. **Stroke hinge** (`axis = 0 0 1`, vertical): sweeps the wing fore-and-aft in a
  roughly horizontal **stroke plane**. This is the big back-and-forth flapping
  motion.
2. **Pitch hinge** (`axis = 1 0 0`, along the span): twists/feathers the wing
  about its long axis. This is the *flip* at each reversal that lets the wing
  keep a useful angle of attack in both directions (the source of net lift, and
  the home of rotational lift).
> Why a horizontal stroke plane? It is the simplest hovering configuration
> (used by hummingbirds and many insects): a horizontal sweep + pitch reversal
> produces vertical lift. Stroke-plane *tilt* is a real control/locomotion knob
> we can add later; we start simple.
 
### The intermediate "stroke link"
Between the thorax and each wing there is a near-massless intermediate body that
carries the stroke hinge. It is given a tiny inertia (1e-7 kg):
 
> **Lesson already paid for:** a body that carries a joint but has *zero* mass or
> inertia makes the MuJoCo solver blow up (`mass and inertia of moving bodies
> must be larger than mjMINVAL`). Any hinged link needs a non-zero inertia. This
> will become a note in `tests/`.
 
### The clamp (no free joint yet)
The thorax has **no joint to the world** → it is fixed in place. This is correct
for Stage 2, where we validate aerodynamic forces on a held body. **Stage 3**
replaces the clamp with a `<freejoint/>` to release the flyer into free flight.
 
### Actuators
Four position actuators (one per hinge) are defined so the joints are drivable.
Whether we *torque-servo* them or *prescribe* the joint angles kinematically is
a Stage 2 decision (the prototype showed servo gains fight the tiny wing
inertias, so we will likely prescribe kinematics first).
 
---
 
## 4. The Stage 1 check (what "done" means)
 
Run:
 
```bash
python experiments/e01_load_model.py          # prints the numbers
python experiments/e01_load_model.py --view    # opens the viewer
```
 
**Pass conditions:**
 
- Model loads with no error.
- Reported numbers match this document:
 total mass ≈ 82 mg, thorax 80 mg, each wing 1 mg, R = 12 mm, c = 3.5 mm,
 aspect ratio ≈ 3.43, 4 joints, 4 actuators, 4 DOF.
- In `--view`, it visibly looks like a small body with two wings hinged at the
 shoulders.
**Confirmed values (sandbox run):**
 
```
total mass        :    82.20 mg
  thorax          :    80.00 mg
  each wing        :     1.00 mg
wing length  (R)  :    12.00 mm
mean chord   (c)  :     3.50 mm
aspect ratio R/c  :     3.43
joints : ['stroke_R', 'pitch_R', 'stroke_L', 'pitch_L']
```
 
---
 
## 5. Open questions parked for later
 
- **Stroke-plane tilt** — currently horizontal; a tilt parameter is a future
 control/locomotion knob.
- **Wing planform** — currently a rectangle; real wings taper, which changes the
 spanwise lift distribution (the "moments of area"). A refinement, not a
 blocker.
- **Inertia fidelity** — the capsule thorax inertia is a reasonable proxy; if
 the control stage proves sensitive to it, we revisit with measured insect
 inertia values.