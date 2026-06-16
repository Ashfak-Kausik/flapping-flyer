# Mechanical Design Specification (as-built in simulation)
 
This is the complete physical definition of the flyer as it exists in `models/flyer.xml`
— every body, part, dimension, mass, the centre of mass, inertia, joints, and actuation.
Units: millimetres (mm) and milligrams (mg) for readability; the model itself is in
metres/kilograms. Frame convention: **+x = forward (head), +y = left span, +z = up**.
 
> Read the companion drawing `mechanical_drawing.svg` (top / side / front orthographic
> views, dimensioned) alongside this table.
 
---
 
## 1. Mass & balance summary
 
| Quantity | Value |
|---|---|
| Total mass | **77.40 mg** |
| Weight | 759.3 µN |
| Centre of mass (from thorax origin) | (+0.05, 0, +0.03) mm — essentially the thorax centre |
| CoM vs wing hinge plane | ~1.2 mm **below** the hinges (pendulum-stable by design) |
| Roll inertia  I_xx | 1.71 × 10⁻¹⁰ kg·m² |
| Pitch inertia I_yy | 6.04 × 10⁻¹⁰ kg·m² |
| Yaw inertia   I_zz | 7.31 × 10⁻¹⁰ kg·m² |
 
This is **RoboBee class** (Harvard RoboBee ≈ 80–100 mg), an order of magnitude heavier
than a real mosquito (~2 mg). The thorax block carries 60 of the 77 mg — it is the
"flight mass," and in a real build it is where structure/payload would live.
 
## 2. Part inventory
 
All solids are simple primitives. Masses are **prescribed** (not derived from a uniform
density), i.e. each part is treated as a lightweight shell/structure tuned to a mass
budget rather than a solid casting — see note in §6.
 
| Part | Shape | Full dimensions (mm) | Position from thorax origin (mm) | Mass (mg) |
|---|---|---|---|---|
| Thorax | ellipsoid | 4.0 (L) × 2.4 (W) × 2.4 (H) | (+1.0, 0, 0) | 60.0 |
| Head | sphere | Ø 1.6 | (+4.0, 0, 0) | 3.0 |
| Tail / abdomen | ellipsoid | 10.0 (L) × 1.2 × 1.2 | (−5.5, 0, 0) | 12.0 |
| Haltere ×2 | capsule | Ø 0.24, ~1.3 long | (−1.2, ±1.15, −0.2) | ~0.1 ea |
| Wing hub ×2 (stroke) | point inertia | — | hinge at (0, ±1.0, +1.2) | 0.1 ea |
| Wing membrane ×2 | ellipsoid | 3.5 (chord) × 12.0 (span) × 0.05 (thick) | centroid (−0.875, ±7.6, +1.2) | 1.0 ea |
 
Body envelope (nose to tail): head front +4.8 mm to tail tip −10.5 mm ⇒ **~15.3 mm long**.
 
## 3. Wing geometry (each wing)
 
| Quantity | Value |
|---|---|
| Planform | elongated ellipse (oval) |
| Chord (max, x) | 3.5 mm |
| Span (root→tip of membrane, y) | 12.0 mm |
| Thickness | 0.05 mm (50 µm membrane) |
| Planform area | ≈ 33 mm² |
| Hinge → wingtip reach | ≈ 13.6 mm (measured max tip |y| = 13.3 mm) |
| **Full wingspan tip-to-tip** | **≈ 27 mm** |
| Pitch (feather) axis | at the quarter-chord (membrane shifted −c/4 in x) |
| Radial area bias | centroid pushed to ~0.55·R outboard |
 
## 4. Joints & degrees of freedom
 
Each wing has **two hinges in series** at the shoulder (4 actuated DOF total):
 
| Joint | Axis | Range | Function |
|---|---|---|---|
| stroke_R / stroke_L | z (vertical) | ±90° | sweep fore/aft (the flap) |
| pitch_R / pitch_L | y (span) | ±120° | feather / flip (angle of attack) |
 
Plus, when free-flying, the body has a 6-DOF free joint (the `<freejoint/>`); the four
hinges are the only **actuated** DOF.
 
## 5. Actuation & operating point (hover)
 
| Quantity | Value |
|---|---|
| Flap frequency | 80 Hz |
| Stroke amplitude (hover trim) | ±72.6° |
| Feather amplitude | ±45° |
| Yaw control | split-cycle (asymmetric half-stroke timing) |
| Roll control | differential stroke amplitude |
| Pitch control | differential stroke offset |
| Thrust control | common stroke amplitude |
 
There is **no aerodynamic model in the physics engine** — lift/drag are computed by our
blade-element code (`src/aero.py`) over spanwise strips and injected. The mechanical model
is geometry + mass + joints only.
 
## 6. Notes for a real build
 
- **Hollow / shell construction.** The prescribed masses imply hollow or skeletal parts,
 not solid castings: a solid 4×2.4×2.4 mm ellipsoid at ~1200 kg/m³ would be ~15 mg, but
 the thorax is set to 60 mg — i.e. it represents a dense flight-mass/structure, while the
 tail and wings are far lighter than solid. A real airframe would be a carbon-fibre or
 SU-8 skeleton with a thin membrane (the 50 µm wing is consistent with a polymer film on a
 vein frame, as in RoboBee).
- **Wing membrane.** 50 µm thick, ~33 mm² — a film (e.g. polyester/parylene) on a stiffer
 spar/vein layout, not a solid plate.
- **Hinges.** Modelled as ideal revolute joints. Real insect-scale "joints" are usually
 **flexure hinges** (compliant flexures), not bearings, and the drive is a resonant
 actuator (piezo bender in RoboBee), not a rotary motor.
- **The big open item — onboard mass — is treated separately (see the payload analysis).**
 The 77 mg budget here includes **no** battery, controller, wiring, or sensors.