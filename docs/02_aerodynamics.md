# stage2-aerodynamics
Bio-Inspired Flapping-Wing Micro-Flyer — Stage 2: Aerodynamic Model. Derives the quasi-steady blade-element model from first principles, from the single question "what is lift?" through all three force terms (translational, rotational, added-mass) injected into MuJoCo, with every step motivated before it appears in code.

## Purpose

Build the quasi-steady blade-element model from the ground up — motivating every step so that every line of `aero.py` feels inevitable, not magic. By the end, the three force terms we inject into MuJoCo should have a clear physical origin, a derivation, and a connection to the code structure.

## Notation

| Symbol | Meaning | Typical value / units |
|--------|---------|----------------------|
| ρ | air density | ≈ 1.2 kg/m³ |
| U | speed of a wing piece through the air | m/s |
| α | angle of attack | radians or degrees |
| S | a reference area | m² |
| c | chord (wing width, front-to-back) | m |
| Ω | flapping angular speed | rad/s |
| ψ̇ | wing pitch / rotation rate | rad/s |
| q | dynamic pressure, ½ρU² | Pa |
| Re | Reynolds number | dimensionless |
| x̂ (hat) | a unit vector — a direction of length 1 | — |

## Theory

### Part 1 — What a wing actually feels: lift and drag

When any object moves through air, the air pushes back with a single resultant force. We never use that resultant directly. We always split it into two pieces, defined relative to the direction the air flows past the object (the relative wind):

- **Drag** — the component parallel to the flow. It opposes motion.
- **Lift** — the component perpendicular to the flow.

> **The single most important point:** Lift is perpendicular to the airflow — not to the ground, not to the wing.

For an airplane in level flight the flow is roughly horizontal, so "lift" happens to point up. For a flapping wing, the wing sweeps forward, back, up, and down while constantly twisting, so the lift vector points in a constantly changing direction. We must always compute the flow direction first, then define lift perpendicular to it. This is precisely where the prototype had a sign bug — see Part 10.

---

### Part 2 — How big is the force? The force equation

Aerodynamic force on a surface is, in every case we care about:

```
F = ½ · ρ · U² · S · C
```

| Factor | Name | Effect | Scaling |
|--------|------|--------|---------|
| ρ | air density | more molecules to push against → more force | linear |
| U² | speed squared | double speed, quadruple force | quadratic |
| S | area | bigger surface intercepts more air | linear |
| C | coefficient | packs in all shape and angle effects | dimensionless, ~0–2 |

Why the square on U? Faster motion means both (a) you strike more air per second and (b) you strike it harder — two factors of U, multiplied. The grouping ½ρU² is called dynamic pressure (q); it is literally the kinetic energy per unit volume of the oncoming air.

`C` is a dimensionless number that packs in everything about shape and angle — one for lift (C_L) and one for drag (C_D). All the interesting physics of how a wing works lives in how these depend on the angle of attack. So lift and drag are:

```
L = ½ ρ U² S · C_L(α)
D = ½ ρ U² S · C_D(α)
```

Everything from here is about (i) getting U and α right for a flapping wing, and (ii) getting the C's right for an insect.

---

### Part 3 — Why an insect is not a tiny airplane: the Reynolds number

One number decides what kind of aerodynamics you are in:

```
Re = ρ U L / μ  =  U L / ν
```

(L is a characteristic length — for us, the chord c; μ is air's viscosity; ν = μ/ρ.) The Reynolds number is the ratio of inertial forces to viscous forces.

| Flyer | Reynolds number | What air "feels like" |
|-------|-----------------|----------------------|
| Passenger jet | ≈ 10,000,000 | thin, slippery |
| Mosquito | ≈ 50–150 | almost syrupy |

This is not a small quantitative difference — it changes the rules. The headline consequence: an airplane wing stalls past roughly 15°; an insect wing does not. At low Re and high angle of attack, insects sustain a **leading-edge vortex (LEV)** — a stable tornado of air sitting on top of the wing that sucks it forward, producing huge lift at angles where a plane would fall out of the sky. Insects routinely fly at angles of attack of 35–45°.

We do not simulate the LEV directly (that needs full CFD). Instead we bake its effect into the coefficients C_L(α) and C_D(α) — which is what the next part does, and why our coefficients look different from an airplane textbook's.

---

### Part 4 — The coefficients, derived

The otherwise arbitrary-looking coefficient formulas become inevitable once you follow three steps.

**Step 1 — Model the wing as a thin flat plate.** At the high angles of attack and low Re of insect flight, the air's resultant force on a flat plate points normal to the plate. Call this normal force N. Empirically, its coefficient grows with the sine of the angle of attack:

```
C_N(α) = A · sin(α)        (A ≈ 3.5 for insect-like plates)
```

**Step 2 — Resolve into lift and drag (just geometry).** Set the flow along a horizontal axis. The plate sits at angle α, so the plate's normal makes angle α with the lift direction. Project N:

```
C_L = C_N cos(α) = A sin(α) cos(α)
C_D = C_N sin(α) = A sin²(α)
```

**Step 3 — Apply two trig identities** (`sin(α)cos(α) = ½ sin(2α)` and `sin²(α) = ½(1 − cos 2α)`):

```
C_L(α) = (A/2) · sin(2α)
C_D(α) = (A/2) · (1 − cos 2α)
```

These are exactly the shapes used in the prototype (`CL = 1.8·sin(2α)`, `CD = 0.2 + 1.5·(1 − cos 2α)`):

| Coefficient | Behavior | Physical reading |
|-------------|----------|-----------------|
| C_L ∝ sin(2α) | peaks at 45°, zero at 0° and 90° | edge-on or face-on → no lift; max lift at 45° |
| C_D ∝ (1 − cos 2α) | min at 0°, max at 90° | knife-through-air → barn-door |

The sin(2α) lift curve — peaking at 45°, not 15° — is the low-Re / LEV signature. An airplane wing's curve peaks near 15° then collapses; completely different.

> **On the constants:** the small fudge factors (0.2 and 1.8 vs 1.5) come from real RoboFly force measurements (Dickinson, Lehmann & Sane 1999). The code uses the measured, published fits and cites them. Both the idealized (A ≈ 3.5) and the empirically tuned forms are "right" — for different purposes.

---

### Part 5 — One wing, many speeds: the blade-element method

A flapping wing pivots about its root, so different parts move at different speeds — the tip races around while the root barely moves. A point at distance r from the root moves at:

```
U(r) = Ω · r
```

There is no single U for the whole wing. The fix is the **blade-element method**:

1. Chop the wing into N thin spanwise strips ("blade elements"), each at radius rᵢ, width dr, local chord c(rᵢ), and area dS = c · dr.
2. For each strip, compute its own local velocity, angle of attack, and forces via the force equation.
3. Sum the strips for total force; sum rᵢ × dFᵢ for total torque about the root.

This is exactly the `for r in r_stations:` loop in the prototype — now you know why it loops over strips and why each strip recomputes velocity and angle.

**A beautiful consequence — for free.** Force on a strip ∝ U² ∝ (Ωr)². Summing over the span, total force ∝ Ω² (times a fixed geometry integral). Since Ω ∝ f (flapping frequency):

```
Lift  ∝  f²
```

This is testable, and we already tested it. Going 30 Hz → 40 Hz should multiply lift by (40/30)² = 1.78×:

| Frequency | Lift/weight (measured) |
|-----------|----------------------|
| 30 Hz | 0.59 |
| 40 Hz | 1.05 |

Ratio = 1.05 / 0.59 = **1.78**. Bang on. That agreement is real evidence the model is wired correctly — the kind of internal consistency check we keep using and logging for the paper.

---

### Part 6 — The quasi-steady assumption

"Quasi-steady" means: at each instant, compute forces as if the wing sat in a steady, unchanging flow at its current velocity and angle — then step forward and repeat. We ignore the flow's memory (the swirling wake left behind) except through a few explicit correction terms.

Why are we allowed to? Because the dominant lift mechanism — the leading-edge vortex — is itself roughly stable through the stroke, so "instantaneous steady flow" is a decent stand-in. It is the accepted, validated simplification for insect flight (Sane & Dickinson), and it is what lets us avoid full CFD.

What does the pure translational model miss? Two unsteady effects that matter most near stroke reversal — the instant the wing flips direction at the end of each half-stroke. Adding them back gives us Terms 2 and 3:

```
Total per-strip force  =  translational   (Parts 4–5)
                        + rotational       (Part 7)
                        + added mass       (Part 8)
```

---

### Part 7 — Term 2: Rotational lift (the Kramer effect)

During each stroke reversal, the wing rapidly rotates about its own long axis — it "flips" so the correct edge leads on the way back. A wing that is simultaneously translating and rotating generates extra circulation, and therefore extra lift, beyond the translational term.

**Intuition — the Magnus effect.** A spinning ball curves in flight because its spin drags air around it, biasing the flow into a sideways force. A flipping wing does the rotational analogue.

The per-strip formula (Sane & Dickinson 2002, from classical thin-airfoil theory):

```
F_rot = C_rot · ρ · U · c² · ψ̇ · dr
  with   C_rot = π · (0.75 − x̂₀)
```

where ψ̇ is the wing's rotation rate and x̂₀ is the rotation-axis position along the chord as a fraction (0 = leading edge, 1 = trailing edge). For a mid-chord axis x̂₀ = 0.5:

```
C_rot = π · 0.25 ≈ 0.785
```

**The critical structural fact.** Rotational lift is proportional to U × ψ̇ — it needs the wing translating fast and rotating fast at the same instant. In the first run this term came out exactly zero: the wing did all its rotating right at reversal where U ≈ 0, so the product U·ψ̇ ≈ 0 everywhere. The fix is **rotation timing** — rotating the wing slightly before reversal ("advanced rotation") so it is still moving fast while it flips. This is not a tuning trick; it is how real insects augment lift, and it hands us a genuine control knob (the stroke–pitch phase) to exploit in the control stage.

> ⚠️ There was also a real sign bug stacked on top of this — see Part 10.

---

### Part 8 — Term 3: Added mass (the acceleration reaction)

To shove the wing through the air, the wing must also shove a slug of air clinging to it into motion. That air has inertia and pushes back. This is the **added-mass** (or virtual-mass) force — a pure reaction to acceleration, indifferent to steady speed, sensitive only to how fast the wing's velocity is changing.

For a flat plate accelerating face-first, the clinging air slug is approximately a cylinder of air with diameter equal to the chord c. Its mass per unit span:

```
m_add = ρ · π · (c/2)²  =  ρ · π · c² / 4      (per unit span)
```

The force is mass × acceleration, opposing the motion:

```
F_am = − (ρ π c² / 4) · dr · a_n
```

where a_n is the strip's acceleration normal to its own face.

**Two consequences to hold onto:**

- **Largest at stroke reversal** — that is where the wing decelerates to a stop and re-accelerates the other way (maximum |a_n|).
- **Averages to ≈ zero over a full cycle** — acceleration integrated over periodic motion nets out. Added mass barely changes average lift, but strongly shapes within-cycle force timing — which matters for control and for structural loads.

We saw exactly this in the ablation: added mass contributed ~185 µN RMS, spiking at reversals, with a cycle-mean near zero. Textbook behavior.

---

### Part 9 — Assembling the whole model

For each strip, the total aerodynamic force is the vector sum:

```
dF = dF_translational + dF_rotational + dF_added_mass
```

| Term | Expression | Source |
|------|-----------|--------|
| Translational | ½ρU²·dS·(C_L·lift̂ + C_D·draĝ) | Parts 4–5 |
| Rotational | C_rot·ρ·U·c²·ψ̇·dr · lift̂ | Part 7 |
| Added mass | −(ρπc²/4)·dr·a_n · normal̂ | Part 8 |

Then for the whole wing:

```
Total force   F = Σᵢ dFᵢ
Total torque  T = Σᵢ (rᵢ − r_CoM) × dFᵢ
```

We hand (F, T) to MuJoCo via `xfrc_applied` on each wing body, every timestep, for each wing. MuJoCo does the rest — moving the bodies under those forces, plus gravity, plus the joints. **MuJoCo never computes a single aerodynamic force itself. That is the whole reason this document exists.**

---

### Part 10 — Reference frames and the sign trap

Every vector above — velocity, lift̂, draĝ, normal̂ — must be expressed in a consistent frame, and the directions must carry the correct sign for each wing. The left and right wings are mirror images, so a convention that is right for one is backwards for the other if you are not careful.

We already hit this hard. Both wings' rotational-lift contributions came out equal and opposite and cancelled to exactly zero in the sum — which masquerades as "the term does nothing."

> **The rule (this will become a test in `tests/`):** Never hand-flip signs per wing if you can avoid it. Derive each wing's motion from its actual angular velocity projected onto its own span axis — `ψ̇_eff = ω · span̂`. That is automatically correct per-wing, and it keeps working in free flight (Stage 3+) when the body itself rotates, where hand-coded signs would silently break.

This is the deep reason Stage 3 (free flight) would have been treacherous to attempt before the aero model was decomposed and verified: a frame bug here corrupts forces right at reversal — the worst place — and you would blame the controller for days.

---

### Part 11 — What this means for the code

When we (re)write `src/aero.py`, it will be a small set of pure functions (no hidden state, easy to test), structured exactly like this document:

1. `coefficients(alpha)` → returns C_L, C_D — Part 4
2. A loop over strips, computing per strip:
   - local velocity and speed U — Part 5
   - angle of attack α — Parts 1, 5
   - translational dF — Parts 2, 4
   - rotational dF — Part 7
   - added-mass dF — Part 8
3. Sum to F, T — Part 9
4. All directions derived from frames — no per-wing sign hacks — Part 10

Every one of those steps now has a "why" you can point to. When a number looks wrong, we will not poke randomly — we will ask which part is lying and check that part in isolation. That is the entire method: decompose, verify each piece, and never build on an unverified layer.

---

## Key Findings

- The nominal quasi-steady model reproduces the f² lift scaling exactly: 30 Hz → 40 Hz gives 1.78× lift, matching the (40/30)² = 1.78 prediction to three significant figures.
- Rotational lift (Kramer effect) contributes meaningfully only when stroke and pitch are phased so U and ψ̇ are both large at the same instant — "advanced rotation" is not optional, it is the mechanism.
- Added mass contributes ~185 µN RMS, spikes at stroke reversals, and averages to approximately zero per cycle — exactly as the flat-plate theory predicts.
- A sign error in the frame convention caused both wings' rotational contributions to cancel identically to zero, demonstrating why per-wing sign hacks are forbidden and frame-derived projections are required.

## Project Structure

| Stage | Directory | Purpose |
|-------|-----------|---------|
| 0 | `stage0-repository/` | Repository setup and conventions |
| 1 | `stage1-flyer-model/` | MuJoCo body model (`flyer.xml`), dimensions justified against insect data |
| 2 | `stage2-aerodynamics/` | Quasi-steady blade-element model, coefficient derivation, three-term force assembly |
| 3 | `stage3-free-flight/` | Free-flight dynamics, body-frame integration, MuJoCo coupling |
| 4 | `stage4-control/` | Stroke–pitch phase control, lift modulation |
| 5 | `stage5-sensing/` | Aerodynamic imaging, surface detector concept |
| 6 | `stage6-paper/` | Paper drafts, figures, and submission materials |

Theory notes and references live in `notes/`.

## References

Each reference is cited at the point of use in the derivations above.

- **Dickinson, M.H., Lehmann, F.-O., Sane, S.P. (1999).** Wing rotation and the aerodynamic basis of insect flight. *Science* 284, 1954–1960. — RoboFly force measurements behind the coefficient fits.

- **Sane, S.P., Dickinson, M.H. (2002).** The aerodynamic effects of wing rotation and a revised quasi-steady model of flapping flight. *J. Exp. Biol.* 205, 1087–1096. — the three-term quasi-steady model we implement.

- **Sane, S.P. (2003).** The aerodynamics of insect flight. *J. Exp. Biol.* 206, 4191–4208. — review; the LEV story.

- **Ellington, C.P. (1984).** The aerodynamics of hovering insect flight (series). *Phil. Trans. R. Soc. Lond. B.* — classical blade-element foundations and moments-of-area.

- **Whitney, J.P., Wood, R.J. (2010).** Aeromechanics of passive rotation in flapping flight. *J. Fluid Mech.* 660, 197–220. — added-mass and rotation detail; RoboBee lineage.

- **Nakata, T., Phillips, N., Simões, P., Russell, I.J., Cheney, J.A., Walker, S.M., Bomphrey, R.J. (2020).** Aerodynamic imaging by mosquitoes inspires a surface detector for autonomous flying vehicles. *Science* 368, 634–637. — Stage 5 sensing inspiration. Previously demonstrated only on a quadrotor, never on a flapping flyer — that is the novelty gap this project targets.

- **Bomphrey, R.J., Nakata, T., Phillips, N., Walker, S.M. (2017).** Smart wing rotation and trailing-edge vortices enable high frequency mosquito flight. *Nature* 544, 92–95. — explains why mosquitoes fly at ~800 Hz with low stroke amplitudes (~40°, less than half the honeybee): they rely on trailing-edge vortex capture and rotational lift rather than the translation-dominated LEV used by most insects. Directly motivates the rotational-lift term (Part 7) and confirms that the "mosquito instinct" points at something aerodynamically real.

- **van Breugel, F., Riffell, J., Fairhall, A., Dickinson, M.H. (2015).** Mosquitoes use vision to associate odor plumes with thermal targets. *Current Biology* 25(16), 2123–2129. — behavioral neuroscience of mosquito sensing; informs how sensing and flight are coupled at the organism level, relevant to the Stage 5 surface-detector concept.

- **Phan, H.V., Kang, T., Park, H.C. (2017).** Design and stable flight of a 21 g insect-like tailless flapping wing micro air vehicle with angular rates feedback control. *Bioinspiration & Biomimetics* 12, 036006. DOI: 10.1088/1748-3190/aa65db. — KUBeetle platform; demonstrates that a tailless insect-like FW-MAV is inherently unstable without feedback and quantifies the angular-rate control requirements. Direct hardware precedent for our design scale and control architecture.

- **Nekoo, S.R., Rashad, R., De Wagter, C., Fuller, S.B., de Croon, G., Stramigioli, S., Ollero, A. (2025).** A review on flapping-wing robots: Recent progress and challenges. *The International Journal of Robotics Research* 44(14), 2305–2339. DOI: 10.1177/02783649251343638. — comprehensive review covering prototyping, modeling, navigation, and control of flapping-wing flying robots; benchmarks the state of the field against which this project positions itself.

- **Cai, J., Sangli, V., Kim, M., Sreenath, K. (2025).** Learning-based trajectory tracking for bird-inspired flapping-wing robots. *ACC 2025 (American Control Conference)*. — model-free reinforcement learning framework for a high-DoF ornithopter; achieves multimodal flight and agile trajectory tracking in simulation. Directly comparable to our learned-residual approach but at bird scale and with full RL rather than a small offline-trained MLP.

- **Kang, D. et al. (2024).** Wing-strain-based flight control of flapping-wing drones through reinforcement learning. *Nature Machine Intelligence* 6, 992–1005. DOI: 10.1038/s42256-024-00893-9. — demonstrates that wing strain (mimicking insect campaniform sensilla) provides attitude and aerodynamic load information sufficient for RL-based flight control; biologically grounded sensing approach relevant to Stage 5.

- **Tu, Z., Fei, F., Zhang, J., Deng, X. (2019).** Acting is seeing: Navigating tight space using flapping wings. *arXiv:1902.08688*. — first flapping-wing robot to use its own wings for environmental perception (wall, ground, obstacle detection) via motor current feedback, without vision. Direct precedent for wing-loading-based sensing; platform is the Purdue Hummingbird (17 cm wingspan, 12 g, 30–40 Hz).