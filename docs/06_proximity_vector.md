# 06 — The Proximity Vector: omnidirectional, calibrated, robust flow-sensing
 
**Headline.** A flapping micro-flyer extracts a **calibrated 3D proximity vector** —
the bearing to nearby surfaces in the horizontal plane plus vertical range — from
the residuals of its own flight controller. No proximity sensor, no learning,
classical control, CPU-only. The signal is robust to surface geometry and to the
magnitude of the aerodynamic effect we model. This is the core contribution
("Arc A"); narrow-space navigation (Arc B) is the demonstration built on top.
 
One figure: `outputs/arcA_summary.png`. Experiments: `e18`–`e21`.
 
---
 
## 0. Thesis — a vector, not a map
 
A body-integrated aerodynamic disturbance contains only ~6 numbers (three forces,
three torques). You **cannot** reconstruct a lidar-style map of the surroundings
from six numbers — it is information-theoretically impossible. What you *can*
extract is a low-dimensional **proximity vector**: the direction and strength of the
*net* aerodynamic influence of nearby surfaces. "Surfaces are net to my front-left
and below, and close." A compass and rangefinder, not a map.
 
This is not a limitation to apologise for; it is the honest — and bio-plausible —
characterisation. Insects do not SLAM through clutter; they react to flow and
mechanosensory fields. A flapping MAV reading a net proximity vector from its own
control residuals and steering toward open space is exactly that strategy, and it is
a sensing modality unavailable to rotorcraft, whose downwash destroys the flow
signal and stirs dust in the same tight spaces. Crucially, the *direction* of the
vector survives even when its *magnitude* (the modelled effect strength, the surface
shape) does not — which is what makes the modality trustworthy (§4, §5).
 
---
 
## 1. Mechanism — sensing from the disturbance observer
 
A nearby surface enhances the lift of the wing strips closest to it (the modelled
ground/surface effect, `ground_effect.py`, κ(d) = 1 + Σᵢ K_GE·(R/4dᵢ)²). That
enhancement appears on the body as a disturbance **wrench**, which the LQG
controller must counter. The controller's Kalman filter is augmented with
disturbance states on chosen axes (`controller.py`, `dist_states`); each estimates
the surface-induced acceleration on its axis from the *trusted, ripple-free*
channel for that axis:
 
| axis | disturbance state | read from | senses |
|---|---|---|---|
| roll  (wx) | `roll_dist`  | roll **angle** (wx distrusted) | lateral surfaces (left/right) |
| pitch (wy) | `pitch_dist` | pitch **angle** (wy distrusted) | fore/aft surfaces |
| vert  (vz) | `floor_dist` | **height** (vz distrusted) | surfaces above/below |
 
The recurring trick on every axis: **distrust the noisy rate channel** (which
carries the huge wingbeat ripple) and read the disturbance from the smooth,
integrated channel (angle, height). This is why the estimates are clean enough to
calibrate rather than merely correlate.
 
---
 
## 2. A1 — the 2D horizontal compass (`e18`)
 
A single flat wall swept to eight bearings around the hovering flyer. Roll+pitch
disturbance estimates form a 2-vector; its direction recovers the wall's bearing.
 
- **Result:** cardinals exact, diagonals within ~17°, **mean error 9°**.
- **The physical finding — anisotropy 11:1.** Lateral sensitivity (S_roll ≈ 1286)
 is ~11× the fore/aft (S_pitch ≈ 113), because the wings are long in span and short
 in chord: a wall to the side sits at the wingtips; a wall ahead is far from the
 short chord. The sensing footprint is *shaped like the wing*. Consequences:
 bearing recovery needs a one-time **per-axis calibration**; fore/aft obstacles are
 sensed at shorter range; and wing aspect ratio becomes a *sensing* design lever.
- Pitch carries a constant free-air bias (≈ +42 rad/s², from pitch trim) that
 subtracts cleanly; roll's bias is ~0.
---
 
## 3. A2 — de-confounding the vertical axis → the 3D vector (`e19`)
 
The vertical axis failed in Stage 6 because the observer read it from `vz`, which
carries the ±10 m/s² vertical wingbeat ripple. The fix mirrors roll: **distrust
`vz`, read the vertical disturbance from the ripple-free height channel**, and
subtract the constant +0.391 m/s² hover-trim bias.
 
- **Calibration:** ε̂ = 1.04·ε_true across settled heights (ratios 1.01–1.04) — a
 *calibrated* sensor, not just a correlated one. Must be read at the height the
 flyer *settles* at (ground effect lifts it ~1 mm above the reference).
- **Floor rangefinder:** held-out floor-distance RMSE **0.03 mm**.
With roll+pitch (§2) and now vertical, all three components of the proximity vector
exist and are individually calibrated.
 
---
 
## 4. A3 — robust to geometry (`e20`)
 
"Flat infinite wall" was never load-bearing. Surfaces are represented by
signed-distance functions (each strip's distance is to the *nearest point* of the
shape: plane, cylinder-inside, cylinder-outside). Calibrated once on a flat wall,
the horizontal compass was tested on geometry it never saw:
 
| geometry | mean bearing error |
|---|---|
| flat wall (reference) | 9° |
| concave tunnel (inside a bore) | 8° |
| convex pillar / rubble chunk | 10° |
| tilted wall (35° lean) | 6° |
 
Essentially identical to flat — the residual is the §2 anisotropy, not the geometry.
The reason is the thesis: the body-integrated disturbance is dominated by the
nearest surface (~1/d²), so shape barely affects *direction*. (Horizontal plane
only here, to avoid the vertical sign-ambiguity of §6.)
 
---
 
## 5. A4 — robust to the magnitude we assumed (`e21`)
 
The surface effect is *added* as an (R/4d)² law of strength K_GE, not derived from
the flow. The natural objection: "you imposed it, so of course you sense it, and you
don't know the real K_GE." The defence is structural:
 
> bearing = atan2(δ_roll / S_roll, −δ_pitch / S_pitch), and K_GE scales **both**
> axes equally, so it cancels in the ratio.
 
Sweeping K_GE over a 4× range (0.5 → 2.0) with the calibration fixed at K_GE = 1:
 
| K_GE | S_roll | S_pitch | ratio | mean bearing err |
|---:|---:|---:|---:|---:|
| 0.5 | 657  | 59  | 11.2 | 6° |
| 1.0 | 1269 | 113 | 11.3 | 7° |
| 2.0 | 2369 | 209 | 11.3 | 7° |
 
Magnitude scales ~linearly; the anisotropy ratio and the bearing error are constant.
**Direction is invariant to the assumed magnitude.** Combined with §4, nothing the
navigation needs depends on the exact effect model — only its existence and its
~1/d² falloff, both physically generic. K_GE remains the single number to pin down
in a wind tunnel for sim-to-real, and it sets *range*, not *direction*.
 
---
 
## 6. Honest limitations
 
- **Anisotropy (11:1).** Fore/aft sensing is ~11× weaker than lateral and needs
 per-axis calibration; without it the weak pitch axis is swamped and the compass
 fails. Real effect, stated, with a design-lever silver lining.
- **Vertical sign-ambiguity.** A surface above and a surface below both increase
 lift, so the vertical axis gives *magnitude*, not floor-vs-ceiling. Disambiguation
 needs a second cue (known altitude, or a tilted-surface's lateral asymmetry). The
 3D-geometry test (§4) is therefore horizontal-plane only.
- **Settled-height dependence.** The vertical estimate is calibrated at the height
 the flyer settles at, not the commanded height.
- **Added, not derived.** Our quasi-steady aero has no wake; the surface effect is a
 literature-grounded perturbation. §4–§5 show the *conclusions* are model-magnitude
 and geometry robust, but magnitudes themselves are model-dependent — a *simulation*
 feasibility result whose K_GE is the wind-tunnel quantity.
---
 
## 7. What this enables
 
The proximity vector is the sensing primitive navigation needs: which way is open,
how close is the nearest surface. Arc B turns it into motion — add yaw actuation
(the flyer currently cannot turn), forward cruise, and a reactive layer that steers
the vector toward open space — first through a single bent corridor, then real
clutter (Arc C). Each component of the vector is already earned and bounded, so the
demonstration will rest on validated capability rather than a staged effect.
 
## Files
- `src/controller.py` — LQG + multi-axis disturbance observer (`dist_states`, `Rk`,
 `roll_dist`/`pitch_dist`/`floor_dist`).
- `src/ground_effect.py` — per-strip κ(d), multi-surface sum, signed-distance
 primitives (plane, general plane, `cyl_in`, `cyl_out`).
- `experiments/e18_proximity_compass.py`, `e19_vertical_sensing.py`,
 `e20_curved_geometry.py`, `e21_robustness.py`.
- `outputs/e18…e21*.{png,csv}`, `outputs/arcA_summary.png`.