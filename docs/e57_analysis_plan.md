# e57 analysis plan: corridor-width sweep on the corrected (true-wall) harness

Written 2026-10-12, before any e57 flight was started, and committed before launch. The file's
SHA-256 is also printed at the top of `outputs/e57_runs.log` when the run launches.

All flights use the corrected harness as it stands at the commit that adds this file, nominal noise
(level 1.0), 1 kHz control, K_GE = 1.0, undegraded feelers. Controller, generator and scoring are
unchanged. Every flight goes through `e51._run_one_course` (e48.run + e50.score_quiet).

## Question

In the primary ensemble (corridors 64 to 76 mm wide) the walls sit 32 to 38 mm from a centred
vehicle, at the weak end of the wing-wash band. Does the aerodynamic channel contribute to
strike-free completion when the corridor is narrower and the walls are closer?

The only earlier width sweep (e52) was flown on the uncorrected harness and is not used as evidence.

## Design

- Five width bands, as in e52: 70-76, 64-70, 58-64, 52-58 and 46-52 mm; three legs per course.
- 40 courses per band from `e52.generate_courses_param`, generator seed 5200 + band index. The first
  20 of each band are the e52 courses; the last 20 are new.
- FULL vs NO_WINGWASH, one flight per variant per course, matched noise seed
  `8000 + 100*band_index + course_id` (not used by any earlier run).
- 400 flights, 200 paired trials.

## Analysis

Outcome: strike-free completion (reached the goal with no wingtip strike, as scored by the harness).
Intervals: 95% cluster bootstrap over courses, 10,000 resamples, percentile method,
`default_rng(20261005)`. Test: exact two-sided McNemar on discordant pairs.

**Primary (H1): narrow corridors.** Paired difference FULL minus NO_WINGWASH pooled over the two
narrowest bands (52-58 and 46-52 mm; 80 pairs). This is the one confirmatory comparison.

| 95% CI on the paired difference | verdict |
|---|---|
| lower bound above 0 | ADVANTAGE CONFIRMED (in narrow corridors) |
| entirely within [-5, +5] pp | NO MEANINGFUL ADVANTAGE |
| upper bound below 0 | WING-WASH HURTS |
| anything else | INCONCLUSIVE |

With 80 pairs the interval will be roughly +/- 10 to 12 pp wide, so only a large effect can be
confirmed and a null result will most likely read INCONCLUSIVE, not equivalent.

**Secondary (descriptive, no multiplicity correction, no confirmatory claim):**
- the paired difference in each of the five bands (40 pairs each), with interval and McNemar p;
- the difference pooled over all 200 pairs;
- goal-reaching, end-to-end per-opening detection and mean minimum clearance, per band and pooled.

## Conduct

No interim analysis and no early stopping. All generated courses are analysed. A flight that errors
is re-flown with the same seed; a pair is excluded only if a flight cannot be completed after
re-running, and any exclusion is reported. The strike-radius sensitivity (13.6 mm, no goal-zone
exception) is reported alongside the primary result, as for the earlier comparisons.
