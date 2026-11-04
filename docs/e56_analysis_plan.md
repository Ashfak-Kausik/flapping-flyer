# e56 analysis plan: powered degraded-feeler confirmation and centring-gain sweep

Written 2026-10-11, before any e56 flight was started. The file's SHA-256 is printed at the top of
`outputs/e56_runs.log` when the run launches.

All flights use the corrected (true-wall) harness as it stands at commit 1b81474, nominal noise
(level 1.0), 1 kHz control, K_GE = 1.0. Controller, generator distributions and scoring are unchanged.
Every flight goes through `e51._run_one_course` (e48.run + e50.score_quiet).

## Part A: confirmatory test at a feeler range limit of 0.05 m

**Origin of the hypothesis.** In e54 (40 primary courses, one noise seed) the only cell with an
apparent wing-wash advantage was feeler range 0.05 m: strike-free 25/40 (FULL) vs 18/40 (NO_WINGWASH),
discordant pairs 11 vs 4, exact McNemar p = 0.118. That cell was one of several and was not
pre-specified, so it is treated as hypothesis-generating only.

**Hypothesis H1.** With the feeler range limited to 0.05 m, strike-free completion is higher for FULL
than for NO_WINGWASH.

**Design.**
- 200 new courses from the primary-ensemble generator (`e50.sample_course(rng, hard=False)`, same
  rejection rule as `e51.generate_courses`), generator seed 5600. None of the 40 courses that
  produced the hypothesis is reused.
- Each course flown once per variant with a matched noise seed, `noise_seed = 9000 + course_id`.
- Feeler degradation `feeler_range = 0.05`, applied as in e54 (navigation readings only; the
  true-wall ray-cast feeding the aerodynamics is untouched).
- 400 flights.

**Sample size.** At the e54 discordance rate (about 37.5%), 200 pairs give roughly 80% power
(two-sided alpha 0.05) to detect a true difference of 12 percentage points, and over 95% power for
the 17.5 points observed in e54.

**Primary outcome and analysis.** Paired difference FULL minus NO_WINGWASH in strike-free completion
(reached the goal with no wingtip strike), with
- 95% cluster-bootstrap confidence interval over courses, 10,000 resamples, percentile method,
  `default_rng(20261005)`;
- exact two-sided McNemar test on the discordant pairs.

**Decision rule.**
| 95% CI on the paired difference | verdict |
|---|---|
| lower bound above 0 | ADVANTAGE CONFIRMED |
| entirely within [-5, +5] pp | NO MEANINGFUL ADVANTAGE (equivalent within the margin) |
| upper bound below 0 | WING-WASH HURTS |
| anything else | INCONCLUSIVE |

The bootstrap interval drives the verdict; the McNemar p-value is reported alongside it.

**Secondary outcomes** (reported with intervals, not part of the rule): goal-reaching, end-to-end
per-opening detection, mean minimum clearance.

**Conduct.** No interim analysis and no early stopping. All generated courses are analysed. A flight
that errors is re-flown with the same seed; a pair is excluded only if a flight cannot be completed
after re-running, and any exclusion is reported.

## Part B: centring-gain sweep (exploratory)

**Question.** Is the absence of a detectable wing-wash contribution at the deployed centring gain
(Kc = 1.5e-5 s^2) a consequence of that gain being too small?

**Design.**
- The 40 primary courses (`e51.generate_courses`, seed 1234), undegraded feelers, FULL configuration
  with `Kc` multiplied by 2, 4 and 8. The roll-reference saturation (2.5 deg) and all other gains are
  unchanged.
- Noise seeds `1000*j + course_id` for j = 1, 2, 3, i.e. the seeds of pass 1 of e53, so the 1x (FULL)
  and 0x (NO_WINGWASH) points are the existing rows of `outputs/stats_pass1.csv` and are not re-flown.
- 360 new flights, 120 paired trials per gain.

**Analysis.** For each gain, strike-free completion and its paired difference against 0x
(NO_WINGWASH), with the same cluster-bootstrap interval over courses, plus mean minimum clearance.
The sweep is descriptive: three gains are compared without multiplicity correction, and no
confirmatory claim will be made from it. A gain that looks better than nominal would be a lead for a
separate confirmatory run on new courses.

## Harness check

One extra flight repeats an existing pass-1 flight (course 16, seed index 1, FULL, gain 1x) through
the e56 runner. Its outcome must match the stored row; this checks that the gain override and the
runner leave the nominal flight unchanged.
