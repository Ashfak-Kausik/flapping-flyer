# True-wall fix: results index

Branch `truewall-fix`, fix commit `9fbef46` ("fix: wing-wash aero senses true-geometry walls, not
noised feeler reconstruction"). The frozen pre-fix results (`v1.0-report`, `outputs/e51_baselines.csv`,
`outputs/e51b_ablation_followup.csv`) are untouched and serve as the "before" record.

Tables are numbered T1, T2, ... in the order they were produced.

---

## Table T1 - e51 primary ablation, ORIGINAL (noisy-wall) vs CORRECTED (true-wall)

**Sources**

- ORIGINAL: `outputs/e51_baselines.csv` (reached, detection, false positives) joined per course with
  `outputs/e51b_ablation_followup.csv`, rows `part == part1` (crashed flag, min clearance). The two
  files agree on the reached flag for all 80 FULL / NO_WINGWASH flights.
- CORRECTED: `outputs/e51_baselines_truewall.csv`, produced by `experiments/e51_truewall.py`.

**Setup (identical in both):** 40 primary-ensemble courses, generator seed 1234, flight seed = course
id, noise level 1.0, 1 kHz control, scoring by `e50.score_quiet` (60 mm match tolerance). The 40
courses contain 68 openings in total.

**Definitions**

- *reached*: flight ended at the finish.
- *strike-free*: reached the finish AND no wall strike anywhere in the run (`crashed == 0`).
- *cond*: scored on that variant's own reached courses. *end-to-end*: over all 40 courses / all 68 openings.
- Clearance statistics are over all 40 flights of the variant, including those that did not reach.
- False positives are counted on reached courses only (as in the original e51).

| configuration | reached n/40 | strike-free n/40 | per-passage (cond; end-to-end) | per-opening (cond; end-to-end) | mean min-clearance (mm) | min min-clearance (mm) | false positives |
|---|---|---|---|---|---|---|---|
| FULL (orig) | 34/40 | 30/40 | 28/34 (82%); 28/40 (70%) | 38/56 (68%); 38/68 (56%) | 18.2 | 1.0 | 1 |
| FULL (true) | 35/40 | 30/40 | 30/35 (86%); 30/40 (75%) | 43/58 (74%); 43/68 (63%) | 18.3 | 2.2 | 1 |
| NO_WINGWASH (orig) | 33/40 | 29/40 | 29/33 (88%); 29/40 (72%) | 42/56 (75%); 42/68 (62%) | 18.7 | 6.5 | 3 |
| NO_WINGWASH (true) | 32/40 | 29/40 | 28/32 (88%); 28/40 (70%) | 41/54 (76%); 41/68 (60%) | 17.4 | 1.6 | 1 |
| NO_FEELERS (true) | 16/40 | 15/40 | 0/16 (0%); 0/40 (0%) | 0/25 (0%); 0/68 (0%) | 15.4 | 6.9 | 0 |
| OPEN_LOOP (true) | 0/40 | 0/40 | n/a (0 reached); 0/40 (0%) | n/a (0 reached); 0/68 (0%) | 0.3 | 0.0 | 0 |

For reference, the original (noisy-wall) NO_FEELERS reached 12/40 and OPEN_LOOP 0/40; their crashed
flags and clearances were never persisted, so they have no full row.

### T1a - paired detection, FULL vs NO_WINGWASH on courses BOTH completed

| wall construction | paired courses | FULL per-passage | NO_WINGWASH per-passage | FULL per-opening | NO_WINGWASH per-opening | FULL FP | NO_WINGWASH FP |
|---|---|---|---|---|---|---|---|
| orig | 32 | 27/32 (84%) | 28/32 (88%) | 36/53 (68%) | 39/53 (74%) | 1 | 3 |
| true | 32 | 28/32 (88%) | 28/32 (88%) | 40/54 (74%) | 41/54 (76%) | 1 | 1 |

Per-course detection count, NO_WINGWASH minus FULL, on the paired set: orig 3 courses higher, 0 lower,
29 equal; true 1 higher, 0 lower, 31 equal.

### T1b - paired completion and clearance, FULL vs NO_WINGWASH, all 40 courses

| wall construction | reached by FULL only (course ids) | reached by NO_WINGWASH only | strike-free on paired set (FULL; NO_WINGWASH) | min-clearance difference FULL - NO_WINGWASH, mean (sd), mm | courses where FULL has more clearance |
|---|---|---|---|---|---|
| orig | 2 (0, 24) | 1 (32) | 29/32; 28/32 | -0.51 (3.14) | 18/40 |
| true | 3 (0, 23, 39) | 0 | 29/32; 29/32 | +0.88 (3.41) | 26/40 |

### T1c - how much the fix itself moved each variant (reached flag, orig -> true)

| variant | courses whose reached flag changed | ids |
|---|---|---|
| FULL | 3 of 40 | 23, 32, 36 |
| NO_WINGWASH | 3 of 40 | 24, 36, 39 |
| NO_FEELERS | 14 of 40 | 1, 4, 6, 7, 11, 16, 17, 20, 27, 28, 30, 34, 37, 38 |
| OPEN_LOOP | 0 of 40 | - |

NO_WINGWASH does not use the wing-wash signal for control, yet 3 of its outcomes changed. The wall
planes also drive the aerodynamic wall force on the vehicle, so the fix changes the simulated physics
for every variant, not only what the wing-wash channel senses.

---

## Verdict (Step 5)

**Question:** does the corrected true-wall construction change the central finding that wing-wash is
redundant to the feelers?

**Answer: (C) ambiguous / within noise.**

- **Detection: unchanged, wing-wash still adds nothing.** On the 32 courses both variants complete,
  FULL finds 40/54 openings and NO_WINGWASH 41/54 (28/32 passages each). Before the fix it was 36/53
  vs 39/53. Removing wing-wash never lowers a course's detection count in either run.
- **Completion and clearance: the direction moved toward wing-wash helping, but the size is inside
  the run-to-run scatter.** FULL vs NO_WINGWASH reached is now 35 vs 32 (was 34 vs 33), with all 3
  discordant courses favouring FULL (was 2 vs 1). Mean min-clearance difference flipped sign, from
  -0.5 mm to +0.9 mm in FULL's favour, with FULL ahead on 26/40 courses (was 18/40). But the fix alone
  flipped the reached flag on 3 courses for each of these two variants (T1c), which is as large as the
  FULL-vs-NO_WINGWASH gap, and the clearance shift is about 1.6 standard errors of the paired
  difference. Strike-free completion is 30 vs 29 in both runs.
- **False positives equalised** at 1 vs 1 (was 1 vs 3), which removes the one metric where FULL
  previously looked better.

So the fix does not produce a clear independent contribution from wing-wash at n = 40, and it does not
confirm redundancy as cleanly as the original run did either. The detection half of the finding holds;
the navigation half now leans slightly toward FULL without being distinguishable from noise. No formal
statistical test was run, per instructions.

One further observation, outside the FULL-vs-NO_WINGWASH question: wing-wash alone (NO_FEELERS)
reaches 16/40 under true walls against 12/40 before and 0/40 for OPEN_LOOP, so the channel carries
usable centring information on its own in both constructions.

---

## Table T2 - statistics protocol, FULL vs NO_WINGWASH on the corrected harness

### T2.0 - Pre-registered decision rule

*Written on 2026-10-05, before any `stats_pass1` or `stats_legsweep` flight was launched. The only
FULL / NO_WINGWASH true-wall data seen at this point is Table T1 (noise seed = course id), which is
not reused below.*

**Design.** FULL vs NO_WINGWASH on the primary 40-course ensemble (generator seed 1234, same
geometry for both variants), 5 noise seeds per course, noise seed matched across the two variants:
`noise_seed = 1000*j + course_id`, j = 1..5. 5 x 40 x 2 = 400 flights, 200 paired trials. Noise
level 1.0, 1 kHz, scoring unchanged. Runner: `experiments/e53_stats.py pass1`, output
`outputs/stats_pass1.csv`.

**Primary metric.** Strike-free completion (reached the goal AND no wall strike anywhere in the
run), paired by course + noise seed. Difference = FULL minus NO_WINGWASH, in percentage points,
over the 200 paired trials.

**Decision rule.** With the 95% confidence interval on that paired difference:

| 95% CI position | verdict |
|---|---|
| entirely within [-5, +5] pp | EQUIVALENT |
| entirely above +5 pp | WING-WASH HELPS |
| entirely below -5 pp | WING-WASH HURTS |
| straddles +5 or -5 pp | UNDERPOWERED / inconclusive at this sample size |

**Secondary metrics** (reported with CI, not used for the decision): per-opening detection on
trials both variants completed (pooled hits / pooled openings on those trials), and mean
min-clearance over all trials.

**Analysis, fixed in advance.**

- Confidence intervals: cluster bootstrap over the 40 courses (resample courses with replacement,
  each carrying all 5 of its paired per-seed outcomes), 10,000 resamples, percentile interval,
  `numpy.random.default_rng(20261005)`.
- Binary primary outcome: exact two-sided McNemar test on the 200 pooled paired trials. Trials within
  a course are not independent, so the bootstrap CI, not the McNemar p, drives the decision.
- Effect sizes: risk difference (pp) and discordant-pair counts with their ratio for strike-free
  completion; Cohen's d_z on the 40 per-course mean differences for min-clearance.
- Leg-sweep replication (Table T3): the same 100 e52 courses (2..6 legs, 20 per cell, width
  58-64 mm, generator seeds 5300-5304), fresh noise seed `7000 + course_id` (e52 used `course_id`),
  one flight per variant per course, strike-free completion, exact two-sided McNemar per cell and
  pooled. The pre-fix effect "reproduces" if the pooled difference is in the same direction (FULL
  higher) with pooled McNemar p < 0.05; otherwise it does not reproduce.

### T2.1 - Pass 1 results (Step 2)

**Source:** `outputs/stats_pass1.csv` (400 flights = 200 paired trials, 40 courses x 5 noise seeds),
analysed by `experiments/e53_analysis.py`. Cluster bootstrap over courses, 10,000 resamples,
percentile 95% interval. McNemar is exact, two-sided, on the 200 pooled paired trials. Run wall time
13,267 s (3 h 41 min) on 16 workers.

| metric | FULL | NO_WINGWASH | paired diff (FULL - NO_WINGWASH) | 95% CI | McNemar p | effect size | verdict vs +/-5 pp margin |
|---|---|---|---|---|---|---|---|
| **strike-free completion (primary)** | 152/200 (76.0%) | 150/200 (75.0%) | +1.0 pp | [-5.0, +6.5] pp | 0.83 | discordant pairs 12 vs 10 (ratio 1.20) | **UNDERPOWERED** (upper bound crosses +5) |
| reached goal (context only) | 165/200 (82.5%) | 167/200 (83.5%) | -1.0 pp | [-4.5, +2.5] pp | 0.75 | discordant pairs 4 vs 6 | not part of the rule |
| per-opening detection, 161 jointly-completed trials (secondary) | 200/267 (74.9%) | 197/267 (73.8%) | +1.1 pp | [-1.2, +3.5] pp | n/a | - | not part of the rule |
| mean min-clearance, all 200 trials (secondary) | 17.98 mm | 17.74 mm | +0.24 mm | [-0.57, +0.98] mm | n/a | Cohen's d_z = +0.09 over courses | not part of the rule |

Supporting detail for the primary metric:

- Per-course strike-free difference: FULL better on 7 courses, worse on 4, identical on 29.
- Per-seed strike-free counts out of 40 (FULL, NO_WINGWASH): (32, 32), (30, 29), (30, 30), (30, 32), (30, 27).
- Wall strikes anywhere in the run: FULL 38/200, NO_WINGWASH 37/200.

Not pre-registered, reported as an observation only: false positives on reached trials were 2 for
FULL (165 trials) and 12 for NO_WINGWASH (167 trials).

**Decision under the pre-registered rule: UNDERPOWERED / inconclusive at this sample size.** The
point estimate is +1.0 pp and there is no sign of a difference (12 vs 10 discordant pairs), but the
95% interval [-5.0, +6.5] pp is 11.5 pp wide and its upper end lies outside the +/-5 pp margin, so
equivalence is not established. The interval excludes any wing-wash benefit above about 6.5 pp and
any harm beyond about 5 pp on this ensemble.

Table T3 (leg-sweep replication) has not been run; it is on hold pending review of this result.

---

## Table T3 - leg-sweep replication under true walls and fresh noise seeds (Step 3)

**Source:** `outputs/stats_legsweep.csv` (200 flights = 100 paired trials), run by
`experiments/e53_stats.py legsweep`, analysed by `experiments/e53_analysis.py`. Same 100 courses as
the pre-fix e52 leg sweep (width 58-64 mm, generator seeds 5300-5304), noise seed `7000 + course_id`
(e52 used `course_id`). Pre-fix columns are from `outputs/e52_wingwash_scaling.csv`, rows
`factor == legs`. Metric: strike-free completion. McNemar is exact, two-sided, on paired courses.
Run wall time 11,562 s (3 h 13 min) on 16 workers.

| cell (legs) | FULL strike-free | NO_WINGWASH strike-free | diff | discordant (FULL-only / NO_WINGWASH-only) | McNemar p | pre-fix e52: FULL / NO_WINGWASH (diff, p) |
|---|---|---|---|---|---|---|
| 2 | 13/20 | 12/20 | +1 | 1 / 0 | 1.000 | 11 / 8 (+3, 0.375) |
| 3 | 10/20 | 9/20 | +1 | 3 / 2 | 1.000 | 11 / 8 (+3, 0.375) |
| 4 | 6/20 | 7/20 | -1 | 1 / 2 | 1.000 | 8 / 9 (-1, 1.000) |
| 5 | 12/20 | 10/20 | +2 | 5 / 3 | 0.727 | 13 / 5 (+8, 0.021) |
| 6 | 7/20 | 4/20 | +3 | 4 / 1 | 0.375 | 5 / 4 (+1, 1.000) |
| **pooled** | **48/100** | **42/100** | **+6** | **14 / 8** | **0.286** | **48 / 34 (+14, 0.0125)** |

Context: reached-goal counts were FULL 70/100 and NO_WINGWASH 73/100; wall strikes anywhere in the
run were FULL 46/100 and NO_WINGWASH 52/100.

**Reproduction verdict (pre-registered criterion: FULL higher AND pooled McNemar p < 0.05): does
NOT reproduce.** The pooled difference keeps its sign but shrinks from +14 to +6 and is not
significant (p = 0.29). The 5-leg cell that drove the original result went from 13 vs 5 to 12 vs 10;
FULL is unchanged and NO_WINGWASH recovered, which is what a seed-specific bad draw for NO_WINGWASH in
the original run would look like. With 22 discordant pairs the replication cannot exclude a real
effect of the original size (rough 95% interval on +6 is about -3 to +15), so this shows the
original p = 0.013 was not robust, not that the true effect is zero.

---

## Overall verdict - does wing-wash make a resolved difference under fair conditions?

**UNDERPOWERED.** Under true walls and matched fresh noise seeds, no difference between FULL and
NO_WINGWASH is resolved, and equivalence within +/-5 pp is not established either.

- Primary ensemble (T2.1): strike-free +1.0 pp, 95% CI [-5.0, +6.5] pp, McNemar p = 0.83. The upper
  bound crosses the +5 pp margin.
- Leg sweep (T3): +6 of 100, McNemar p = 0.29; the pre-fix 48-vs-34 effect did not reproduce.
- Secondary metrics show nothing: detection +1.1 pp [-1.2, +3.5], min-clearance +0.24 mm
  [-0.57, +0.98].

What the data do support: any wing-wash benefit on strike-free completion in the primary ensemble is
smaller than about 6.5 pp, and every point estimate collected so far on the corrected harness sits
at or slightly above zero in FULL's favour (+1.0 pp primary, +6 pp leg sweep) without reaching
significance. No expansion run has been launched.

---

## Table T4 - e54 degraded feeler, RANGE AXIS ONLY (partial run)

**Source:** `outputs/degraded_feeler.csv.partial.jsonl`, the checkpoint of
`experiments/e54_degraded_feeler.py` (code commit `b303ce0`). The sweep was cut off by a power loss
on 2026-10-07 after 306 of 960 flights, all on the range axis; the checkpoint's damaged tail (326 NUL
bytes after the last complete record) was truncated and the 306 records are intact. No final
`outputs/degraded_feeler.csv` exists. The dropout and noise axes have **not** been flown and are on
hold: with the feeler read every 1 kHz control tick and drops independent per tick, hold-last-value
dropout leaves a ray stale for only 5 ms on average even at p = 0.8, so the dropout model is under
review.

**Setup:** FULL vs NO_WINGWASH on the corrected (true-wall) harness, primary 40-course ensemble,
noise seed `1000 + course_id` matched across variants and levels (= pass 1 seed index 1), noise
level 1.0, 1 kHz, no gains retuned. `feeler_range = X` makes any feeler reading beyond X m return
clear; the idealized antenna reach is 0.16 m. Only the navigation feelers are degraded; the true
ray-cast feeding the wing-wash wall planes is untouched. The idealized row is the seed-index-1 slice
of `outputs/stats_pass1.csv`, not re-flown.

**Definitions:** *strike-free* = reached the finish AND no wall strike anywhere in the run.
*crashed* = wall strike anywhere in the run (a crashed flight may still reach the finish). Paired by
course. CI is the T2 cluster bootstrap over courses (10,000 resamples, percentile,
`default_rng(20261005)`); McNemar is exact, two-sided. This sweep was not pre-registered and the
four levels are not corrected for multiplicity.

| feeler range (m) | paired courses | FULL strike-free | NO_WINGWASH strike-free | paired diff (FULL - NO_WINGWASH) | 95% CI (pp) | discordant (FULL-only / NO_WINGWASH-only), McNemar p | FULL crashed | NO_WINGWASH crashed |
|---|---|---|---|---|---|---|---|---|
| idealized (0.16, pass 1 seed 1) | 40 | 32/40 | 32/40 | 0.0 pp | [-10.0, +10.0] | 2 / 2, p = 1.000 | 6/40 | 5/40 |
| 0.12 | 40 | 30/40 | 31/40 | -2.5 pp | [-12.5, +7.5] | 2 / 3, p = 1.000 | 6/40 | 4/40 |
| 0.08 | 40 | 31/40 | 31/40 | 0.0 pp | [-12.5, +12.5] | 3 / 3, p = 1.000 | 5/40 | 6/40 |
| 0.05 | 40 | 25/40 | 18/40 | +17.5 pp | [0.0, +35.0] | 11 / 4, p = 0.118 | 14/40 | 17/40 |
| 0.03 (incomplete) | 32 | 0/32 | 1/32 | -3.1 pp | [-9.4, 0.0] | 0 / 1, p = 1.000 | 32/32 | 31/32 |

The 0.03 cell has 66 of 80 flights. Courses 29, 31, 32, 33, 36, 37, 38 and 39 lack one or both
variants and are excluded from the paired row. Counting every 0.03 flight that finished: FULL 0/34
strike-free with 34/34 crashed, NO_WINGWASH 1/32 strike-free with 31/32 crashed.

Context, reached-goal counts (FULL / NO_WINGWASH): idealized 32 / 34, 0.12 31 / 33, 0.08 33 / 33,
0.05 31 / 27, 0.03 12 / 12 of 32.

**Range-axis verdict: NO - a wing-wash rescue of a range-limited feeler is not resolved; at 0.03 m
it is BOTH-FAIL-TOGETHER.**

- 0.12 and 0.08 m: the feeler is not meaningfully degraded (30-31/40 against 32/40 idealized) and
  the variants are indistinguishable, so there is nothing to rescue.
- 0.05 m: the only level where the variants separate. FULL 25/40 vs NO_WINGWASH 18/40, +17.5 pp,
  11 vs 4 discordant courses. The direction is what a rescue would look like, but the CI lower bound
  sits at 0.0 and McNemar p = 0.118, at one of four levels examined, so it is suggestive and not
  established. FULL still loses 7 courses relative to idealized, so at most a partial rescue.
- 0.03 m: both variants strike a wall in essentially every flight (FULL 0/32, NO_WINGWASH 1/32
  strike-free). The 8 unpaired courses cannot change this: FULL could reach at most 8/40.

---

## Closing summary - corrected-harness verdict across T1-T4

**Under the corrected true-geometry harness, wing-wash is statistically indistinguishable from
no-wing-wash on the primary task.** No table resolves a difference between FULL and NO_WINGWASH.

| table | what it tested | FULL vs NO_WINGWASH | outcome |
|---|---|---|---|
| T1 | e51 primary ablation, true walls, 40 courses, 1 seed | strike-free 30/40 vs 29/40; openings found 43/68 vs 41/68 | within noise |
| T2 | paired stats, 40 courses x 5 matched seeds (pre-registered) | strike-free 152/200 vs 150/200, +1.0 pp, 95% CI [-5.0, +6.5] pp, McNemar p = 0.83 | no difference resolved |
| T3 | leg-sweep replication, 100 courses, fresh seeds (pre-registered) | strike-free 48/100 vs 42/100, +6, McNemar p = 0.29 (pre-fix: 48 vs 34, p = 0.0125) | did not reproduce |
| T4 | feeler range limited to 0.12 / 0.08 / 0.05 / 0.03 m, 1 seed (not pre-registered) | 30 vs 31, 31 vs 31, 25 vs 18 (p = 0.118), 0 vs 1 of 32 | no rescue resolved |

- **Primary task (T1, T2).** Strike-free completion differs by +1.0 pp over 200 paired trials with
  12 vs 10 discordant pairs; detection and min-clearance show nothing either. Indistinguishable is
  the accurate word, not equivalent: the 95% interval reaches +6.5 pp, so equivalence within the
  pre-registered +/-5 pp margin was not established, and a benefit of up to about 6.5 pp is not
  excluded.
- **The one apparent positive was a seed artifact (T3).** The pre-fix leg-sweep effect (48 vs 34,
  p = 0.0125) did not survive true walls and fresh noise seeds: 48 vs 42, p = 0.29. The 5-leg cell
  that drove it went from 13 vs 5 to 12 vs 10, with FULL unchanged and NO_WINGWASH recovering, which
  is the signature of a bad seed draw for NO_WINGWASH in the original run. The replication is too
  small to exclude a real effect of the original size, but nothing now supports one.
- **Degrading the feeler does not produce a resolved wing-wash rescue (T4).** At 0.12 and 0.08 m the
  feeler is barely affected and the variants match. At 0.03 m both fail together, striking a wall in
  essentially every flight; that cell is incomplete (32 paired courses, 66 of 80 flights), but the 8
  missing courses cannot change the outcome. The 0.05 m cell (25/40 vs 18/40, +17.5 pp, CI [0.0, +35.0] pp,
  p = 0.118) is a single-seed hint only: one level of four, not pre-registered, not corrected for
  multiplicity, and not replicated.
- **Headline mission figures (FULL, true walls, T1) and how to read them.** Reached 35/40,
  strike-free 30/40, detection 43/58, localization about 8 mm, false positives about 0.03 per
  course. These come from the single-seed T1 run (noise seed = course id), not from the T2
  statistics run. Three of them are conditional on the flight reaching the finish:
  - Detection 43/58 (74%) counts openings only on the 35 courses FULL completed. End-to-end, over
    all 40 courses, it is 43/68 (63%).
  - False positives: 1 in 35 reached courses = 0.029 per reached course (0.025 over all 40).
  - Localization 7.9 mm is the mean over the 43 detected openings, with sd 7.2 mm and median
    6.9 mm. Missed openings contribute no error.
- **Not run.** The dropout and noise axes of e54 were never flown. The dropout model is too weak to
  be informative as written: with the feeler refreshed at 1 kHz and drops independent per tick,
  hold-last-value dropout leaves a ray stale for about 5 ms on average even at p = 0.8. The 0.03 m
  range cell is also incomplete (66 of 80 flights). The checkpoint remains resumable.
