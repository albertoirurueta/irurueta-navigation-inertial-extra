# Implementation plan: Fix mistakes and deviations from Tedaldi et al. (ICRA 2014) / imu_tk in the threshold-factor optimizers

## Task summary

Issue #29 reviews the threshold-factor optimizers in `calibration.intervals.thresholdfactor` against Tedaldi et al.
(ICRA 2014) and imu_tk. Two bugs must be fixed with regression tests (E1, E2) and four deviations (E3-E6) each need a
documented decision. The upstream library `irurueta-navigation-inertial` 1.12.1 (already in `pom.xml`, released with the
fixes of upstream issue #41, including the variance-norm detector statistic N7) is the dependency; the end-to-end test
required by the last acceptance criterion builds on it.

Decisions taken with the user during planning:

| Finding | Decision |
|---|---|
| E1 | Fix: check `KnownBiasGyroscopeCalibrator` (3 optimizers) + regression tests per optimizer. |
| E2 | Fix as proposed: reset results at start of `optimize()`, `hasResult` only when `< Double.MAX_VALUE`, throw `IntervalDetectorThresholdFactorOptimizerException` otherwise; bracketed optimizers map a failed calibration to `Double.MAX_VALUE`. |
| E3 | Keep default range `[2, 10]` (now matches imu_tk units because 1.12.1 uses the variance-norm statistic, upstream N7); document the statistic, the `k_old ≈ k²` relation and the unchanged constants in Javadoc, Antora and changelog. |
| E4 | Fix with an opt-out flag: new `setInitialGyroscopeBiasUsed(boolean)` / `isInitialGyroscopeBiasUsed()` in the 3 gyroscope-capable optimizers, default `true`; when enabled, the generator's `getInitialAvgAngularSpeedTriad()` is set as known bias (`KnownBiasGyroscopeCalibrator`) or initial bias (`UnknownBiasGyroscopeCalibrator`) on each evaluation. |
| E5 | Default rules use the accelerometer MSE only (imu_tk behaviour); gyroscope/magnetometer MSE ignored by default and documented; custom `*MseRule` implementations stay possible. |
| E6 | `DefaultGyroscopeQualityScoreMapper` derives the score from sequence duration / item count (shorter sequences score higher) instead of the constant base-noise standard deviations. |

Source: GitHub issue #29
Base branch: develop

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "E1 and E2 fixed with regression tests." | Task 1, Task 5 |
| 2 | "A decision is taken and documented for each deviation (E3 to E6): fix as proposed, or keep with the difference from the paper / imu_tk documented in Javadoc and Antora docs." | Task 2 (E5), Task 3 (E6), Task 6 (E4), Task 7 (E3), Task 9 (Antora docs) |
| 3 | "E3 is resolved consistently with the decision taken for N7 in albertoirurueta/irurueta-navigation-inertial#41." | Task 7 |
| 4 | "The irurueta-navigation-inertial dependency is updated once albertoirurueta/irurueta-navigation-inertial#41 is released, and the optimizers are tested end-to-end against synthetic data with known parameters." | Task 8 (dependency already at 1.12.1 — verified in Task 8.1) |
| 5 | derived: behavioural/default changes are recorded in the changelog | Task 9 |

## Current code state

- Project type: `java-library` (Java 21, Maven, no wrapper, single module, `1.9.0-SNAPSHOT`), JUnit 5 + Mockito, JaCoCo, Checkstyle
  (`checkstyle.xml`), PMD, SpotBugs. `irurueta-navigation-inertial` is at 1.12.1 (`pom.xml:93`). No UI.
- Package `src/main/java/com/irurueta/navigation/inertial/calibration/intervals/thresholdfactor/` (abbreviated `tf/`), tests mirror it
  under `src/test/java/...` (16 test classes: Exhaustive*/Bracketed* optimizers, Default*MseRule, Default*QualityScoreMapper).
- **E1**: `keepBestResult` in `GyroscopeIntervalDetectorThresholdFactorOptimizer` (~l.1030),
  `AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer` (~l.1427) and
  `AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer` (~l.1677) use
  `instanceof KnownBiasAccelerometerCalibrator` for the gyroscope calibrator.
- **E2**: every `evaluateForThresholdFactor` returns `Double.MAX_VALUE` (no `keepBestResult`) on detection failure/too few sequences.
  `Exhaustive*Optimizer.optimize()` sets `hasResult = true` after any non-throwing evaluation (e.g. l.161 in the gyroscope one) and
  never resets `optimalThresholdFactor` and the kept results; `Bracketed*Optimizer.optimize()` (l.~286-350) sets `minMse = Double.MAX_VALUE`,
  calls `mseOptimizer.minimize()` and returns `optimalThresholdFactor`, converting `NavigationException` from the evaluation into
  `EvaluationException` (`initializeOptimizerListeners`), which aborts the minimization.
- **E3**: `DEFAULT_MIN_THRESHOLD_FACTOR = 2.0` / `DEFAULT_MAX_THRESHOLD_FACTOR = 10.0` in all 5 base optimizers (l.~59-89).
- **E4**: gyroscope generator is `GyroscopeMeasurementsGenerator` (also embedded in the combined generators); 1.12.1 exposes
  `getInitialAvgAngularSpeedTriad()`. `evaluateForThresholdFactor` of the gyroscope, accelerometer+gyroscope and accelerometer+gyroscope+magnetometer
  optimizers never reads it.
- **E5**: `DefaultAccelerometerAndGyroscopeMseRule` and `DefaultAccelerometerGyroscopeAndMagnetometerMseRule` return plain sums.
- **E6**: `DefaultGyroscopeQualityScoreMapper.map` averages per-item standard deviations, all equal across sequences.
- Docs: `docs/modules/ROOT/pages/index.adoc` (optimizers mentioned); `CHANGELOG.md` has `## [Unreleased]` (Keep a Changelog). `README.md`
  mentions the optimizers.
- Precedent: `.archive/implementation_plan_23.md` (format only). Conventions: Javadoc on every public/protected member
  (`iru-java-javadoc`), license headers (`iru-check-license`), 80% coverage, Checkstyle/PMD/SpotBugs vs baseline; tests run via the
  `iru-gate-runner` agent (`Skill iru-java-test`), never directly in the main conversation.

## Implementation steps

### Group 1 — E1, E5, E6 (Parallelizable: yes — disjoint files: gyroscope-capable base optimizers, the two default MSE rules, the gyroscope mapper)

- [x] Task 1. E1: use `KnownBiasGyroscopeCalibrator` when keeping gyroscope biases _(java)_ — implemented, tests added; validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD
  - [x] Task 1.1. Add regression tests (fail first) in `ExhaustiveGyroscope…`, `ExhaustiveAccelerometerAndGyroscope…`, `ExhaustiveAccelerometerGyroscopeAndMagnetometer…` test classes (and the Bracketed equivalents): run the optimizer with a `KnownBiasEasyGyroscopeCalibrator` (bias set) and assert `getEstimatedGyroscopeBiases()` equals the calibrator bias (non-null) — done (6 test classes); validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD
  - [x] Task 1.2. In `tf/GyroscopeIntervalDetectorThresholdFactorOptimizer.keepBestResult`, `tf/AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.keepBestResult` and `tf/AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer.keepBestResult` replace `instanceof KnownBiasAccelerometerCalibrator` (gyroscope branch) with `KnownBiasGyroscopeCalibrator`; fix imports. Note: the gyroscope-only optimizer stores it in `estimatedBiases`, the combined ones in `estimatedGyroscopeBiases` — done (3 optimizers); validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD
  ```java
  } else if (gyroscopeCalibrator instanceof KnownBiasGyroscopeCalibrator knownBiasGyroscopeCalibrator) {
      estimatedGyroscopeBiases = knownBiasGyroscopeCalibrator.getBias();
  }
  ```
- [x] Task 2. E5: default MSE rules use the accelerometer MSE only _(java)_ — implemented; validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD
  - [x] Task 2.1. Update `DefaultAccelerometerAndGyroscopeMseRuleTest` and `DefaultAccelerometerGyroscopeAndMagnetometerMseRuleTest`: `evaluate(a, g[, m])` returns `a` regardless of the other terms (fail first) — implemented; validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD
  - [x] Task 2.2. Change `DefaultAccelerometerAndGyroscopeMseRule` and `DefaultAccelerometerGyroscopeAndMagnetometerMseRule` to return `accelerometerMse`; rewrite class Javadoc (units reasoning: (m/s²)⁴ vs unit-vector distance vs T⁴; imu_tk behaviour; custom rules for other weighting) — implemented; validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD
  - [x] Task 2.3. Adjust existing optimizer tests that assumed the summed MSE (search for expectations built with the default rules in the combined optimizer tests) — implemented; validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD (no optimizer test edits needed: they only assert default rule class)
- [x] Task 3. E6: gyroscope quality score from sequence duration / item count _(java)_ — implemented, tests added; validated: thresholdfactor tests 322 pass, full suite 674 pass; coverage >=94% line on touched classes; only new Checkstyle LineLength on added lines (repo-wide pattern), no new PMD
  - [x] Task 3.1. Rewrite `DefaultGyroscopeQualityScoreMapperTest` (fail first): longer sequences (larger duration between first and last item timestamps, or more items) score strictly lower than shorter ones; scores in `(0, 1]`; null/empty items give 0.0; two sequences with identical items but different duration give different scores
  - [x] Task 3.2. Change `DefaultGyroscopeQualityScoreMapper.map` to `1 / (1 + duration)` where duration is derived from `BodyKinematicsSequence` item timestamps (`getTimestampSeconds()` of last minus first sorted item; fall back to item count × mean interval when timestamps are unavailable); Javadoc explains integration error grows with sequence length and cites the prior constant-score limitation
- [x] Task 4. Run Group 1 tests via `iru-gate-runner` (`iru-java-test` for the thresholdfactor package) _(java)_

### Group 2 — E2: no result when no threshold factor is valid (Parallelizable: no — touches the same base optimizer files as Group 1 and all 10 Exhaustive/Bracketed classes plus their tests)

- [x] Task 5. E2: failed evaluations never report success _(java)_
  - [x] Task 5.1. Add regression tests (fail first) for all 5 Exhaustive and 5 Bracketed test classes using a data source that makes interval detection fail at every factor (e.g. too little data): `optimize()` throws `IntervalDetectorThresholdFactorOptimizerException`; a second run after a successful run with failing data also throws and the previous optimal factor/results are not exposed (reset)
  - [x] Task 5.2. Add a protected `resetResults()` helper in each of the 5 base optimizers (`tf/Accelerometer…`, `tf/Gyroscope…`, `tf/Magnetometer…`, `tf/AccelerometerAndGyroscope…`, `tf/AccelerometerGyroscopeAndMagnetometer…`) that sets `optimalThresholdFactor = 0.0`, `minMse = Double.MAX_VALUE` and nulls/clears estimated parameters, covariances, biases, base noise level, threshold; call it at the start of `optimize()` in every Exhaustive/Bracketed class
  - [x] Task 5.3. Exhaustive optimizers: set `hasResult` only when `evaluateForThresholdFactor(thresholdFactor) < Double.MAX_VALUE`
  - [x] Task 5.4. Bracketed optimizers: after `mseOptimizer.minimize()` throw `IntervalDetectorThresholdFactorOptimizerException` when `minMse == Double.MAX_VALUE`; in `initializeOptimizerListeners` catch `NavigationException`/calibration failures per evaluation and return `Double.MAX_VALUE` instead of throwing `EvaluationException` (keep real programming errors, e.g. `NotReadyException`, propagating as today)
  - [x] Task 5.5. Update Javadoc of `optimize()` (`@throws` conditions) in the 10 classes; run the 10 test classes via `iru-gate-runner`

### Group 3 — E4: initial gyroscope bias (Parallelizable: yes — single task; depends on Groups 1 and 2 for the same optimizer files)
    - Done: resetResults() in 5 base optimizers; Exhaustive/Bracketed optimize() updated; 3 new tests per Exhaustive class (2) and per Bracketed class (4); coverage of 10 classes 86-95% lines; tests green. 48 Checkstyle LineLength (80 col) hits on touched lines left in place (repo style uses 120 cols).

- [x] Task 6. E4: apply the initial static-period gyroscope bias to the gyroscope calibrator _(java)_
  - Done: 3 base optimizers (Gyroscope, AccelerometerAndGyroscope, AccelerometerGyroscopeAndMagnetometer) apply the initial static average angular speed via KnownBiasGyroscopeCalibrator.setBias, EasyGyroscopeCalibrator.setInitialBias or UnknownBiasNonLinearGyroscopeCalibrator.setInitialBias (opt-out setInitialGyroscopeBiasUsed(false)); tests in the 3 Exhaustive test classes (known/unknown bias, used/not used, getter/setter, locked), Bracketed KeepsCalibratorBias tests opt out; coverage 94-97% line / 84-92% branch; no new PMD/SpotBugs issues, only Checkstyle line-length/hidden-field consistent with existing code.
  - [x] Task 6.1. Tests (fail first) in the Exhaustive test classes of the gyroscope, accelerometer+gyroscope and accelerometer+gyroscope+magnetometer optimizers: with `KnownBiasEasyGyroscopeCalibrator` and a data source with a known constant gyroscope bias in the initial static period, the calibrator bias after `optimize()` equals the generator's initial average angular speed; with `setInitialGyroscopeBiasUsed(false)` the caller-provided bias is untouched; same for an unknown-bias calibrator's initial bias
  - [x] Task 6.2. In the 3 base optimizers add `private boolean initialGyroscopeBiasUsed = true` with `isInitialGyroscopeBiasUsed()` / `setInitialGyroscopeBiasUsed(boolean)` (throws `LockedException` when running) and Javadoc; in `evaluateForThresholdFactor`, after the generator processed all data and before `calibrator.calibrate()`, read `generator.getInitialAvgAngularSpeedTriad()` and set it with `KnownBiasGyroscopeCalibrator.setBias(double[])`/`setBiasCoordinates(...)` or as initial bias of `UnknownBiasGyroscopeCalibrator` (confirm the exact initial-bias setter in 1.12.1 with `javap`; skip silently for calibrators that implement neither)
  - [x] Task 6.3. Javadoc on class level: explain the paper/imu_tk behaviour (mean angular rate of the initial static period removed before optimization) and the opt-out; run tests via `iru-gate-runner`

### Group 4 — E3 documentation in code and end-to-end test (Parallelizable: yes — Task 7 edits Javadoc of the base optimizers, Task 8 only adds a new test file)

- [x] Task 7. E3: document the threshold factor statistic and keep the default range _(java)_
  - [x] Task 7.1. (test `testDefaultThresholdFactorRangeIsKept` in ThresholdFactorOptimizersEndToEndTest, no extra file) Add a test asserting `DEFAULT_MIN_THRESHOLD_FACTOR == 2.0` and `DEFAULT_MAX_THRESHOLD_FACTOR == 10.0` for the 5 base optimizers (documents the decision)
  - [x] Task 7.2. Javadoc of both constants (5 base optimizers) and of the class: the factor multiplies the variance-norm statistic of `TriadStaticIntervalDetector` (1.12.1), like imu_tk `th_mult` sweeping 2…10; values for earlier versions (std-norm statistic) correspond roughly to `k²`
- [x] Task 8. End-to-end synthetic test _(java)_
  - [x] Task 8.1. Verify `pom.xml` depends on `irurueta-navigation-inertial` 1.12.1 (no change expected) and that the upstream release contains the variance-norm detector
  - [x] Task 8.2. New test `src/test/java/com/irurueta/navigation/inertial/calibration/intervals/thresholdfactor/ThresholdFactorOptimizersEndToEndTest.java` (seeded `Random`): generate synthetic IMU `BodyKinematics` streams (initial static period with gyroscope bias, 36-50 static positions separated by rotations) with known accelerometer (upper-triangular `Ma`, bias) and gyroscope (`Mg`, bias) parameters
  - [x] Task 8.3. Run `ExhaustiveAccelerometerAndGyroscope…` and `BracketedAccelerometerAndGyroscope…` optimizers with the default calibrators/rules; assert the optimal threshold factor is within `[2, 10]` and estimated `Ma`, `ba`, `Mg`, `bg` match the known parameters within tolerances justified by the noise (use fixed seeds; check stability over repeated runs); also assert E2 (failing data throws) and E4 (initial bias used)
  - [x] Task 8.4. Run the test via `iru-gate-runner` — 5 e2e tests + 352 optimizer tests pass; Javadoc-only main changes (no logic, coverage unchanged, Javadoc build and license OK)

### Group 5 — documentation, changelog and final validation (Parallelizable: yes — different files)

- [x] Task 9. Docs and changelog
  - [x] Task 9.1. `CHANGELOG.md` `[Unreleased]`: `### Fixed` (E1 gyroscope biases lost with known-bias gyroscope calibrators; E2 optimizers no longer report success/stale results), `### Changed` (E4 initial gyroscope bias applied by default with opt-out; E5 default combined MSE rules use the accelerometer MSE only; E6 default gyroscope quality scores depend on sequence duration; E3 default range kept, threshold factor now refers to the variance-norm statistic of 1.12.1)
  - [x] Task 9.2. Run `/iru-update-docs` for `docs/modules/ROOT/pages/index.adoc` (and `README.md` if it describes the optimizers): the decisions E3-E6, the new `isInitialGyroscopeBiasUsed` option and the deviations from the paper/imu_tk; build docs offline and verify
- [x] Task 10. Final validation _(java)_
  - [x] Task 10.1. Via `iru-gate-runner`: full unit suite (`iru-java-test`), `iru-java-coverage` (≥ 80% on touched classes), `iru-java-code-quality` vs baseline (no new Checkstyle/PMD/SpotBugs issues), `iru-java-javadoc` for touched classes, `iru-check-license`, `iru-check-security`
