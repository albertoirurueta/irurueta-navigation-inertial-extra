# Implementation plan: Fix bracketed optimizer range, RandomWalkEstimator statistics and KalmanDriftEstimator filtering (second calibration review)

## Task summary

Issue #31 is a second review of the algorithms of this library. It lists eight findings (F1-F8) that are not part of #29
and asks to verify the status of E1-E3 of #29. Five `Bracketed*` and five `Exhaustive*` threshold-factor optimizers
(`calibration.intervals.thresholdfactor`) and `RandomWalkEstimator` / `KalmanDriftEstimator` (`calibration`) are affected.
Dependency `irurueta-navigation-inertial` is already at 1.12.1.

Decisions taken with the user during planning:

| Finding | Decision |
|---|---|
| F1 | Fix: the bracket is set from the *current* threshold-factor range at the start of every `optimize()` (and `setMseOptimizer` keeps setting it). Tests assert the bracket after `setThresholdFactorRange` and that the optimum lies inside a non-default range. |
| F2 | Fix: close a drift period when `(numberOfSamples + 1) % driftPeriodSamples == 0`; update tests that rely on the current number of periods. |
| F3 | Fix with Welford-style recursion (difference against previous mean times difference against new mean). **Population variance** (divide by `n`) is kept and documented. Test compares against a batch computation. |
| F4 | Fix: `reset()` also calls `driftEstimator.reset()`; test resets in the middle of a drift period. |
| F5 | **Feed fixes to the filter**: the body is static at the reference frame, so the reference position and zero velocity are provided with `kalmanEstimator.updatePositionAndVelocity(...)` every epoch interval, so a Kalman epoch really runs; every sample is integrated over one time interval (initial `update(null, 0.0)` then timestamps `(n + 1) * timeInterval`). Javadoc/docs updated; test compares with `DriftEstimator` sample by sample. |
| F6 | Fix: iterate with an integer index (`min + i * step`, `i <= round((max - min) / step)`); progress derived from the same count. |
| F7 | Accepted code changes: replace `Double.MAX_VALUE` by a large finite penalty inside the Brent objective, start the bracket at the centre of the range; limitation documented in Javadoc and Antora, recommending the exhaustive optimizers as default. **No** coarse-to-fine seeding. |
| F8 | Fix the Javadoc copy-paste mistakes. |
| E1, E2 (#29) | Already fixed on `develop` by PR #32; verify regression tests exist and add any missing. |
| E3 (#29) | Confirm resolved (variance-norm statistic in 1.12.1, default range `[2, 10]` kept) and say so in docs/changelog. |

Source: GitHub issue #31
Base branch: develop

Open clarifications: none

## Acceptance criteria coverage

| # | Criterion | Covered by tasks |
|---|---|---|
| 1 | "F1 to F4 fixed with regression tests." | Task 1, Task 3 |
| 2 | "A decision is taken, implemented and documented for F5." | Task 4, Task 7 |
| 3 | "F6 and F8 fixed." | Task 2, Task 5 |
| 4 | "F7 documented in Javadoc and Antora docs, with the code changes that are accepted." | Task 1, Task 7 |
| 5 | "E1 and E2 of #29 fixed (they can be done together with this issue); E3 of #29 confirmed as resolved." | Task 6, Task 7 |
| 6 | derived: behavioural changes (F2, F3, F5, F7) are recorded in the changelog | Task 7 |

## Current code state

- Project type: `java-library` (Java 21, Maven, no wrapper, single module, `1.9.0-SNAPSHOT`), JUnit 5 + Mockito, JaCoCo,
  Checkstyle (`checkstyle.xml`), PMD, SpotBugs. No UI. Antora docs in `docs/` (`modules/ROOT/pages/{index,installation,reference,whats-new}.adoc`),
  `CHANGELOG.md` has `## [Unreleased]` (Keep a Changelog).
- Package `tf/` = `src/main/java/com/irurueta/navigation/inertial/calibration/intervals/thresholdfactor/`; tests mirror it under `src/test/java/...`.
  `cal/` = `src/main/java/com/irurueta/navigation/inertial/calibration/`.
- **F1/F7**: `Bracketed{Accelerometer,Gyroscope,Magnetometer,AccelerometerAndGyroscope,AccelerometerGyroscopeAndMagnetometer}IntervalDetectorThresholdFactorOptimizer`
  call `optimizer.setBracket(minThresholdFactor, minThresholdFactor, maxThresholdFactor)` only in `setMseOptimizer`
  (l.273 / 234 / 296 in three of them). `setThresholdFactorRange` lives in the abstract parents and never updates it. `optimize()` returns
  `Double.MAX_VALUE` from `optimizerListener` when detection/calibration fails (E2 fix), and checks `minMse == Double.MAX_VALUE`
  after `mseOptimizer.minimize()`.
- **F6**: `Exhaustive*Optimizer.optimize()` loops `for (var thresholdFactor = min; thresholdFactor <= max; thresholdFactor += step)` with
  `progressStep = step / (step + max - min)` (e.g. `ExhaustiveAccelerometer...:159-171`); `hasResult` is set only for MSE `< Double.MAX_VALUE` (E2 fix).
- **F8**: Javadoc of `thresholdFactorStep` (field/getter/setter) in `Exhaustive*` says "between `getMaxThresholdFactor()` and `getMaxThresholdFactor()`";
  `IntervalDetectorThresholdFactorOptimizer` class Javadoc and `optimize()` "Minimum Squared Error"; a comment in
  `AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer` omits the magnetometer.
- **F2-F4**: `cal/RandomWalkEstimator.addBodyKinematics` (~l.3960-4040) reads `numberOfSamples` before the sample is added, tests
  `numberOfSamples % driftPeriodSamples == 0`, updates the mean first then uses the updated mean in the variance update; `reset()` (~l.4052) resets only
  `biasEstimator` and statistics. Tests: `RandomWalkEstimatorTest`.
- **F5**: `cal/KalmanDriftEstimator.addBodyKinematics` (~l.1020-1060) only calls `kalmanEstimator.update(fixedKinematics, getElapsedTimeSeconds())`;
  `initialize()` (~l.1119) builds `INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame)`. 1.12.1 API of that class
  (`com.irurueta.navigation.inertial`): `update(BodyKinematics, double)`, `updatePositionAndVelocity(ECEFPosition, ECEFVelocity, double)`,
  `propagate(double)`, `getEpochInterval()`, `getState(...)`. Tests: `KalmanDriftEstimatorTest`, `DriftEstimatorTest` (reference behaviour).
- E1/E2 are fixed (`KnownBiasGyroscopeCalibrator` checks in the gyroscope-capable optimizers; `hasResult` only when MSE `< Double.MAX_VALUE`).
- Precedent: `.archive/implementation_plan_29.md` (same package, same structure of group/task/doc tasks).

## Implementation steps

### Group 1 (Parallelizable: yes — tasks touch disjoint main and test files)

- [x] Task 1. Bracketed optimizers: honour the threshold-factor range and make Brent robust (F1, F7) _(java)_ — implemented, tests added; validated: Bracketed* classes 85.3% line, 80.8-84.6% branch coverage, PMD clean, no new SpotBugs (CT_CONSTRUCTOR_THROW removed), Checkstyle only repo-wide LineLength style (five Bracketed*IntervalDetectorThresholdFactorOptimizer classes and Bracketed*Test classes)
  - [x] Task 1.1. In the five `Bracketed*IntervalDetectorThresholdFactorOptimizer` classes add a private `updateBracket()` that calls
    `mseOptimizer.setBracket(minThresholdFactor, centre, maxThresholdFactor)` with `centre = 0.5 * (min + max)`; call it from `setMseOptimizer`
    (when the optimizer is set) and at the start of `optimize()` (after `mseOptimizer` is checked and before `minimize()`), so
    `setThresholdFactorRange` after construction is honoured. Keep `setBracket` failures mapped as today (`LockedException`).
  - [x] Task 1.2. In the objective listener (`initializeOptimizerListeners`) return a large finite penalty constant
    (`private static final double INVALID_MSE_PENALTY = 1e30;` or similar, documented) instead of `Double.MAX_VALUE` for invalid factors, and
    treat `minMse >= INVALID_MSE_PENALTY` as "no valid result" (instead of `== Double.MAX_VALUE`) so the E2 behaviour (throw
    `IntervalDetectorThresholdFactorOptimizerException`) is preserved. Keep `evaluateForThresholdFactor` semantics for `keepBestResult` unchanged.
  - [x] Task 1.3. Javadoc on the five classes: describe the piecewise-constant objective, plateaus, that the returned factor is the best one *evaluated*
    (not necessarily the minimum in the range), and recommend the `Exhaustive*` optimizers as default.
  - [x] Task 1.4. Tests in the five `Bracketed*Test` classes: bracket equals `[min, centre, max]` after `setThresholdFactorRange(20.0, 30.0)`
    and after a second range change; `optimize()` with a non-default range returns a threshold factor inside that range (use the synthetic data
    approach of `ThresholdFactorOptimizersEndToEndTest` where possible); an invalid-everywhere data source still throws.

- [x] Task 2. Exhaustive optimizers: index-based iteration and Javadoc (F6, F8 part 1) _(java)_
  - [x] Task 2.1. In the five `Exhaustive*IntervalDetectorThresholdFactorOptimizer` classes compute
    `final var count = (int) Math.round((max - min) / step) + 1` (guard `count >= 1`), iterate `for (var i = 0; i < count; i++)` with
    `thresholdFactor = Math.min(max, min + i * step)` (so the maximum is always evaluated and never exceeded), and compute `progressStep = 1.0f / count`.
  - [x] Task 2.2. Fix the `thresholdFactorStep` Javadoc (field, getter, setter): "between `getMinThresholdFactor()` and `getMaxThresholdFactor()`".
  - [x] Task 2.3. Tests in the five `Exhaustive*Test` classes: a range/step such as `[0.0, 0.3]` with step `0.1` evaluates 4 factors including the maximum
    (e.g. via a listener/spy counting evaluations, or by checking the optimal factor can be the maximum); progress ends at `1.0`.
    _Task 2 implemented in the five Exhaustive* optimizers and their tests; validated: Exhaustive* 95-98% line, 94.4% branch coverage, no new PMD or SpotBugs._

- [x] Task 3. `RandomWalkEstimator`: drift-period counting, recursive variance, reset (F2, F3, F4) _(java)_ — implemented in RandomWalkEstimator.java / RandomWalkEstimatorTest.java; validated: RandomWalkEstimator 96.7% line, 98.4% branch, no new PMD or SpotBugs.
  - [x] Task 3.1. F2: in `addBodyKinematics` compute the samples including the current one (`numberOfSamples + 1`, or read
    `getNumberOfProcessedSamples()` after `biasEstimator.addBodyKinematics`) and close the period when `% driftPeriodSamples == 0`; keep the
    `onStart` condition on the pre-increment count. — implemented in RandomWalkEstimator.java / RandomWalkEstimatorTest.java; validated: RandomWalkEstimator 96.7% line, 98.4% branch, no new PMD or SpotBugs.
  - [x] Task 3.2. F3: compute `diffPrev = drift - avgPrev` before updating the mean, update the mean, `diffNew = drift - avgNew`, then
    `var = var * tmp + diffPrev * diffNew / n` for position, velocity and attitude. Document in Javadoc of the three variance getters (and their
    standard deviations) that the **population variance** is returned. — implemented in RandomWalkEstimator.java / RandomWalkEstimatorTest.java; validated: RandomWalkEstimator 96.7% line, 98.4% branch, no new PMD or SpotBugs.
  - [x] Task 3.3. F4: call `driftEstimator.reset()` in `reset()` (inside the existing locked section; keep the return value semantics) and document it. — implemented in RandomWalkEstimator.java / RandomWalkEstimatorTest.java; validated: RandomWalkEstimator 96.7% line, 98.4% branch, no new PMD or SpotBugs.
  - [x] Task 3.4. Update `RandomWalkEstimatorTest`: expected `getNumberOfProcessedDriftPeriods()` after N samples is `N / driftPeriodSamples`
    (first period not closed after one sample); add tests: variance of known drift sequence (`0, 2` -> `1`) and a batch comparison over several
    periods for position/velocity/attitude; reset in the middle of a drift period leaves `getNumberOfProcessedDriftPeriods() == 0` and a clean first period. — implemented in RandomWalkEstimator.java / RandomWalkEstimatorTest.java; validated: RandomWalkEstimator 96.7% line, 98.4% branch, no new PMD or SpotBugs.

- [x] Task 4. `KalmanDriftEstimator`: run the Kalman filter with static fixes and integrate every sample (F5) _(java)_ — implemented in KalmanDriftEstimator.java and KalmanDriftEstimatorTest.java; validated: KalmanDriftEstimator 97.2% line, 67.6% branch, no new PMD or SpotBugs.
  - [x] Task 4.1. In `initialize()` create the filter, then `kalmanEstimator.update(null, 0.0)`-style initialisation so the first real sample is integrated over
    one `timeInterval` (confirm the exact accepted arguments against the 1.12.1 API with a quick test; adjust the timestamp convention to
    `(numberOfProcessedSamples + 1) * timeInterval`).
  - [x] Task 4.2. In `addBodyKinematics`, after `update(fixedKinematics, timestamp)`, every `epochInterval` seconds (`kalmanEstimator.getEpochInterval()`,
    counted in samples, at least 1) call `kalmanEstimator.updatePositionAndVelocity(referenceECEFPosition, zeroECEFVelocity, timestamp)` using
    the reference frame position and zero velocity (preallocated fields, no allocation per sample); keep `getState` and the drift computations as they are.
  - [x] Task 4.3. Update class/method Javadoc: describe the INS Kalman filtered solution fed with static position/velocity fixes, the epoch interval
    and the timestamp convention. Reset must discard the filter as today.
  - [x] Task 4.4. Update `KalmanDriftEstimatorTest`: compare with `DriftEstimator` sample by sample (same number of integrated samples; with the
    filter running the drift must be less than or equal to the dead-reckoning drift for a biased/noisy static body, or equal when the filter is
    configured not to correct); assert `getElapsedTimeSeconds()` after the first sample equals one `timeInterval`; remove the previous "equal drift" expectations that no
    longer hold.

- [x] Task 5. Remaining Javadoc mistakes in the abstract optimizers (F8 part 2) _(java)_ — Javadoc/comment fixes done; validated: comment-only, no new issues.
  - [x] Task 5.1. `IntervalDetectorThresholdFactorOptimizer`: class Javadoc "interval detection of accelerometer data" -> generic wording
    (accelerometer, gyroscope and/or magnetometer data); `optimize()` "Minimum Squared Error" -> "Mean Square Error".
  - [x] Task 5.2. `AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer`: comment "convert accelerometer and gyroscope mse to global mse"
    -> include the magnetometer. Grep the package for the same copy-paste wording in the other abstract optimizers.

### Group 2 (Parallelizable: yes — one task, verification of #29 items; runs after Group 1 because it touches the same test classes)

- [x] Task 6. Verify E1, E2 and E3 of #29 on `develop` _(java)_
  - [x] Task 6.1. Confirm each gyroscope-capable optimizer test class has a regression test with a `KnownBiasGyroscopeCalibrator` (E1) and that every
    exhaustive/bracketed test class has a test where no threshold factor yields a valid calibration (E2: exception, no stale result); add the missing ones.
  - [x] Task 6.2. Confirm E3 in code (`DEFAULT_MIN_THRESHOLD_FACTOR = 2.0` / `DEFAULT_MAX_THRESHOLD_FACTOR = 10.0` with the 1.12.1 variance-norm statistic) and note the outcome
    in Task 7's docs; no code change expected.
  - [x] Task 6.3. Run the tests of the whole `thresholdfactor` package and `calibration` estimators via the `iru-gate-runner` agent
    (`Agent({description: "Run calibration tests", subagent_type: "iru-gate-runner", prompt: "Invoke Skill({skill: \"iru-java-test\", args: \"com.irurueta.navigation.inertial.calibration.**\"}) and report a compact summary"})`).

### Group 3 (Parallelizable: yes — single task)

- [x] Task 7. Documentation and changelog _(untagged)_
  - [x] Task 7.1. `docs/modules/ROOT/pages/reference.adoc` / `index.adoc`: document (a) the Brent limitation of the bracketed optimizers and the recommendation to use the exhaustive
    ones, honoured range and centre start (F1, F7); (b) `RandomWalkEstimator` population variance, drift-period counting and reset behaviour, and that the bias PSD is
    a documented heuristic `(‖mean bias‖ / elapsed time)² · timeInterval`, not an Allan-variance estimate; (c) `KalmanDriftEstimator` static-fix filtering (F5);
    (d) E1/E2 fixed and E3 confirmed resolved. Review `README.md` ("Allan-variance-style" wording) and `whats-new.adoc`.
  - [x] Task 7.2. `CHANGELOG.md` under `## [Unreleased]`: `### Fixed` entries for F1, F2, F3, F4, F5, F6, F8 and `### Changed` for F7 (penalty, centre start, docs) and the
    behaviour changes (number of drift periods, variance values, Kalman drift values).
  - [x] Task 7.3. Run `/iru-update-docs` and build the Antora docs with `/iru-build-docs` to verify the site builds.
