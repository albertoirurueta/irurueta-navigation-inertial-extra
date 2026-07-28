# Changelog

All notable changes to this project are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/), and this project adheres to
[Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.8.0] - 2026-07-28

### Changed

- Raised the minimum/target Java version from 17 to 21; consumers must build/run against JDK 21 or later.
- Updated internal dependency versions. No changes to this library's own source code in this release.

## [1.7.0] - 2026-03-05

### Changed

- Updated dependency versions: `irurueta-numerical` to 1.5.0, `irurueta-geometry` to 1.5.0, `irurueta-navigation`
  to 1.7.1, and `irurueta-navigation-inertial` to 1.9.0. No changes to this library's own source code in this
  release.

## [1.6.0] - 2026-02-04

### Changed

- Updated the `irurueta-navigation-inertial` dependency to 1.8.0. No changes to this library's own source in
  this release.

## [1.5.0] - 2025-12-27

### Changed

- Updated internal `irurueta-*` dependency versions (`irurueta-numerical`, `irurueta-geometry`,
  `irurueta-navigation`, `irurueta-navigation-inertial`). No changes to this library's own source in this
  release.

## [1.4.2] - 2025-09-23

### Changed

- Updated internal `irurueta-*` dependencies (`irurueta-numerical`, `irurueta-geometry`, `irurueta-algebra`,
  `irurueta-navigation`, `irurueta-navigation-inertial`) to their latest patch versions.

## [1.4.1] - 2025-09-21

### Changed

- Updated compile-scope dependency versions used by consumers: `irurueta-navigation`, `irurueta-navigation-inertial`,
  `irurueta-numerical`, `irurueta-geometry`, `irurueta-units`, and `irurueta-algebra`.

## [1.4.0] - 2025-05-11

### Changed

- Updated transitive dependencies: `irurueta-navigation`, `irurueta-navigation-inertial`, `irurueta-numerical`,
  `irurueta-geometry`, `irurueta-units`, and `irurueta-algebra`.
- Internal code cleanup across the calibration package (naming conventions, type inference); the public API is
  unchanged.

## [1.3.2] - 2024-10-11

No user-facing changes — this release consists solely of a CI workflow configuration fix and the version bump.

## [1.3.1] - 2024-10-11

No user-facing changes — re-released as 1.3.1 with no functional changes (version/metadata only).

## [1.3.0] - 2024-10-11

### Fixed

- `RandomWalkEstimator`: corrected the position-drift variance calculation, which was incorrectly combining
  position and attitude drift terms instead of using the position drift term twice, causing inaccurate position
  drift statistics/variance estimates.

### Changed

- Raised the minimum supported Java version to 17 (previously Java 7); consumers must upgrade their JDK to use
  this version.
- Updated transitive dependencies: `irurueta-navigation`, `irurueta-navigation-inertial`, `irurueta-numerical`,
  `irurueta-geometry`, `irurueta-units`, and `irurueta-algebra`.

## [1.2.0] - 2022-08-01

No user-facing changes — this release consists of test-suite improvements and documentation updates only; no
files under the library's source or resources were modified.

## [1.1.1] - 2022-01-27

### Changed

- Updated the `irurueta-navigation-inertial` dependency to version 1.1.1.

## [1.1.0] - 2021-12-12

### Changed

- Updated compile-scope dependencies on sibling `irurueta-*` libraries (`irurueta-numerical`, `irurueta-geometry`,
  `irurueta-units`, `irurueta-algebra`, `irurueta-navigation`, `irurueta-navigation-inertial`) from 1.0.0 to
  1.1.0.

## [1.0.0] - 2021-12-10

Initial release. This library provides additional GNSS/INS calibration estimators as a companion to
`irurueta-navigation` and `irurueta-navigation-inertial`.

### Added

- `DriftEstimator` / `KalmanDriftEstimator`: estimate accumulated drift in body orientation, position, and
  velocity per unit of time from an already-calibrated IMU.
- `RandomWalkEstimator`: estimates random walk (bias drift) of accelerometer/gyroscope while the device remains
  static.
- `AccelerometerBiasRandomWalkSource` / `GyroscopeBiasRandomWalkSource` interfaces exposing estimated random-walk
  PSD values for accelerometer and gyroscope bias.
- `PositionUncertaintySource`, `VelocityUncertaintySource`, `AttitudeUncertaintySource`,
  `PositionNoiseStandardDeviationSource`, `VelocityNoiseStandardDeviationSource` interfaces exposing
  uncertainty/noise standard deviations from drift/random-walk estimators.
- `IntervalDetectorThresholdFactorOptimizer` framework (accelerometer, gyroscope, magnetometer, and combined
  variants) to find the optimal static-interval-detection threshold factor that minimizes calibration error.
- `Exhaustive*` and `Bracketed*` optimizer implementations (exhaustive grid search and bracketed search) for
  accelerometer, gyroscope, magnetometer, and combined calibration.
- Default quality-score mappers and MSE rules providing ready-to-use scoring strategies for the optimizers.
- `INSLooselyCoupledKalmanConfigCreator` / `INSLooselyCoupledKalmanInitializerConfigCreator` helpers to build INS
  Loosely Coupled Kalman filter/initializer configuration from estimated noise characteristics.
- `DriftEstimationException` and `RandomWalkEstimationException` exception types.
- Listener interfaces (`DriftEstimatorListener`, `RandomWalkEstimatorListener`,
  `IntervalDetectorThresholdFactorOptimizerListener`) for progress/result callbacks.

[Unreleased]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.8.0...HEAD
[1.8.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.7.0...1.8.0
[1.7.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.6.0...1.7.0
[1.6.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.5.0...1.6.0
[1.5.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.4.2...1.5.0
[1.4.2]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.4.1...1.4.2
[1.4.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.4.0...1.4.1
[1.4.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.3.2...1.4.0
[1.3.2]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.3.1...1.3.2
[1.3.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.3.0...1.3.1
[1.3.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.2.0...1.3.0
[1.2.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.1.1...1.2.0
[1.1.1]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.1.0...1.1.1
[1.1.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/compare/1.0.0...1.1.0
[1.0.0]: https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/releases/tag/1.0.0
