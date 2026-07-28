# irurueta-navigation-inertial-extra

Additional calibration estimators for GNSS/INS navigation

[![Build Status](https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/actions/workflows/main.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/actions/workflows/main.yml)
[![Build Status](https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/actions/workflows/develop.yml/badge.svg)](https://github.com/albertoirurueta/irurueta-navigation-inertial-extra/actions/workflows/develop.yml)
[![Maven Central](https://img.shields.io/maven-central/v/com.irurueta/irurueta-navigation-inertial-extra.svg)](https://search.maven.org/artifact/com.irurueta/irurueta-navigation-inertial-extra)

[![Bugs](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=bugs)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
[![Code Smells](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=code_smells)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
[![Coverage](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=coverage)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)

[![Duplicated lines](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=duplicated_lines_density)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
[![Lines of code](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=ncloc)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)

[![Maintainability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=sqale_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
[![Quality gate](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=alert_status)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
[![Reliability](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=reliability_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)

[![Security](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=security_rating)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
[![Technical debt](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=sqale_index)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
[![Vulnerabilities](https://sonarcloud.io/api/project_badges/measure?project=albertoirurueta_irurueta-navigation-inertial-extra&metric=vulnerabilities)](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)

`irurueta-navigation-inertial-extra` builds on top of
[irurueta-navigation-inertial](https://github.com/albertoirurueta/irurueta-navigation-inertial) with the
higher-level tools needed to validate an IMU calibration and to automatically tune it: drift estimation, sensor
random-walk (Allan-variance-style) estimation, and threshold-factor optimizers for the static/dynamic interval
detectors used during calibration data collection.

> **⚠️ Experimental.** This library implements research-grade calibration-validation and tuning techniques —
> notably the static/dynamic interval detection and threshold-factor search inspired by David Tedaldi, Alberto
> Pretto, Emanuele Menegatti,
> ["A Robust and Easy to Implement Method for IMU Calibration without External Equipments"](https://albertopretto.altervista.org/papers/tpm_icra2014.pdf)
> (ICRA 2014; reference implementation: [imu_tk](https://github.com/Kyle-ak/imu_tk)) — that have not been
> hardened through large-scale production use. APIs may still change between minor releases.

## Project Status

| | |
| --- | --- |
| Language | Java 21 |
| Build tool | Maven |
| Current development version | 1.9.0-SNAPSHOT |
| Latest release | 1.8.0 |
| License | Apache License 2.0 |
| CI | GitHub Actions — build/test/Sonar/docs on every push to `develop`, and on every published release |
| Quality | SonarCloud, JaCoCo coverage, Checkstyle, SpotBugs, PMD |

## Documentation

* [Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation-inertial-extra) — conceptual
  guide, installation instructions and reference links.
* [Maven site report](https://albertoirurueta.github.io/irurueta-navigation-inertial-extra/mvn-site) — Javadoc,
  Surefire test results, JaCoCo coverage, Checkstyle, SpotBugs, PMD and cross-referenced source.
* [SonarCloud dashboard](https://sonarcloud.io/dashboard?id=albertoirurueta_irurueta-navigation-inertial-extra)
* [Changelog](CHANGELOG.md)

## Installation

Add the following dependency to your project:

Latest release:
```
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation-inertial-extra</artifactId>
    <version>1.8.0</version>
    <scope>compile</scope>
</dependency>
```

Latest snapshot:
```
<dependency>
    <groupId>com.irurueta</groupId>
    <artifactId>irurueta-navigation-inertial-extra</artifactId>
    <version>1.9.0-SNAPSHOT</version>
    <scope>compile</scope>
</dependency>
```

## How It Works

Once accelerometer and gyroscope measurements have been fixed using a calibration model, `DriftEstimator` (and its
Kalman-filter-based subclass `KalmanDriftEstimator`) lets you quantify how much residual error still accumulates
while the body carrying the IMU remains static — feed it consecutive `BodyKinematics` samples and it reports the
accumulated drift in position, velocity and orientation:

```java
// Initial static position/attitude of the IMU, expressed in NED coordinates.
final NEDFrame referenceFrame = new NEDFrame(latitude, longitude, height);

final DriftEstimator estimator = new DriftEstimator(referenceFrame);

// Feed already-calibrated accelerometer/gyroscope samples collected while the
// device remains static.
for (final BodyKinematics kinematics : staticKinematicsSamples) {
    estimator.addBodyKinematics(kinematics);
}

// Query the accumulated drift once all samples have been processed.
final Distance positionDrift = new Distance(0.0, DistanceUnit.METER);
estimator.getCurrentPositionDriftNorm(positionDrift);
```

`RandomWalkEstimator` repeats this process over many consecutive static periods to estimate accelerometer/gyroscope
bias random walk, and the `IntervalDetectorThresholdFactorOptimizer` hierarchy searches for the static/dynamic
interval-detection threshold factor that minimizes calibration error. See the
[Antora documentation site](https://albertoirurueta.github.io/irurueta-navigation-inertial-extra) for the full
conceptual guide.

## License

This library is licensed under the [Apache License, Version 2.0](LICENSE.txt).
