/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor;

import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * End-to-end tests of the threshold factor optimizers using a deterministic (seeded) synthetic IMU stream
 * with known accelerometer and gyroscope parameters.
 */
class ThresholdFactorOptimizersEndToEndTest {

    private static final double TIME_INTERVAL_SECONDS = 0.02;
    private static final double GRAVITY = 9.80665;
    private static final double MICRO_G = 9.80665e-6;
    private static final double DEG_TO_RAD = Math.PI / 180.0;

    private static final int INITIAL_STATIC_SAMPLES = 5000;
    private static final int STATIC_SAMPLES = 303;
    private static final int DYNAMIC_SAMPLES = 400;

    private static final long[] SEEDS = {1L, 2L, 3L};

    // Noise: accelerometer 100 micro-g/sqrt(Hz) and gyroscope 0.01 deg/sqrt(h) like in the paper (per sample
    // standard deviation is rootPsd / sqrt(timeInterval)).
    private static final double ACCEL_NOISE_ROOT_PSD = 100.0 * MICRO_G;
    private static final double GYRO_NOISE_ROOT_PSD = 0.01 * DEG_TO_RAD / 60.0;

    // Tolerances. Each static position is averaged over ~300 samples (per sample std ~7e-3 m/s^2), and 36-50
    // positions are used. Maximum errors observed over the fixed seeds are ba 1.8e-4, Ma 3.9e-5, bg 3.6e-6 and
    // Mg 4.4e-5, so tolerances are ~5 times larger than those.
    private static final double MA_TOLERANCE = 2e-4;
    private static final double BA_TOLERANCE = 1e-3;
    private static final double MG_TOLERANCE = 2e-4;
    private static final double BG_TOLERANCE = 2e-5;

    private final List<TimedBodyKinematics> samples = new ArrayList<>();

    private final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
            new AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource() {
                @Override
                public int count() {
                    return samples.size();
                }

                @Override
                public TimedBodyKinematics getAt(final int index) {
                    return samples.get(index);
                }
            };

    @Test
    void testDefaultThresholdFactorRangeIsKept() {
        assertEquals(2.0, AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR, 0.0);
        assertEquals(10.0, AccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR, 0.0);
        assertEquals(2.0, GyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR, 0.0);
        assertEquals(10.0, GyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR, 0.0);
        assertEquals(2.0, MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR, 0.0);
        assertEquals(10.0, MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR, 0.0);
        assertEquals(2.0, AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                0.0);
        assertEquals(10.0,
                AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR, 0.0);
        assertEquals(2.0,
                AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer
                        .DEFAULT_MIN_THRESHOLD_FACTOR, 0.0);
        assertEquals(10.0,
                AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer
                        .DEFAULT_MAX_THRESHOLD_FACTOR, 0.0);
    }

    @Test
    void testExhaustiveOptimizerRecoversKnownParameters() throws Exception {
        for (final var seed : SEEDS) {
            generateSamples(seed);

            final var optimizer = new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                    dataSource, createAccelerometerCalibrator(), createGyroscopeCalibrator());
            final var thresholdFactor = optimizer.optimize();

            checkResult(optimizer, thresholdFactor);

            // optimization is deterministic for the same data
            final var repeated = new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                    dataSource, createAccelerometerCalibrator(), createGyroscopeCalibrator());
            assertEquals(thresholdFactor, repeated.optimize(), 0.0);
        }
    }

    @Test
    void testBracketedOptimizerRecoversKnownParameters() throws Exception {
        for (final var seed : SEEDS) {
            generateSamples(seed);

            final var optimizer = new BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                    dataSource, createAccelerometerCalibrator(), createGyroscopeCalibrator());
            final var thresholdFactor = optimizer.optimize();

            checkResult(optimizer, thresholdFactor);
        }
    }

    @Test
    void testFailingDataThrowsAndResetsResults() throws Exception {
        generateSamples(SEEDS[0]);

        final var optimizer = new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                dataSource, createAccelerometerCalibrator(), createGyroscopeCalibrator());
        optimizer.optimize();
        assertTrue(optimizer.getThreshold() > 0.0);

        // too little data makes interval detection fail for every threshold factor (E2)
        samples.clear();
        assertThrows(IntervalDetectorThresholdFactorOptimizerException.class, optimizer::optimize);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);

        generateSamples(SEEDS[0]);
        final var bracketed = new BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                dataSource, createAccelerometerCalibrator(), createGyroscopeCalibrator());
        bracketed.optimize();
        samples.clear();
        assertThrows(IntervalDetectorThresholdFactorOptimizerException.class, bracketed::optimize);
        assertEquals(0.0, bracketed.getOptimalThresholdFactor(), 0.0);
        assertEquals(0.0, bracketed.getThreshold(), 0.0);
    }

    @Test
    void testInitialGyroscopeBiasIsUsedByDefault() throws Exception {
        generateSamples(SEEDS[0]);

        final var gyroscopeCalibrator = createGyroscopeCalibrator();
        final var optimizer = new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                dataSource, createAccelerometerCalibrator(), gyroscopeCalibrator);
        assertTrue(optimizer.isInitialGyroscopeBiasUsed());
        optimizer.optimize();

        // initial bias is the mean angular rate of the initial static period, which must be close to
        // the true bias (E4): noise of the mean is ~3e-7 rad/s
        final var bg = bg().getBuffer();
        final var initialBias = gyroscopeCalibrator.getInitialBias();
        assertNotNull(initialBias);
        assertTrue(Math.abs(initialBias[0]) > 0.0);
        assertArrayEquals(bg, initialBias, 5e-6);

        // when the option is disabled, the calibrator initial bias is kept
        final var otherCalibrator = createGyroscopeCalibrator();
        final var kept = new double[]{1e-3, -1e-3, 2e-3};
        otherCalibrator.setInitialBias(kept);
        final var other = new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                dataSource, createAccelerometerCalibrator(), otherCalibrator);
        other.setInitialGyroscopeBiasUsed(false);
        try {
            other.optimize();
        } catch (final IntervalDetectorThresholdFactorOptimizerException ignore) {
            // calibration might fail with a wrong initial bias, which is irrelevant for this check
        }
        assertArrayEquals(kept, otherCalibrator.getInitialBias(), 0.0);
    }

    private void checkResult(
            final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer,
            final double thresholdFactor) throws WrongSizeException {
        assertTrue(thresholdFactor >= 2.0 && thresholdFactor <= 10.0, "threshold factor " + thresholdFactor);
        assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);

        final var estimatedBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
        final var estimatedMa = optimizer.getEstimatedAccelerometerMa();
        final var estimatedBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
        final var estimatedMg = optimizer.getEstimatedGyroscopeMg();

        assertTrue(ba().equals(estimatedBa, BA_TOLERANCE), "ba " + estimatedBa);
        assertTrue(ma().equals(estimatedMa, MA_TOLERANCE), "ma " + estimatedMa);
        assertTrue(bg().equals(estimatedBg, BG_TOLERANCE), "bg " + estimatedBg);
        assertTrue(mg().equals(estimatedMg, MG_TOLERANCE), "mg " + estimatedMg);
    }

    private static KnownGravityNormAccelerometerCalibrator createAccelerometerCalibrator() throws WrongSizeException {
        return new KnownGravityNormAccelerometerCalibrator(GRAVITY, true, new Matrix(3, 1), new Matrix(3, 3));
    }

    private static EasyGyroscopeCalibrator createGyroscopeCalibrator() throws LockedException,
            WrongSizeException {
        final var calibrator = new EasyGyroscopeCalibrator();
        calibrator.setCommonAxisUsed(true);
        calibrator.setGDependentCrossBiasesEstimated(false);
        calibrator.setInitialBias(new double[3]);
        calibrator.setInitialMg(new Matrix(3, 3));
        calibrator.setInitialGg(new Matrix(3, 3));
        return calibrator;
    }

    private static Matrix ba() {
        return Matrix.newFromArray(new double[]{900 * MICRO_G, -1300 * MICRO_G, 800 * MICRO_G});
    }

    private static Matrix bg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0, 13 * DEG_TO_RAD / 3600.0, -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix ma() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);
        return result;
    }

    private static Matrix mg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);
        return result;
    }

    /**
     * Generates an initial static period (with gyroscope bias), followed by 36 to 50 static positions
     * separated by rotations about random axes.
     *
     * @param seed seed of the random generator.
     */
    private void generateSamples(final long seed) throws WrongSizeException {
        samples.clear();

        final var random = new Random(seed);
        final var errors = new IMUErrors(ba(), bg(), ma(), mg(), new Matrix(3, 3), ACCEL_NOISE_ROOT_PSD,
                GYRO_NOISE_ROOT_PSD, 0.0, 0.0);

        // body to local rotation (row-major), initially level
        var rotation = new double[]{1, 0, 0, 0, 1, 0, 0, 0, 1};
        var index = 0;

        index = addStatic(INITIAL_STATIC_SAMPLES, rotation, errors, random, index);

        final var numPositions = 36 + random.nextInt(15);
        for (var i = 0; i < numPositions; i++) {
            index = addStatic(STATIC_SAMPLES, rotation, errors, random, index);
            if (i < numPositions - 1) {
                // random unit axis (in body frame) and rotation angle between 60 and 170 degrees
                final var axis = new double[]{random.nextGaussian(), random.nextGaussian(), random.nextGaussian()};
                final var norm = Math.sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]);
                for (var k = 0; k < 3; k++) {
                    axis[k] /= norm;
                }
                final var angle = Math.toRadians(60.0 + 110.0 * random.nextDouble());
                rotation = addDynamic(rotation, axis, angle, errors, random, index);
                index += DYNAMIC_SAMPLES;
            }
        }
    }

    private int addStatic(
            final int numSamples, final double[] rotation, final IMUErrors errors, final Random random,
            final int startIndex) {
        var index = startIndex;
        for (var i = 0; i < numSamples; i++, index++) {
            add(index, trueKinematics(rotation, 0.0, 0.0, 0.0), errors, random);
        }
        return index;
    }

    private double[] addDynamic(
            final double[] startRotation, final double[] axis, final double angle, final IMUErrors errors,
            final Random random, final int startIndex) {
        var rotation = startRotation;
        for (var i = 0; i < DYNAMIC_SAMPLES; i++) {
            // smooth bump profile of angular rate, whose integral over the period equals the angle
            final var p = (double) i / (double) DYNAMIC_SAMPLES;
            final var s = Math.sin(Math.PI * p);
            final var rate = 2.0 * angle / (DYNAMIC_SAMPLES * TIME_INTERVAL_SECONDS) * s * s;

            add(startIndex + i, trueKinematics(rotation, axis[0] * rate, axis[1] * rate, axis[2] * rate), errors,
                    random);

            rotation = multiply(rotation, rodrigues(axis, rate * TIME_INTERVAL_SECONDS));
        }
        return rotation;
    }

    private void add(final int index, final BodyKinematics truth, final IMUErrors errors, final Random random) {
        final var measured = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, truth, errors, random);
        final var timed = new TimedBodyKinematics();
        timed.setKinematics(measured);
        timed.setTimestampSeconds(index * TIME_INTERVAL_SECONDS);
        samples.add(timed);
    }

    /**
     * Gets true kinematics: specific force is the reaction to gravity (local down) expressed in body frame, which
     * is the third row of the body to local rotation matrix times -g.
     */
    private static BodyKinematics trueKinematics(
            final double[] rotation, final double wx, final double wy, final double wz) {
        final var result = new BodyKinematics();
        result.setSpecificForceCoordinates(-GRAVITY * rotation[6], -GRAVITY * rotation[7], -GRAVITY * rotation[8]);
        result.setAngularRateCoordinates(wx, wy, wz);
        return result;
    }

    private static double[] rodrigues(final double[] axis, final double theta) {
        final var c = Math.cos(theta);
        final var s = Math.sin(theta);
        final var t = 1.0 - c;
        final var x = axis[0];
        final var y = axis[1];
        final var z = axis[2];
        return new double[]{
                t * x * x + c, t * x * y - s * z, t * x * z + s * y,
                t * x * y + s * z, t * y * y + c, t * y * z - s * x,
                t * x * z - s * y, t * y * z + s * x, t * z * z + c
        };
    }

    private static double[] multiply(final double[] a, final double[] b) {
        final var result = new double[9];
        for (var i = 0; i < 3; i++) {
            for (var j = 0; j < 3; j++) {
                var sum = 0.0;
                for (var k = 0; k < 3; k++) {
                    sum += a[i * 3 + k] * b[k * 3 + j];
                }
                result[i * 3 + j] = sum;
            }
        }
        return result;
    }
}
