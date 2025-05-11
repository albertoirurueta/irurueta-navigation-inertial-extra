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
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.BodyMagneticFluxDensity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.BodyMagneticFluxDensityGenerator;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownFrameMagnetometerNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.navigation.inertial.estimators.BodyMagneticFluxDensityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.numerical.optimization.BrentSingleOptimizer;
import com.irurueta.statistics.GaussianRandomizer;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.jupiter.api.Test;

import java.io.IOException;
import java.util.*;

import static org.junit.jupiter.api.Assertions.*;
import static org.mockito.Mockito.mock;

class BracketedMagnetometerIntervalDetectorThresholdFactorOptimizerTest implements
        IntervalDetectorThresholdFactorOptimizerListener<BodyKinematicsAndMagneticFluxDensity,
                MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_HARD_IRON = -1e-5;
    private static final double MAX_HARD_IRON = 1e-5;

    private static final double MIN_SOFT_IRON = -1e-6;
    private static final double MAX_SOFT_IRON = 1e-6;

    private static final double MIN_ANGLE_DEGREES = -45.0;
    private static final double MAX_ANGLE_DEGREES = 45.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;

    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;

    private static final double MIN_HEIGHT_METERS = -500.0;
    private static final double MAX_HEIGHT_METERS = 10000.0;

    private static final double SMALL_MAGNETOMETER_NOISE_STD = 1e-12;
    private static final double MAGNETOMETER_NOISE_STD = 200e-9;

    private static final double MIN_DELTA_POS_METERS = -0.01;
    private static final double MAX_DELTA_POS_METERS = 0.01;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-9;
    private static final double LARGE_ABSOLUTE_ERROR = 5e-5;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final Calendar START_CALENDAR = Calendar.getInstance();
    private static final Calendar END_CALENDAR = Calendar.getInstance();

    private static final long START_TIMESTAMP_MILLIS;
    private static final long END_TIMESTAMP_MILLIS;

    static {
        START_CALENDAR.set(2020, Calendar.JANUARY, 1, 0, 0, 0);
        END_CALENDAR.set(2025, Calendar.DECEMBER, 31, 23, 59, 59);

        START_TIMESTAMP_MILLIS = START_CALENDAR.getTimeInMillis();
        END_TIMESTAMP_MILLIS = END_CALENDAR.getTimeInMillis();
    }

    private final List<BodyKinematicsAndMagneticFluxDensity> bodyKinematicsAndMagneticFluxDensities =
            new ArrayList<>();

    private final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
            new MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource() {
                @Override
                public int count() {
                    return bodyKinematicsAndMagneticFluxDensities.size();
                }

                @Override
                public BodyKinematicsAndMagneticFluxDensity getAt(final int index) {
                    return bodyKinematicsAndMagneticFluxDensities.get(index);
                }
            };

    private final MagnetometerMeasurementsGeneratorListener generatorListener =
            new MagnetometerMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onInitializationCompleted(
                        final MagnetometerMeasurementsGenerator generator, final double baseNoiseLevel) {
                    // not used
                }

                @Override
                public void onError(
                        final MagnetometerMeasurementsGenerator generator, 
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    // not used
                }

                @Override
                public void onStaticIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onStaticIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onGeneratedMeasurement(
                        final MagnetometerMeasurementsGenerator generator,
                        final StandardDeviationBodyMagneticFluxDensity measurement) {
                    generatorMeasurements.add(measurement);
                }

                @Override
                public void onReset(final MagnetometerMeasurementsGenerator generator) {
                    // not used
                }
            };

    private final List<StandardDeviationBodyMagneticFluxDensity> generatorMeasurements = new ArrayList<>();

    private int start;

    private int end;

    private float progress;

    @Test
    void testConstructor1() {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);
    }

    @Test
    void testConstructor2() {
        final var ds = mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(ds);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertSame(ds, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);
    }

    @Test
    void testConstructor3() {
        final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();

        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(calibrator);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // Force IllegalArgumentException
        final var wrongCalibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();
        assertThrows(IllegalArgumentException.class,
                () -> new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(wrongCalibrator));
    }

    @Test
    void testConstructor4() {
        final var ds = mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();

        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(ds, calibrator);

        // check default values
        assertNotNull(optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertTrue(optimizer.isReady());
        assertSame(ds, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // Force IllegalArgumentException
        final var wrongCalibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();
        assertThrows(IllegalArgumentException.class,
                () -> new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(ds, wrongCalibrator));
    }

    @Test
    void testConstructor5() {
        final var mseOptimizer = new BrentSingleOptimizer();

        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);
    }

    @Test
    void testConstructor6() {
        final var ds = mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        final var mseOptimizer = new BrentSingleOptimizer();

        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(ds,
                mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertNull(optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertSame(ds, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);
    }

    @Test
    void testConstructor7() {
        final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();
        final var mseOptimizer = new BrentSingleOptimizer();

        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(calibrator,
                mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // Force IllegalArgumentException
        final var wrongCalibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();
        assertThrows(IllegalArgumentException.class,
                () -> new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(wrongCalibrator, mseOptimizer));
    }

    @Test
    void testConstructor8() {
        final var ds = mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();
        final var mseOptimizer = new BrentSingleOptimizer();

        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(ds, calibrator,
                mseOptimizer);

        // check default values
        assertSame(mseOptimizer, optimizer.getMseOptimizer());
        assertSame(calibrator, optimizer.getCalibrator());
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertTrue(optimizer.isReady());
        assertSame(ds, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final var acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final var acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final var acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final var acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedHardIron());
        assertNull(optimizer.getEstimatedMm());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // Force IllegalArgumentException
        final var wrongCalibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();
        assertThrows(IllegalArgumentException.class,
                () -> new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(ds, wrongCalibrator,
                        mseOptimizer));
    }

    @Test
    void testGetSetMseOptimizer() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getMseOptimizer());

        // set a new value
        final var mseOptimizer = new BrentSingleOptimizer();
        optimizer.setMseOptimizer(mseOptimizer);

        // check
        assertSame(mseOptimizer, optimizer.getMseOptimizer());

        // set null value
        optimizer.setMseOptimizer(null);

        // check
        assertNull(optimizer.getMseOptimizer());
    }

    @Test
    void testIsReady() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertFalse(optimizer.isReady());

        // set data source
        final var ds = mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);
        optimizer.setDataSource(ds);

        // check
        assertFalse(optimizer.isReady());

        // set calibrator
        final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();
        optimizer.setCalibrator(calibrator);

        // check
        assertTrue(optimizer.isReady());

        // unset quality score mapper
        optimizer.setQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());

        // set quality score mapper
        optimizer.setQualityScoreMapper(new DefaultMagnetometerQualityScoreMapper());

        // check
        assertTrue(optimizer.isReady());

        // unset MSE optimizer
        optimizer.setMseOptimizer(null);

        // check
        assertFalse(optimizer.isReady());
    }

    @Test
    void testGetSetCalibrator() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getCalibrator());

        // set a new value
        final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();

        optimizer.setCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getCalibrator());

        // Force IllegalArgumentException
        final var wrongCalibrator = new KnownFrameMagnetometerNonLinearLeastSquaresCalibrator();
        assertThrows(IllegalArgumentException.class, () -> optimizer.setCalibrator(wrongCalibrator));
    }

    @Test
    void testGetSetQualityScoreMapper() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getQualityScoreMapper());
        assertEquals(DefaultMagnetometerQualityScoreMapper.class, optimizer.getQualityScoreMapper().getClass());

        // set a new value
        //noinspection unchecked
        final QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> qualityScoreMapper =
                mock(QualityScoreMapper.class);
        optimizer.setQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getQualityScoreMapper());
    }

    @Test
    void testGetSetThresholdFactorRange() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(MagnetometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);

        // set new values
        optimizer.setThresholdFactorRange(0.0, 1.0);

        // check
        assertEquals(0.0, optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(1.0, optimizer.getMaxThresholdFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setThresholdFactorRange(-1.0, 1.0));
        assertThrows(IllegalArgumentException.class, () -> optimizer.setThresholdFactorRange(1.0, -1.0));
        assertThrows(IllegalArgumentException.class, () -> optimizer.setThresholdFactorRange(2.0, 1.0));
    }

    @Test
    void testGetSetDataSource() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNull(optimizer.getDataSource());

        // set a new value
        final var ds = mock(MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource.class);

        optimizer.setDataSource(ds);

        // check
        assertSame(ds, optimizer.getDataSource());
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);

        // set a new value
        final var timeInterval = 0.01;
        optimizer.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, optimizer.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeIntervalAsTime() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        final var timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set a new value
        final var timeInterval2 = new Time(0.01, TimeUnit.SECOND);
        optimizer.setTimeInterval(timeInterval2);

        // check
        final var timeInterval3 = optimizer.getTimeIntervalAsTime();
        final var timeInterval4 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);

        // Force IllegalArgumentException
        final var wrongTimeInterval = new Time(-1.0, TimeUnit.SECOND);
        assertThrows(IllegalArgumentException.class, () -> optimizer.setTimeInterval(wrongTimeInterval));
    }

    @Test
    void testGetSetMinStaticSamples() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());

        // set a new value
        final var minStaticSamples = 50;
        optimizer.setMinStaticSamples(minStaticSamples);

        // check
        assertEquals(minStaticSamples, optimizer.getMinStaticSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setMinStaticSamples(1));
    }

    @Test
    void testGetSetMaxDynamicSamples() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());

        // set a new value
        final var maxDynamicSamples = 500;
        optimizer.setMaxDynamicSamples(maxDynamicSamples);

        // check
        assertEquals(maxDynamicSamples, optimizer.getMaxDynamicSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setMaxDynamicSamples(1));
    }

    @Test
    void testGetSetWindowSize() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());

        // set a new value
        final var windowSize = 51;
        optimizer.setWindowSize(windowSize);

        // check
        assertEquals(windowSize, optimizer.getWindowSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setWindowSize(1));
    }

    @Test
    void testGetSetInitialStaticSamples() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());

        // set a new value
        final var initialStaticSamples = 100;
        optimizer.setInitialStaticSamples(initialStaticSamples);

        // check
        assertEquals(initialStaticSamples, optimizer.getInitialStaticSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setInitialStaticSamples(1));
    }

    @Test
    void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);

        // set a new value
        final var instantaneousNoiseLevelFactor = 3.0;
        optimizer.setInstantaneousNoiseLevelFactor(instantaneousNoiseLevelFactor);

        // check
        assertEquals(instantaneousNoiseLevelFactor, optimizer.getInstantaneousNoiseLevelFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setInstantaneousNoiseLevelFactor(0.0));
    }

    @Test
    void testGetSetBaseNoiseLevelAbsoluteThreshold() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // set a new value
        final var baseNoiseLevelAbsoluteThreshold = 1e-5;
        optimizer.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);

        // check
        assertEquals(baseNoiseLevelAbsoluteThreshold, optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setBaseNoiseLevelAbsoluteThreshold(0.0));
    }

    @Test
    void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        final var acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set a new value
        final var baseNoiseLevelAbsoluteThreshold = 1e-5;
        final var acceleration2 = new Acceleration(baseNoiseLevelAbsoluteThreshold,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        optimizer.setBaseNoiseLevelAbsoluteThreshold(acceleration2);

        // check
        final var acceleration3 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final var acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration4);

        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getListener());

        // set a new value
        optimizer.setListener(this);

        // check
        assertSame(this, optimizer.getListener());
    }

    @Test
    void testGetSetProgressDelta() throws LockedException {
        final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer();

        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // set a new value
        optimizer.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, optimizer.getProgressDelta(), 0.0f);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> optimizer.setProgressDelta(2.0f));
    }

    @Test
    void testOptimizeGeneralWithNoise() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, IntervalDetectorThresholdFactorOptimizerException, CalibrationException,
            IOException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            bodyKinematicsAndMagneticFluxDensities.clear();
            generatorMeasurements.clear();

            // generate measurements
            final var randomizer = new UniformRandomizer();

            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(false, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition, MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(false);
            calibrator.setTime(timestamp);

            // create optimizer
            final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(dataSource,
                    calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, start);
            assertEquals(0, end);
            assertEquals(0.0f, progress, 0.0f);

            final var thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, start);
            assertEquals(1, end);
            assertTrue(progress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final var accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final var accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final var threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final var optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final var optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using the estimated threshold factor
            // on generator that optimizes calibration
            final var generator = new MagnetometerMeasurementsGenerator(generatorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (final var bodyKinematics : bodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from a generator that used the optimal threshold factor
            calibrator.setMeasurements(generatorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testOptimizeCommonAxisSmallNoiseOnlyRotation() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException,
            IOException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            bodyKinematicsAndMagneticFluxDensities.clear();
            generatorMeasurements.clear();

            // generate measurements
            final var randomizer = new UniformRandomizer();

            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(false, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition, SMALL_MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(true);
            calibrator.setTime(timestamp);

            // create optimizer
            final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(dataSource,
                    calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, start);
            assertEquals(0, end);
            assertEquals(0.0f, progress, 0.0f);

            final double thresholdFactor;
            try {
                thresholdFactor = optimizer.optimize();
            } catch (final IntervalDetectorThresholdFactorOptimizerException e) {
                continue;
            }

            // check optimization results
            assertEquals(1, start);
            assertEquals(1, end);
            assertTrue(progress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final var accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final var accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final var threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final var optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final var optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using the estimated threshold factor
            // on generator that optimizes calibration
            final var generator = new MagnetometerMeasurementsGenerator(generatorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (final var bodyKinematics : bodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from a generator that used the optimal threshold factor
            calibrator.setMeasurements(generatorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testOptimizeCommonAxisSmallNoiseWithRotationAndPositionChange() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException, CalibrationException,
            IOException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            bodyKinematicsAndMagneticFluxDensities.clear();
            generatorMeasurements.clear();

            // generate measurements
            final var randomizer = new UniformRandomizer();

            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var numMeasurements = KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronCommonAxis();
            assertNotNull(mm);
            final var timestamp = new Date(createTimestamp(randomizer));
            final var nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(true, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition, SMALL_MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            final var calibrator = new KnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(true);
            calibrator.setTime(timestamp);

            // create optimizer
            final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(dataSource,
                    calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, start);
            assertEquals(0, end);
            assertEquals(0.0f, progress, 0.0f);

            final double thresholdFactor;
            try {
                thresholdFactor = optimizer.optimize();
            } catch (final IntervalDetectorThresholdFactorOptimizerException e) {
                continue;
            }

            // check optimization results
            assertEquals(1, start);
            assertEquals(1, end);
            assertTrue(progress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final var accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final var accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final var threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final var optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final var optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using the estimated threshold factor
            // on generator that optimizes calibration
            final var generator = new MagnetometerMeasurementsGenerator(generatorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (final var bodyKinematics : bodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from a generator that used the optimal threshold factor
            calibrator.setMeasurements(generatorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testOptimizeRobustCalibrator() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            LockedException, NotReadyException, IntervalDetectorThresholdFactorOptimizerException, CalibrationException,
            IOException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            bodyKinematicsAndMagneticFluxDensities.clear();
            generatorMeasurements.clear();

            // generate measurements
            final var randomizer = new UniformRandomizer();

            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var numMeasurements = 3 * KnownPositionAndInstantMagnetometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            final var hardIron = Matrix.newFromArray(generateHardIron(randomizer));
            final var mm = generateSoftIronGeneral();
            assertNotNull(mm);
            final var timestamp = new Date(createTimestamp(randomizer));
            var nedPosition = createPosition(randomizer);
            assertTrue(generateBodyKinematicsAndMagneticFluxDensity(false, hardIron, mm, accelNoiseRootPSD,
                    gyroNoiseRootPSD, numMeasurements, timestamp, nedPosition, MAGNETOMETER_NOISE_STD));

            // configure calibrator and data source
            final var calibrator = new PROMedSRobustKnownPositionAndInstantMagnetometerCalibrator();
            calibrator.setPosition(nedPosition);
            calibrator.setCommonAxisUsed(false);
            calibrator.setTime(timestamp);

            // create optimizer
            final var optimizer = new BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer(dataSource,
                    calibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, start);
            assertEquals(0, end);
            assertEquals(0.0f, progress, 0.0f);

            final var thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, start);
            assertEquals(1, end);
            assertTrue(progress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final var accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final var accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final var threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final var threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertNotNull(optimizer.getEstimatedHardIron());

            final var optimalHardIron = Matrix.newFromArray(optimizer.getEstimatedHardIron());
            final var optimalMm = optimizer.getEstimatedMm();

            assertNotNull(optimalHardIron);
            assertNotNull(optimalMm);

            if (!hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(optimalHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(optimalMm, VERY_LARGE_ABSOLUTE_ERROR));

            // generate measurements for calibrator using the estimated threshold factor
            // on generator that optimizes calibration
            final var generator = new MagnetometerMeasurementsGenerator(generatorListener);

            generator.setThresholdFactor(thresholdFactor);

            for (final var bodyKinematics : bodyKinematicsAndMagneticFluxDensities) {
                assertTrue(generator.process(bodyKinematics));
            }

            // use generated measurements from a generator that used the optimal threshold factor
            calibrator.setMeasurements(generatorMeasurements);

            // calibrate
            calibrator.calibrate();

            // check calibration result
            final var estimatedHardIron = calibrator.getEstimatedHardIronAsMatrix();
            final var estimatedMm = calibrator.getEstimatedMm();

            if (!hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }

            assertTrue(hardIron.equals(estimatedHardIron, LARGE_ABSOLUTE_ERROR));
            assertTrue(mm.equals(estimatedMm, VERY_LARGE_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onOptimizeStart(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematicsAndMagneticFluxDensity,
                    MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        start++;
        checkLocked((BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeEnd(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematicsAndMagneticFluxDensity,
                    MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        end++;
        checkLocked((BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeProgressChange(
            final IntervalDetectorThresholdFactorOptimizer<BodyKinematicsAndMagneticFluxDensity,
                    MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> optimizer,
            final float progress) {
        assertTrue(progress >= 0.0f);
        assertTrue(progress <= 1.0f);
        assertTrue(progress > this.progress);
        if (this.progress == 0.0f) {
            checkLocked((BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer) optimizer);
        }
        this.progress = progress;
    }

    private static void checkLocked(final BracketedMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer) {
        assertTrue(optimizer.isRunning());
        assertThrows(LockedException.class, () -> optimizer.setDataSource(null));
        assertThrows(LockedException.class, () -> optimizer.setListener(null));
        assertThrows(LockedException.class, () -> optimizer.setCalibrator(null));
        assertThrows(LockedException.class, () -> optimizer.setQualityScoreMapper(null));
        assertThrows(LockedException.class, () -> optimizer.setThresholdFactorRange(0.0, 1.0));
        assertThrows(LockedException.class, () -> optimizer.setTimeInterval(0.01));
        assertThrows(LockedException.class, () -> optimizer.setTimeInterval(null));
        assertThrows(LockedException.class, () -> optimizer.setMinStaticSamples(0));
        assertThrows(LockedException.class, () -> optimizer.setMaxDynamicSamples(0));
        assertThrows(LockedException.class, () -> optimizer.setWindowSize(0));
        assertThrows(LockedException.class, () -> optimizer.setInitialStaticSamples(0));
        assertThrows(LockedException.class, () -> optimizer.setInstantaneousNoiseLevelFactor(0.0));
        assertThrows(LockedException.class, () -> optimizer.setBaseNoiseLevelAbsoluteThreshold(0.0));
        assertThrows(LockedException.class, () -> optimizer.setBaseNoiseLevelAbsoluteThreshold(null));
        assertThrows(LockedException.class, () -> optimizer.setMseOptimizer(null));
        assertThrows(LockedException.class, optimizer::optimize);
    }

    private void reset() {
        start = 0;
        end = 0;
        progress = 0.0f;
    }

    private boolean generateBodyKinematicsAndMagneticFluxDensity(
            final boolean changePosition, Matrix hardIron, final Matrix mm, final double accelNoiseRootPSD,
            final double gyroNoiseRootPSD, final int numMeasurements, final Date timestamp,
            final NEDPosition nedPosition, final double magnetometerNoiseStd) throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, IOException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var wmmEstimator = new WMMEarthMagneticFluxDensityEstimator();

        final var noiseRandomizer = new GaussianRandomizer(0.0, magnetometerNoiseStd);

        var cnb = generateBodyC(randomizer);
        var nedC = cnb.inverseAndReturnNew();

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame, ecefFrame);

        final var generator = new MagnetometerMeasurementsGenerator();

        // generate initial static samples
        final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        final var random = new Random();
        generateStaticSamples(generator, initialStaticSamples, trueKinematics, errors, hardIron, mm, wmmEstimator,
                random, timestamp, nedPosition, cnb, noiseRandomizer);

        final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final var dynamicPeriodLength = 2 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        for (var i = 0; i < numMeasurements; i++) {
            nedFrame.getPosition(nedPosition);
            nedC = nedFrame.getCoordinateTransformation();
            cnb = nedC.inverseAndReturnNew();

            // generate static samples
            generateStaticSamples(generator, staticPeriodLength, trueKinematics, errors, hardIron, mm, wmmEstimator,
                    random, timestamp, nedPosition, cnb, noiseRandomizer);

            // generate dynamic samples
            generateDynamicSamples(generator, dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame,
                    errors, hardIron, mm, wmmEstimator, random, timestamp, nedPosition, cnb, noiseRandomizer,
                    changePosition);
        }

        return generator.getStatus() != TriadStaticIntervalDetector.Status.FAILED;
    }

    private static BodyMagneticFluxDensity generateB(
            final double[] hardIron, final Matrix softIron,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator,
            final GaussianRandomizer noiseRandomizer,
            final Date timestamp,
            final NEDPosition position,
            final CoordinateTransformation cnb) {

        final var earthB = wmmEstimator.estimate(position, timestamp);

        final var truthMagnetic = BodyMagneticFluxDensityEstimator.estimate(earthB, cnb);
        final var measuredMagnetic = generateMeasuredMagneticFluxDensity(truthMagnetic, hardIron, softIron);

        if (noiseRandomizer != null) {
            measuredMagnetic.setBx(measuredMagnetic.getBx() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBy(measuredMagnetic.getBy() + noiseRandomizer.nextDouble());
            measuredMagnetic.setBz(measuredMagnetic.getBz() + noiseRandomizer.nextDouble());
        }

        return measuredMagnetic;
    }

    private static CoordinateTransformation generateBodyC(final UniformRandomizer randomizer) {
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw1 = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));

        return new CoordinateTransformation(roll, pitch, yaw1, FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME);
    }

    private static BodyMagneticFluxDensity generateMeasuredMagneticFluxDensity(
            final BodyMagneticFluxDensity input, final double[] hardIron, final Matrix softIron) {
        return BodyMagneticFluxDensityGenerator.generate(input, hardIron, softIron);
    }

    private static double[] generateHardIron(final UniformRandomizer randomizer) {
        final var result = new double[BodyMagneticFluxDensity.COMPONENTS];
        randomizer.fill(result, MIN_HARD_IRON, MAX_HARD_IRON);
        return result;
    }

    private static Matrix generateSoftIronCommonAxis() {
        final var mm = generateSoftIronGeneral();
        assertNotNull(mm);

        for (var col = 0; col < mm.getColumns(); col++) {
            for (var row = 0; row < mm.getRows(); row++) {
                if (row > col) {
                    mm.setElementAt(row, col, 0.0);
                }
            }
        }
        return mm;
    }

    private static Matrix generateSoftIronGeneral() {
        try {
            return Matrix.createWithUniformRandomValues(BodyMagneticFluxDensity.COMPONENTS,
                    BodyMagneticFluxDensity.COMPONENTS, MIN_SOFT_IRON, MAX_SOFT_IRON);
        } catch (final WrongSizeException ignore) {
            // never happens
            return null;
        }
    }

    private static NEDPosition createPosition(final UniformRandomizer randomizer) {
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT_METERS, MAX_HEIGHT_METERS);

        return new NEDPosition(latitude, longitude, height);
    }

    private static long createTimestamp(final UniformRandomizer randomizer) {
        return randomizer.nextLong(START_TIMESTAMP_MILLIS, END_TIMESTAMP_MILLIS);
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix generateMaCommonAxis() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        final var tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }

    private static double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }

    private void generateStaticSamples(
            final MagnetometerMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final IMUErrors errors, final Matrix hardIron, final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final Random random, final Date timestamp,
            final NEDPosition nedPosition, final CoordinateTransformation cnb, final GaussianRandomizer noiseRandomizer)
            throws LockedException {

        for (var i = 0; i < numSamples; i++) {
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random);

            final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final var kb = new BodyKinematicsAndMagneticFluxDensity(measuredKinematics, b);
            assertTrue(generator.process(kb));

            bodyKinematicsAndMagneticFluxDensities.add(kb);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void generateDynamicSamples(
            final MagnetometerMeasurementsGenerator generator, final int numSamples,
            final BodyKinematics trueKinematics, final UniformRandomizer randomizer, final ECEFFrame ecefFrame,
            final NEDFrame nedFrame, final IMUErrors errors, final Matrix hardIron, final Matrix mm,
            final WMMEarthMagneticFluxDensityEstimator wmmEstimator, final Random random, final Date timestamp,
            final NEDPosition nedPosition, final CoordinateTransformation cnb, final GaussianRandomizer noiseRandomizer,
            final boolean changePosition) throws InvalidSourceAndDestinationFrameTypeException, LockedException {

        final var deltaX = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaY = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaZ = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var nedC = nedFrame.getCoordinateTransformation();

        final var roll = nedC.getRollEulerAngle();
        final var pitch = nedC.getPitchEulerAngle();
        final var yaw = nedC.getYawEulerAngle();

        final var oldNedFrame = new NEDFrame(nedFrame);
        final var newNedFrame = new NEDFrame();
        final var oldEcefFrame = new ECEFFrame(ecefFrame);
        final var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        for (var i = 0; i < numSamples; i++) {
            final var newRoll = oldRoll + deltaRoll;
            final var newPitch = oldPitch + deltaPitch;
            final var newYaw = oldYaw + deltaYaw;
            final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final var newEcefX = oldEcefX + deltaX;
            final var newEcefY = oldEcefY + deltaY;
            final var newEcefZ = oldEcefZ + deltaZ;

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame,
                    trueKinematics);

            // add error to true kinematics
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random);

            final var b = generateB(hardIron.getBuffer(), mm, wmmEstimator, noiseRandomizer, timestamp, nedPosition,
                    cnb);
            final var kb = new BodyKinematicsAndMagneticFluxDensity(measuredKinematics, b);
            assertTrue(generator.process(kb));

            bodyKinematicsAndMagneticFluxDensities.add(kb);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after the dynamic sequence finishes, update true kinematics for a
        // static sequence at the current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame, trueKinematics);
    }
}
