/*
 * Copyright (C) 2020 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.ECEFGravity;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsGenerator;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.IMUErrors;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownFrameAccelerometerNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.PROMedSRobustKnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.generators.MeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.KnownFrameGyroscopeNonLinearLeastSquaresCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.PROMedSRobustEasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.noise.WindowedTriadNoiseEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeUnit;
import org.junit.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;

public class ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerTest implements
        IntervalDetectorThresholdFactorOptimizerListener<TimedBodyKinematics,
                AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource> {

    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 5e-4;

    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-2;

    private static final double LARGE_ABSOLUTE_ERROR = 5e-3;

    private static final double SMALL_ABSOLUTE_ERROR = 1e-6;

    private static final double SMALL_ROOT_PSD = 1e-15;

    private static final int NUM_MEASUREMENTS = 10;

    private final List<TimedBodyKinematics> mTimedBodyKinematics = new ArrayList<>();

    private final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource mDataSource =
            new AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource() {

                @Override
                public int count() {
                    return mTimedBodyKinematics.size();
                }

                @Override
                public TimedBodyKinematics getAt(final int index) {
                    return mTimedBodyKinematics.get(index);
                }
            };

    private final AccelerometerAndGyroscopeMeasurementsGeneratorListener mGeneratorListener =
            new AccelerometerAndGyroscopeMeasurementsGeneratorListener() {
                @Override
                public void onInitializationStarted(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onInitializationCompleted(
                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                        final double accelerometerBaseNoiseLevel) {
                    // not used
                }

                @Override
                public void onError(
                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                        final TriadStaticIntervalDetector.ErrorReason reason) {
                    // not used
                }

                @Override
                public void onStaticIntervalDetected(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalDetected(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onStaticIntervalSkipped(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onDynamicIntervalSkipped(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                    // not used
                }

                @Override
                public void onGeneratedAccelerometerMeasurement(
                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                        final StandardDeviationBodyKinematics measurement) {
                    mAccelerometerGeneratorMeasurements.add(measurement);
                }

                @Override
                public void onGeneratedGyroscopeMeasurement(
                        final AccelerometerAndGyroscopeMeasurementsGenerator generator,
                        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
                    mGyroscopeGeneratorMeasurements.add(sequence);
                }

                @Override
                public void onReset(final AccelerometerAndGyroscopeMeasurementsGenerator generator) {
                    // not used
                }
            };

    private final List<StandardDeviationBodyKinematics> mAccelerometerGeneratorMeasurements = new ArrayList<>();

    private final List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> mGyroscopeGeneratorMeasurements =
            new ArrayList<>();

    private int mStart;

    private int mEnd;

    private float mProgress;

    @Test
    public void testConstructor1() {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertNull(optimizer.getAccelerometerCalibrator());
        assertNull(optimizer.getGyroscopeCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class, optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerAndGyroscopeMseRule.class, optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
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
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);
    }

    @Test
    public void testConstructor2() {
        final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource.class);

        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(dataSource);

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertNull(optimizer.getAccelerometerCalibrator());
        assertNull(optimizer.getGyroscopeCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class, optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerAndGyroscopeMseRule.class, optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertSame(dataSource, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
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
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);
    }

    @Test
    public void testConstructor3() {
        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();

        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(accelerometerCalibrator,
                        gyroscopeCalibrator);

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertSame(accelerometerCalibrator, optimizer.getAccelerometerCalibrator());
        assertSame(gyroscopeCalibrator, optimizer.getGyroscopeCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class, optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerAndGyroscopeMseRule.class, optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertFalse(optimizer.isReady());
        assertNull(optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
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
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                        new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(), gyroscopeCalibrator));
        assertThrows(IllegalArgumentException.class,
                () -> new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(
                        accelerometerCalibrator, new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator()));
    }

    @Test
    public void testConstructor4() {
        final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource.class);
        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();

        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(dataSource,
                        accelerometerCalibrator, gyroscopeCalibrator);

        // check default values
        assertEquals(ExhaustiveAccelerometerIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);
        assertSame(accelerometerCalibrator, optimizer.getAccelerometerCalibrator());
        assertSame(gyroscopeCalibrator, optimizer.getGyroscopeCalibrator());
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class, optimizer.getGyroscopeQualityScoreMapper().getClass());
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerAndGyroscopeMseRule.class, optimizer.getMseRule().getClass());
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MIN_THRESHOLD_FACTOR,
                optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_MAX_THRESHOLD_FACTOR,
                optimizer.getMaxThresholdFactor(), 0.0);
        assertTrue(optimizer.isReady());
        assertSame(dataSource, optimizer.getDataSource());
        assertFalse(optimizer.isRunning());
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final Time timeInterval2 = new Time(1.0, TimeUnit.DAY);
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
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());
        final Acceleration acceleration2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration2);
        assertEquals(acceleration1, acceleration2);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
        final Acceleration acceleration3 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
        assertEquals(0.0, acceleration3.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration3.getUnit());
        final Acceleration acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(acceleration4);
        assertEquals(acceleration3, acceleration4);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getGyroscopeBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelPsd(), 0.0);
        assertEquals(0.0, optimizer.getAccelerometerBaseNoiseLevelRootPsd(), 0.0);
        assertEquals(0.0, optimizer.getThreshold(), 0.0);
        final Acceleration acceleration5 = optimizer.getThresholdAsMeasurement();
        assertEquals(0.0, acceleration5.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration5.getUnit());
        final Acceleration acceleration6 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getThresholdAsMeasurement(acceleration6);
        assertEquals(acceleration5, acceleration6);
        assertNull(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedAccelerometerBiasFxVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFyVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiasFzVariance());
        assertNull(optimizer.getEstimatedAccelerometerBiases());
        assertNull(optimizer.getEstimatedAccelerometerMa());
        assertNull(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm());
        assertNull(optimizer.getEstimatedGyroscopeBiasXVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasYVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiasZVariance());
        assertNull(optimizer.getEstimatedGyroscopeBiases());
        assertNull(optimizer.getEstimatedGyroscopeMg());
        assertNull(optimizer.getEstimatedGyroscopeGg());
        assertEquals(0.0, optimizer.getMinMse(), 0.0);
        assertEquals(0.0, optimizer.getOptimalThresholdFactor(), 0.0);
        assertNull(optimizer.getListener());
        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class,
                () -> new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(dataSource,
                        new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator(), gyroscopeCalibrator));
        assertThrows(IllegalArgumentException.class,
                () -> new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(dataSource,
                        accelerometerCalibrator, new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator()));
    }

    @Test
    public void testGetSetThresholdFactorStep() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(ExhaustiveGyroscopeIntervalDetectorThresholdFactorOptimizer.DEFAULT_STEP,
                optimizer.getThresholdFactorStep(), 0.0);

        // set new value
        optimizer.setThresholdFactorStep(2.0);

        // check
        assertEquals(2.0, optimizer.getThresholdFactorStep(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setThresholdFactorStep(0.0));
    }

    @Test
    public void testGetSetAccelerometerCalibrator() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getAccelerometerCalibrator());

        // set new value
        final KnownGravityNormAccelerometerCalibrator calibrator = new KnownGravityNormAccelerometerCalibrator();

        optimizer.setAccelerometerCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getAccelerometerCalibrator());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setAccelerometerCalibrator(
                new KnownFrameAccelerometerNonLinearLeastSquaresCalibrator()));
    }

    @Test
    public void testGetSetGyroscopeCalibrator() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getGyroscopeCalibrator());

        // set new value
        final EasyGyroscopeCalibrator calibrator = new EasyGyroscopeCalibrator();

        optimizer.setGyroscopeCalibrator(calibrator);

        // check
        assertSame(calibrator, optimizer.getGyroscopeCalibrator());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setGyroscopeCalibrator(
                new KnownFrameGyroscopeNonLinearLeastSquaresCalibrator()));
    }

    @Test
    public void testGetSetAccelerometerQualityScoreMapper() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getAccelerometerQualityScoreMapper());
        assertEquals(DefaultAccelerometerQualityScoreMapper.class,
                optimizer.getAccelerometerQualityScoreMapper().getClass());

        // set new value
        //noinspection unchecked
        final QualityScoreMapper<StandardDeviationBodyKinematics> qualityScoreMapper = mock(QualityScoreMapper.class);
        optimizer.setAccelerometerQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getAccelerometerQualityScoreMapper());
    }

    @Test
    public void testGetSetGyroscopeQualityScoreMapper() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getGyroscopeQualityScoreMapper());
        assertEquals(DefaultGyroscopeQualityScoreMapper.class, optimizer.getGyroscopeQualityScoreMapper().getClass());

        // set new value
        //noinspection unchecked
        final QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> qualityScoreMapper =
                mock(QualityScoreMapper.class);
        optimizer.setGyroscopeQualityScoreMapper(qualityScoreMapper);

        // check
        assertSame(qualityScoreMapper, optimizer.getGyroscopeQualityScoreMapper());
    }

    @Test
    public void testGetSetMseRule() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNotNull(optimizer.getMseRule());
        assertEquals(DefaultAccelerometerAndGyroscopeMseRule.class, optimizer.getMseRule().getClass());

        // set new value
        final AccelerometerAndGyroscopeMseRule rule = mock(AccelerometerAndGyroscopeMseRule.class);
        optimizer.setMseRule(rule);

        // check
        assertSame(rule, optimizer.getMseRule());
    }

    @Test
    public void testGetSetThresholdFactorRange() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertEquals(ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer
                        .DEFAULT_MIN_THRESHOLD_FACTOR, optimizer.getMinThresholdFactor(), 0.0);
        assertEquals(ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer
                        .DEFAULT_MAX_THRESHOLD_FACTOR, optimizer.getMaxThresholdFactor(), 0.0);

        // set new value
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
    public void testGetSetDataSource() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default values
        assertNull(optimizer.getDataSource());

        // set new value
        final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource.class);

        optimizer.setDataSource(dataSource);

        // check
        assertSame(dataSource, optimizer.getDataSource());
    }

    @Test
    public void testIsReady() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertFalse(optimizer.isReady());

        // set data source
        final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
                mock(AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource.class);
        optimizer.setDataSource(dataSource);

        // check
        assertFalse(optimizer.isReady());

        // set accelerometer calibrator
        final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                new KnownGravityNormAccelerometerCalibrator();
        optimizer.setAccelerometerCalibrator(accelerometerCalibrator);

        // check
        assertFalse(optimizer.isReady());

        // set gyroscope calibrator
        final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
        optimizer.setGyroscopeCalibrator(gyroscopeCalibrator);

        // check
        assertTrue(optimizer.isReady());

        // unset accelerometer quality score mapper
        optimizer.setAccelerometerQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());

        // set quality score mapper
        optimizer.setAccelerometerQualityScoreMapper(new DefaultAccelerometerQualityScoreMapper());

        // check
        assertTrue(optimizer.isReady());

        // unset gyroscope quality score mapper
        optimizer.setGyroscopeQualityScoreMapper(null);

        // check
        assertFalse(optimizer.isReady());
    }

    @Test
    public void testGetSetTimeInterval() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, optimizer.getTimeInterval(), 0.0);

        // set new value
        final double timeInterval = 0.01;
        optimizer.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, optimizer.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setTimeInterval(-1.0));
    }

    @Test
    public void testGetSetTimeIntervalAsTime() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        final Time timeInterval1 = optimizer.getTimeIntervalAsTime();
        assertEquals(WindowedTriadNoiseEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(),
                0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set new value
        final Time timeInterval2 = new Time(0.01, TimeUnit.SECOND);
        optimizer.setTimeInterval(timeInterval2);

        // check
        final Time timeInterval3 = optimizer.getTimeIntervalAsTime();
        final Time timeInterval4 = new Time(1.0, TimeUnit.DAY);
        optimizer.getTimeIntervalAsTime(timeInterval4);

        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setTimeInterval(
                new Time(-1.0, TimeUnit.SECOND)));
    }

    @Test
    public void testGetSetMinStaticSamples() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MIN_STATIC_SAMPLES, optimizer.getMinStaticSamples());

        // set new value
        final int minStaticSamples = 50;
        optimizer.setMinStaticSamples(minStaticSamples);

        // check
        assertEquals(minStaticSamples, optimizer.getMinStaticSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setMinStaticSamples(1));
    }

    @Test
    public void testGetSetMaxDynamicSamples() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(MeasurementsGenerator.DEFAULT_MAX_DYNAMIC_SAMPLES, optimizer.getMaxDynamicSamples());

        // set new value
        final int maxDynamicSamples = 500;
        optimizer.setMaxDynamicSamples(maxDynamicSamples);

        // check
        assertEquals(maxDynamicSamples, optimizer.getMaxDynamicSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setMaxDynamicSamples(1));
    }

    @Test
    public void testGetSetWindowSize() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE, optimizer.getWindowSize());

        // set new value
        final int windowSize = 51;
        optimizer.setWindowSize(windowSize);

        // check
        assertEquals(windowSize, optimizer.getWindowSize());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setWindowSize(1));
    }

    @Test
    public void testGetSetInitialStaticSamples() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES, optimizer.getInitialStaticSamples());

        // set new value
        final int initialStaticSamples = 100;
        optimizer.setInitialStaticSamples(initialStaticSamples);

        // check
        assertEquals(initialStaticSamples, optimizer.getInitialStaticSamples());

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setInitialStaticSamples(1));
    }

    @Test
    public void testGetSetInstantaneousNoiseLevelFactor() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_INSTANTANEOUS_NOISE_LEVEL_FACTOR,
                optimizer.getInstantaneousNoiseLevelFactor(), 0.0);

        // set new value
        final double instantaneousNoiseLevelFactor = 3.0;
        optimizer.setInstantaneousNoiseLevelFactor(instantaneousNoiseLevelFactor);

        // check
        assertEquals(instantaneousNoiseLevelFactor, optimizer.getInstantaneousNoiseLevelFactor(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setInstantaneousNoiseLevelFactor(0.0));
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThreshold() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // set new value
        final double baseNoiseLevelAbsoluteThreshold = 1e-5;
        optimizer.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);

        // check
        assertEquals(baseNoiseLevelAbsoluteThreshold, optimizer.getBaseNoiseLevelAbsoluteThreshold(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setBaseNoiseLevelAbsoluteThreshold(0.0));
    }

    @Test
    public void testGetSetBaseNoiseLevelAbsoluteThresholdAsMeasurement() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        final Acceleration acceleration1 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        assertEquals(TriadStaticIntervalDetector.DEFAULT_BASE_NOISE_LEVEL_ABSOLUTE_THRESHOLD,
                acceleration1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, acceleration1.getUnit());

        // set new value
        final double baseNoiseLevelAbsoluteThreshold = 1e-5;
        final Acceleration acceleration2 = new Acceleration(baseNoiseLevelAbsoluteThreshold,
                AccelerationUnit.METERS_PER_SQUARED_SECOND);
        optimizer.setBaseNoiseLevelAbsoluteThreshold(acceleration2);

        // check
        final Acceleration acceleration3 = optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
        final Acceleration acceleration4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        optimizer.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(acceleration4);

        assertEquals(acceleration2, acceleration3);
        assertEquals(acceleration2, acceleration4);
    }

    @Test
    public void testGetSetListener() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        // check default value
        assertNull(optimizer.getListener());

        // set new value
        optimizer.setListener(this);

        // check
        assertSame(this, optimizer.getListener());
    }

    @Test
    public void testGetSetProgressDelta() throws LockedException {
        final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer();

        assertEquals(IntervalDetectorThresholdFactorOptimizer.DEFAULT_PROGRESS_DELTA, optimizer.getProgressDelta(),
                0.0);

        // set new value
        optimizer.setProgressDelta(0.5f);

        // check
        assertEquals(0.5f, optimizer.getProgressDelta(), 0.0f);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> optimizer.setProgressDelta(-1.0f));
        assertThrows(IllegalArgumentException.class, () -> optimizer.setProgressDelta(2.0f));
    }

    @Test
    public void testOptimizeMaCommonAxisWithNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double accelNoiseRootPSD = getAccelNoiseRootPSD();
        final double gyroNoiseRootPSD = getGyroNoiseRootPSD();

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematics.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            generateBodyKinematics(nedFrame, ecefFrame, false, ma, accelNoiseRootPSD, gyroNoiseRootPSD,
                    numSequences, numMeasurements);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            // create optimizer
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(mDataSource,
                            accelerometerCalibrator, gyroscopeCalibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration result
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeGeneralNoGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematics.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            generateBodyKinematics(nedFrame, ecefFrame, false, ma, SMALL_ROOT_PSD, gyroNoiseRootPSD,
                    numSequences, numMeasurements);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), false, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            // create optimizer
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(mDataSource,
                            accelerometerCalibrator, gyroscopeCalibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration result
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeMaCommonAxisNoGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematics.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            generateBodyKinematics(nedFrame, ecefFrame, false, ma, SMALL_ROOT_PSD, gyroNoiseRootPSD,
                    numSequences, numMeasurements);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            // create optimizer
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(mDataSource,
                            accelerometerCalibrator, gyroscopeCalibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration result
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, LARGE_ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeGeneralGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaGeneral();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematics.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            generateBodyKinematics(nedFrame, ecefFrame, false, ma, SMALL_ROOT_PSD, gyroNoiseRootPSD,
                    numSequences, numMeasurements);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), false, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(true);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            // create optimizer
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(mDataSource,
                            accelerometerCalibrator, gyroscopeCalibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration result
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeMaCommonAxisGDependentCrossBiasesWithSmallNoise() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematics.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS_AND_CROSS_BIASES;
            generateBodyKinematics(nedFrame, ecefFrame, false, ma, SMALL_ROOT_PSD, gyroNoiseRootPSD,
                    numSequences, NUM_MEASUREMENTS);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(true);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            // create optimizer
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(mDataSource,
                            accelerometerCalibrator, gyroscopeCalibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, SMALL_ABSOLUTE_ERROR));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration result
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            if (!ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, SMALL_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, SMALL_ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeMaCommonAxisNoGDependentCrossBiasesWithSmallNoiseRotationAndPositionChange()
            throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException, LockedException,
            NotReadyException, IntervalDetectorThresholdFactorOptimizerException, InvalidRotationMatrixException,
            RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematics.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
            generateBodyKinematics(nedFrame, ecefFrame, true, ma, SMALL_ROOT_PSD, gyroNoiseRootPSD,
                    numSequences, numMeasurements);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new KnownGravityNormAccelerometerCalibrator(gravity.getNorm(), true, initialBa,
                            initialMa);

            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final EasyGyroscopeCalibrator gyroscopeCalibrator = new EasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            // create optimizer
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(mDataSource,
                            accelerometerCalibrator, gyroscopeCalibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration result
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            if (!ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, VERY_LARGE_ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    public void testOptimizeRobustCalibrators() throws WrongSizeException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, NotReadyException,
            IntervalDetectorThresholdFactorOptimizerException, InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix ma = generateMaCommonAxis();
        final Matrix mg = generateMg();
        final Matrix gg = new Matrix(3, 3);

        final double gyroNoiseRootPSD = 0.0;

        int numValid = 0;
        for (int t = 0; t < TIMES; t++) {
            mTimedBodyKinematics.clear();
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();

            // generate measurements

            final NEDFrame nedFrame = generateFrame();
            final ECEFFrame ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final int numSequences = 3 * EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
            final int numMeasurements = 3 * NUM_MEASUREMENTS;
            generateBodyKinematics(nedFrame, ecefFrame, false, ma, SMALL_ROOT_PSD, gyroNoiseRootPSD,
                    numSequences, numMeasurements);

            final AccelerometerAndGyroscopeMeasurementsGenerator generator =
                    new AccelerometerAndGyroscopeMeasurementsGenerator(mGeneratorListener);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // as an initial value for gyroscope bias we can use the average
            // gyroscope values during initialization. A more accurate initial
            // guess for bias could be obtained by using leveling with magnetometer
            // and accelerometer readings (once both magnetometer and accelerometer
            // are calibrated).
            final AngularSpeedTriad initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
            final Matrix initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

            if (generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
                continue;
            }

            final ECEFGravity gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

            // configure calibrators and data source
            final Matrix initialBa = new Matrix(3, 1);
            final Matrix initialMa = new Matrix(3, 3);
            final PROMedSRobustKnownGravityNormAccelerometerCalibrator accelerometerCalibrator =
                    new PROMedSRobustKnownGravityNormAccelerometerCalibrator();
            accelerometerCalibrator.setGroundTruthGravityNorm(gravity.getNorm());
            accelerometerCalibrator.setCommonAxisUsed(true);
            accelerometerCalibrator.setInitialBias(initialBa);
            accelerometerCalibrator.setInitialMa(initialMa);


            final Matrix initialMg = new Matrix(3, 3);
            final Matrix initialGg = new Matrix(3, 3);
            final PROMedSRobustEasyGyroscopeCalibrator gyroscopeCalibrator = new PROMedSRobustEasyGyroscopeCalibrator();
            gyroscopeCalibrator.setCommonAxisUsed(true);
            gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
            gyroscopeCalibrator.setInitialBias(initialBg);
            gyroscopeCalibrator.setInitialMg(initialMg);
            gyroscopeCalibrator.setInitialGg(initialGg);

            // create optimizer
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer =
                    new ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(mDataSource,
                            accelerometerCalibrator, gyroscopeCalibrator);
            optimizer.setListener(this);

            reset();
            assertEquals(0, mStart);
            assertEquals(0, mEnd);
            assertEquals(0.0f, mProgress, 0.0f);

            final double thresholdFactor = optimizer.optimize();

            // check optimization results
            assertEquals(1, mStart);
            assertEquals(1, mEnd);
            assertTrue(mProgress > 0.0f);
            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevel() > 0.0);
            final Acceleration accelerometerBaseNoiseLevel1 = optimizer.getAccelerometerBaseNoiseLevelAsMeasurement();
            assertEquals(accelerometerBaseNoiseLevel1.getValue().doubleValue(),
                    optimizer.getAccelerometerBaseNoiseLevel(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, accelerometerBaseNoiseLevel1.getUnit());
            final Acceleration accelerometerBaseNoiseLevel2 = new Acceleration(1.0,
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getAccelerometerBaseNoiseLevelAsMeasurement(accelerometerBaseNoiseLevel2);
            assertEquals(accelerometerBaseNoiseLevel1, accelerometerBaseNoiseLevel2);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelPsd() > 0.0);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getGyroscopeBaseNoiseLevelPsd()),
                    optimizer.getGyroscopeBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getAccelerometerBaseNoiseLevelPsd() > 0.0);
            assertEquals(optimizer.getAccelerometerBaseNoiseLevelPsd(),
                    Math.pow(optimizer.getAccelerometerBaseNoiseLevel(), 2.0) * optimizer.getTimeInterval(),
                    SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getGyroscopeBaseNoiseLevelRootPsd() > 0.0);
            assertEquals(Math.sqrt(optimizer.getAccelerometerBaseNoiseLevelPsd()),
                    optimizer.getAccelerometerBaseNoiseLevelRootPsd(), SMALL_ABSOLUTE_ERROR);
            assertTrue(optimizer.getThreshold() > 0.0);
            final Acceleration threshold1 = optimizer.getThresholdAsMeasurement();
            assertEquals(optimizer.getThreshold(), threshold1.getValue().doubleValue(), 0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, threshold1.getUnit());
            final Acceleration threshold2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            optimizer.getThresholdAsMeasurement(threshold2);
            assertEquals(threshold1, threshold2);
            assertTrue(optimizer.getEstimatedAccelerometerBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFxVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFyVariance() > 0.0);
            assertTrue(optimizer.getEstimatedAccelerometerBiasFzVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedAccelerometerBiases());

            final Matrix optimalBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final Matrix optimalMa = optimizer.getEstimatedAccelerometerMa();

            assertNotNull(optimalBa);
            assertNotNull(optimalMa);

            if (!ba.equals(optimalBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(optimalMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(optimalBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(optimalMa, ABSOLUTE_ERROR));

            assertTrue(optimizer.getEstimatedGyroscopeBiasStandardDeviationNorm() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasXVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasYVariance() > 0.0);
            assertTrue(optimizer.getEstimatedGyroscopeBiasZVariance() > 0.0);
            assertNotNull(optimizer.getEstimatedGyroscopeBiases());

            final Matrix optimalBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final Matrix optimalMg = optimizer.getEstimatedGyroscopeMg();
            final Matrix optimalGg = optimizer.getEstimatedGyroscopeGg();

            assertNotNull(optimalBg);
            assertNotNull(optimalMg);
            assertNotNull(optimalGg);

            if (!bg.equals(optimalBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(optimalMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(optimalGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(optimalBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(optimalMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(optimalGg, 0.0));

            // generate measurements for calibrator using estimated threshold factor
            // on generator that optimizes calibration
            mAccelerometerGeneratorMeasurements.clear();
            mGyroscopeGeneratorMeasurements.clear();
            generator.reset();
            generator.setThresholdFactor(thresholdFactor);

            for (TimedBodyKinematics timedBodyKinematics : mTimedBodyKinematics) {
                assertTrue(generator.process(timedBodyKinematics));
            }

            // use generated measurements from generator that used optimal threshold factor
            accelerometerCalibrator.setMeasurements(mAccelerometerGeneratorMeasurements);
            gyroscopeCalibrator.setSequences(mGyroscopeGeneratorMeasurements);

            // calibrate
            try {
                accelerometerCalibrator.calibrate();
                gyroscopeCalibrator.calibrate();
            } catch (final CalibrationException e) {
                continue;
            }

            // check calibration result
            final Matrix estimatedBa = accelerometerCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMa = accelerometerCalibrator.getEstimatedMa();

            final Matrix estimatedBg = gyroscopeCalibrator.getEstimatedBiasesAsMatrix();
            final Matrix estimatedMg = gyroscopeCalibrator.getEstimatedMg();
            final Matrix estimatedGg = gyroscopeCalibrator.getEstimatedGg();

            if (!ba.equals(estimatedBa, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!ma.equals(estimatedMa, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ba.equals(estimatedBa, ABSOLUTE_ERROR));
            assertTrue(ma.equals(estimatedMa, ABSOLUTE_ERROR));

            if (!bg.equals(estimatedBg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!mg.equals(estimatedMg, ABSOLUTE_ERROR)) {
                continue;
            }
            if (!gg.equals(estimatedGg, 0.0)) {
                continue;
            }
            assertTrue(bg.equals(estimatedBg, ABSOLUTE_ERROR));
            assertTrue(mg.equals(estimatedMg, ABSOLUTE_ERROR));
            assertTrue(gg.equals(estimatedGg, 0.0));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onOptimizeStart(
            final IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematics,
                    AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mStart++;
        checkLocked((ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeEnd(
            final IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematics,
                    AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource> optimizer) {
        mEnd++;
        checkLocked((ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer) optimizer);
    }

    @Override
    public void onOptimizeProgressChange(
            final IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematics,
                    AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource> optimizer,
            final float progress) {
        assertTrue(progress >= 0.0f);
        assertTrue(progress <= 1.0f);
        assertTrue(progress > mProgress);
        if (mProgress == 0.0) {
            checkLocked((ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer) optimizer);
        }
        mProgress = progress;
    }

    private static void checkLocked(
            final ExhaustiveAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer) {
        assertTrue(optimizer.isRunning());
        assertThrows(LockedException.class, () -> optimizer.setDataSource(null));
        assertThrows(LockedException.class, () -> optimizer.setListener(null));
        assertThrows(LockedException.class, () -> optimizer.setAccelerometerCalibrator(null));
        assertThrows(LockedException.class, () -> optimizer.setGyroscopeCalibrator(null));
        assertThrows(LockedException.class, () -> optimizer.setAccelerometerQualityScoreMapper(null));
        assertThrows(LockedException.class, () -> optimizer.setGyroscopeQualityScoreMapper(null));
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
        assertThrows(LockedException.class, () -> optimizer.setThresholdFactorStep(0.0));
        assertThrows(LockedException.class, optimizer::optimize);
    }

    private void generateBodyKinematics(
            final NEDFrame nedFrame, final ECEFFrame ecefFrame, final boolean changePosition, final Matrix ma,
            final double accelNoiseRootPSD, final double gyroNoiseRootPSD, final int numSequences,
            final int numMeasurements) throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {

        final Matrix ba = generateBa();
        final Matrix bg = generateBg();
        final Matrix mg = generateMg();
        final Matrix gg = generateGg();

        final double accelQuantLevel = 0.0;
        final double gyroQuantLevel = 0.0;

        final IMUErrors errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final BodyKinematics trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(
                TIME_INTERVAL_SECONDS, ecefFrame, ecefFrame);

        // generate initial static samples
        final int initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        generateStaticSamples(initialStaticSamples, trueKinematics, errors, random, 0);

        final int n = Math.max(numSequences + 1, numMeasurements);

        final int staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final int dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        int start = initialStaticSamples;
        for (int i = 0; i < n; i++) {
            // generate static samples
            generateStaticSamples(staticPeriodLength, trueKinematics, errors, random, start);
            start += staticPeriodLength;

            // generate dynamic samples
            generateDynamicSamples(dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame, errors, random,
                    start, changePosition);
            start += dynamicPeriodLength;
        }
    }

    private static NEDFrame generateFrame() throws InvalidSourceAndDestinationFrameTypeException {
        final Random random = new Random();
        final UniformRandomizer randomizer = new UniformRandomizer(random);
        final double latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final double longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final double height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final NEDPosition nedPosition = new NEDPosition(latitude, longitude, height);

        final double roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final double yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final CoordinateTransformation nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        return new NEDFrame(nedPosition, nedC);
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

    private static Matrix generateMaGeneral() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private Matrix generateMg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final Matrix result = new Matrix(3, 3);
        final double tmp = DEG_TO_RAD / (3600 * 9.80665);
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
            final int numSamples, final BodyKinematics trueKinematics, final IMUErrors errors, final Random random,
            final int startSample) {

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random);

            final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(j * TIME_INTERVAL_SECONDS);

            mTimedBodyKinematics.add(timedMeasuredKinematics);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void generateDynamicSamples(
            final int numSamples, final BodyKinematics trueKinematics, final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame, final NEDFrame nedFrame, final IMUErrors errors, final Random random,
            final int startSample, final boolean changePosition) throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {

        final double sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final double specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final double angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final double deltaX = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaY = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final double deltaZ = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final double deltaRoll = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaPitch = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final double deltaYaw = Math.toRadians(randomizer.nextDouble(
                MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final double ecefX = ecefFrame.getX();
        final double ecefY = ecefFrame.getY();
        final double ecefZ = ecefFrame.getZ();

        final CoordinateTransformation nedC = nedFrame.getCoordinateTransformation();

        final double roll = nedC.getRollEulerAngle();
        final double pitch = nedC.getPitchEulerAngle();
        final double yaw = nedC.getYawEulerAngle();

        final Quaternion beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

        NEDFrame oldNedFrame = new NEDFrame(nedFrame);
        NEDFrame newNedFrame = new NEDFrame();
        ECEFFrame oldEcefFrame = new ECEFFrame(ecefFrame);
        ECEFFrame newEcefFrame = new ECEFFrame();

        double oldEcefX = ecefX - deltaX;
        double oldEcefY = ecefY - deltaY;
        double oldEcefZ = ecefZ - deltaZ;
        double oldRoll = roll - deltaRoll;
        double oldPitch = pitch - deltaPitch;
        double oldYaw = yaw - deltaYaw;

        final BodyKinematics measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random);
        final double beforeMeanFx = measuredBeforeGravityKinematics.getFx();
        final double beforeMeanFy = measuredBeforeGravityKinematics.getFy();
        final double beforeMeanFz = measuredBeforeGravityKinematics.getFz();

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence = new BodyKinematicsSequence<>();
        sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

        final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> trueSequence =
                new BodyKinematicsSequence<>();
        final List<StandardDeviationTimedBodyKinematics> trueTimedKinematicsList = new ArrayList<>();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final double progress = (double) i / (double) numSamples;

            final double newRoll = oldRoll + interpolate(deltaRoll, progress);
            final double newPitch = oldPitch + interpolate(deltaPitch, progress);
            final double newYaw = oldYaw + interpolate(deltaYaw, progress);
            final CoordinateTransformation newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw,
                    FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
            final NEDPosition newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final double newEcefX = oldEcefX + interpolate(deltaX, progress);
            final double newEcefY = oldEcefY + interpolate(deltaY, progress);
            final double newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final double timestampSeconds = j * TIME_INTERVAL_SECONDS;

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame,
                    trueKinematics);

            // add error to true kinematics
            final BodyKinematics measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                    trueKinematics, errors, random);

            final TimedBodyKinematics timedMeasuredKinematics = new TimedBodyKinematics();
            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(timestampSeconds);

            mTimedBodyKinematics.add(timedMeasuredKinematics);

            final StandardDeviationTimedBodyKinematics trueTimedKinematics = new StandardDeviationTimedBodyKinematics(
                    new BodyKinematics(trueKinematics), timestampSeconds, specificForceStandardDeviation,
                    angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        trueSequence.setItems(trueTimedKinematicsList);

        final Quaternion afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, QuaternionStepIntegratorType.RUNGE_KUTTA,
                afterQ);

        final CoordinateTransformation newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(),
                FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);


        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after dynamic sequence finishes, update true kinematics for a
        // static sequence at current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame, trueKinematics);
    }

    // This is required to simulate a smooth transition of values during
    // dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behaviour.
    private static double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }

    private void reset() {
        mStart = 0;
        mEnd = 0;
        mProgress = 0.0f;
    }
}
