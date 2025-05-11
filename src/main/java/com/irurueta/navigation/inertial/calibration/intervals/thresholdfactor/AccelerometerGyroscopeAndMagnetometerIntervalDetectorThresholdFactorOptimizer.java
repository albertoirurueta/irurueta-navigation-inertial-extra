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
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSTightlyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.GyroscopeNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerCalibratorMeasurementType;
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownBiasAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.OrderedStandardDeviationBodyKinematicsAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.QualityScoredAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.UnknownBiasAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.accelerometer.UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener;
import com.irurueta.navigation.inertial.calibration.gyroscope.AccelerometerDependentGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeCalibratorMeasurementOrSequenceType;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.OrderedBodyKinematicsSequenceGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QualityScoredGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.UnknownBiasGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.magnetometer.KnownHardIronMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerCalibratorMeasurementType;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.QualityScoredMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.UnknownHardIronMagnetometerCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Time;

import java.util.ArrayList;
import java.util.List;

/**
 * Optimizes the threshold factor for interval detection of accelerometer and gyroscope
 * data based on results of calibration.
 * Implementations of this class will attempt to find the best threshold factor
 * between the provided range of values.
 * Only accelerometer calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link AccelerometerNonLinearCalibrator} and must support
 * {@link AccelerometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_KINEMATICS}).
 * Only gyroscope calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link GyroscopeNonLinearCalibrator} and must support
 * {@link GyroscopeCalibratorMeasurementOrSequenceType#BODY_KINEMATICS_SEQUENCE}).
 * Only magnetometer calibrators based on unknown orientation are supported, in other terms,
 * calibrators must be {@link MagnetometerNonLinearCalibrator} and must support
 * {@link MagnetometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY}.
 */
public abstract class AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer extends
        IntervalDetectorThresholdFactorOptimizer<TimedBodyKinematicsAndMagneticFluxDensity,
                AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> implements
        AccelerometerNoiseRootPsdSource, GyroscopeNoiseRootPsdSource {

    /**
     * Default minimum threshold factor.
     */
    public static final double DEFAULT_MIN_THRESHOLD_FACTOR = 2.0;

    /**
     * Default maximum threshold factor.
     */
    public static final double DEFAULT_MAX_THRESHOLD_FACTOR = 10.0;

    /**
     * Minimum threshold factor.
     */
    protected double minThresholdFactor = DEFAULT_MIN_THRESHOLD_FACTOR;

    /**
     * Maximum threshold factor.
     */
    protected double maxThresholdFactor = DEFAULT_MAX_THRESHOLD_FACTOR;

    /**
     * Accelerometer calibrator.
     */
    private AccelerometerNonLinearCalibrator accelerometerCalibrator;

    /**
     * Gyroscope calibrator.
     */
    private GyroscopeNonLinearCalibrator gyroscopeCalibrator;

    /**
     * Magnetometer calibrator.
     */
    private MagnetometerNonLinearCalibrator magnetometerCalibrator;

    /**
     * A measurement generator for accelerometer, gyroscope and magnetometer calibrators.
     */
    private AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator;

    /**
     * Generated measurements to be used for accelerometer calibration.
     */
    private List<StandardDeviationBodyKinematics> accelerometerMeasurements;

    /**
     * Generated sequences to be used for gyroscope calibration.
     */
    private List<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>> gyroscopeSequences;

    /**
     * Generated measurements to be used for magnetometer calibration.
     */
    private List<StandardDeviationBodyMagneticFluxDensity> magnetometerMeasurements;

    /**
     * Mapper to convert {@link StandardDeviationBodyKinematics} measurements into
     * quality scores.
     */
    private QualityScoreMapper<StandardDeviationBodyKinematics> accelerometerQualityScoreMapper =
            new DefaultAccelerometerQualityScoreMapper();

    /**
     * Mapper to convert {@link BodyKinematicsSequence} sequences of {@link StandardDeviationTimedBodyKinematics}
     * into quality scores.
     */
    private QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
            gyroscopeQualityScoreMapper = new DefaultGyroscopeQualityScoreMapper();

    /**
     * Mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements
     * into quality scores.
     */
    private QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> magnetometerQualityScoreMapper =
            new DefaultMagnetometerQualityScoreMapper();

    /**
     * Rule to convert accelerometer, gyroscope and magnetometer MSE
     * values into a single global MSE value.
     */
    private AccelerometerGyroscopeAndMagnetometerMseRule mseRule =
            new DefaultAccelerometerGyroscopeAndMagnetometerMseRule();

    /**
     * Estimated norm of gyroscope noise root PSD (Power Spectral Density)
     * expressed as (rad * s^-0.5).
     */
    private double angularSpeedNoiseRootPsd;

    /**
     * Accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found expressed in
     * meters per squared second (m/s^2).
     * This is equal to the standard deviation of the accelerometer measurements
     * during the initialization phase.
     */
    private double baseNoiseLevel;

    /**
     * Threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2) for the best calibration solution that
     * has been found.
     */
    private double threshold;

    /**
     * Estimated covariance matrix for estimated accelerometer parameters.
     */
    private Matrix estimatedAccelerometerCovariance;

    /**
     * Estimated accelerometer scale factors and cross-coupling errors.
     * This is the product of matrix Ta containing cross-coupling errors and Ka
     * containing scaling factors.
     * So that:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix. However, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    private Matrix estimatedAccelerometerMa;

    /**
     * Estimated accelerometer biases for each IMU axis expressed in meter per squared
     * second (m/s^2).
     */
    private double[] estimatedAccelerometerBiases;

    /**
     * Estimated covariance matrix for estimated gyroscope parameters.
     */
    private Matrix estimatedGyroscopeCovariance;

    /**
     * Estimated angular rate biases for each IMU axis expressed in radians per
     * second (rad/s).
     */
    private double[] estimatedGyroscopeBiases;

    /**
     * Estimated gyroscope scale factors and cross-coupling errors.
     * This is the product of matrix Tg containing cross-coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix. However, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    private Matrix estimatedGyroscopeMg;

    /**
     * Estimated G-dependent cross-biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     * This instance allows any 3x3 matrix.
     */
    private Matrix estimatedGyroscopeGg;

    /**
     * Estimated covariance matrix for estimated magnetometer parameters.
     */
    private Matrix estimatedMagnetometerCovariance;

    /**
     * Estimated magnetometer soft-iron matrix containing scale factors
     * and cross-coupling errors.
     * This is the product of matrix Tm containing cross-coupling errors and Km
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix. However, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     */
    private Matrix estimatedMagnetometerMm;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] estimatedMagnetometerHardIron;

    /**
     * Constructor.
     */
    protected AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer() {
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    protected AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
        initialize();
    }

    /**
     * Constructor.
     *
     * @param accelerometerCalibrator an accelerometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @param gyroscopeCalibrator     a gyroscope calibrator to be used to optimize
     *                                its Mean Square Error (MSE).
     * @param magnetometerCalibrator  a magnetometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements,
     *                                  if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics} or
     *                                  if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    protected AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator) {
        try {
            setAccelerometerCalibrator(accelerometerCalibrator);
            setGyroscopeCalibrator(gyroscopeCalibrator);
            setMagnetometerCalibrator(magnetometerCalibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource              instance in charge of retrieving data for this optimizer.
     * @param accelerometerCalibrator an accelerometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @param gyroscopeCalibrator     a gyroscope calibrator to be used to optimize
     *                                its Mean Square Error (MSE).
     * @param magnetometerCalibrator  a magnetometer calibrator to be used to
     *                                optimize its Mean Square Error (MSE).
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements,
     *                                  if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics} or
     *                                  if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    protected AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator) {
        super(dataSource);
        try {
            setAccelerometerCalibrator(accelerometerCalibrator);
            setGyroscopeCalibrator(gyroscopeCalibrator);
            setMagnetometerCalibrator(magnetometerCalibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Gets provided accelerometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return accelerometer calibrator to be used to optimize its MSE.
     */
    public AccelerometerNonLinearCalibrator getAccelerometerCalibrator() {
        return accelerometerCalibrator;
    }

    /**
     * Sets accelerometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @param accelerometerCalibrator accelerometer calibrator to be used to optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements.
     */
    public void setAccelerometerCalibrator(
            final AccelerometerNonLinearCalibrator accelerometerCalibrator) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (accelerometerCalibrator != null && accelerometerCalibrator.getMeasurementType()
                != AccelerometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_KINEMATICS) {
            throw new IllegalArgumentException();
        }

        this.accelerometerCalibrator = accelerometerCalibrator;
    }

    /**
     * Gets provided gyroscope calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return gyroscope calibrator to be used to optimize its MSE.
     */
    public GyroscopeNonLinearCalibrator getGyroscopeCalibrator() {
        return gyroscopeCalibrator;
    }

    /**
     * Sets gyroscope calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @param gyroscopeCalibrator gyroscope calibrator to be use dto optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics}.
     */
    public void setGyroscopeCalibrator(final GyroscopeNonLinearCalibrator gyroscopeCalibrator) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (gyroscopeCalibrator != null && gyroscopeCalibrator.getMeasurementOrSequenceType()
                != GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE) {
            throw new IllegalArgumentException();
        }

        this.gyroscopeCalibrator = gyroscopeCalibrator;
    }

    /**
     * Gets provided magnetometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return magnetometer calibrator to be used to optimize its MSE.
     */
    public MagnetometerNonLinearCalibrator getMagnetometerCalibrator() {
        return magnetometerCalibrator;
    }

    /**
     * Sets a magnetometer calibrator to be used to optimize its Mean Square Error.
     *
     * @param magnetometerCalibrator magnetometer calibrator to be used to optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public void setMagnetometerCalibrator(final MagnetometerNonLinearCalibrator magnetometerCalibrator)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (magnetometerCalibrator != null && magnetometerCalibrator.getMeasurementType()
                != MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY) {
            throw new IllegalArgumentException();
        }

        this.magnetometerCalibrator = magnetometerCalibrator;
    }

    /**
     * Gets mapper to convert {@link StandardDeviationBodyKinematics} accelerometer measurements
     * into quality scores.
     *
     * @return mapper to convert accelerometer measurements into quality scores.
     */
    public QualityScoreMapper<StandardDeviationBodyKinematics> getAccelerometerQualityScoreMapper() {
        return accelerometerQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link StandardDeviationBodyKinematics} accelerometer measurements
     * into quality scores.
     *
     * @param accelerometerQualityScoreMapper mapper to convert accelerometer measurements into
     *                                        quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setAccelerometerQualityScoreMapper(
            final QualityScoreMapper<StandardDeviationBodyKinematics> accelerometerQualityScoreMapper)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.accelerometerQualityScoreMapper = accelerometerQualityScoreMapper;
    }

    /**
     * Gets mapper to convert {@link BodyKinematicsSequence} gyroscope sequences of
     * {@link StandardDeviationTimedBodyKinematics} into quality scores.
     *
     * @return mapper to convert gyroscope sequences into quality scores.
     */
    public QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
    getGyroscopeQualityScoreMapper() {
        return gyroscopeQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link BodyKinematicsSequence} gyroscope sequences of
     * {@link StandardDeviationTimedBodyKinematics} into quality scores.
     *
     * @param gyroscopeQualityScoreMapper mapper to convert gyroscope sequences
     *                                    into quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setGyroscopeQualityScoreMapper(
            final QualityScoreMapper<BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>>
                    gyroscopeQualityScoreMapper) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.gyroscopeQualityScoreMapper = gyroscopeQualityScoreMapper;
    }

    /**
     * Gets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} magnetometer
     * measurements into quality scores.
     *
     * @return mapper to convert magnetometer measurements into quality scores.
     */
    public QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> getMagnetometerQualityScoreMapper() {
        return magnetometerQualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} magnetometer
     * measurements into quality scores.
     *
     * @param magnetometerQualityScoreMapper mapper to convert magnetometer measurements into
     *                                       quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setMagnetometerQualityScoreMapper(
            final QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> magnetometerQualityScoreMapper)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.magnetometerQualityScoreMapper = magnetometerQualityScoreMapper;
    }

    /**
     * Gets rule to convert accelerometer, gyroscope and magnetometer
     * MSE values into a single global MSE value.
     *
     * @return rule to convert accelerometer, gyroscope and
     * magnetometer MSE values into a single global MSE value.
     */
    public AccelerometerGyroscopeAndMagnetometerMseRule getMseRule() {
        return mseRule;
    }

    /**
     * Sets rule to convert accelerometer, gyroscope and magnetometer
     * MSE values into a single global MSE value.
     *
     * @param mseRule rule to convert accelerometer, gyroscope and
     *                magnetometer MSE values into a single global
     *                MSE value.
     * @throws LockedException if optimizer is already running.
     */
    public void setMseRule(final AccelerometerGyroscopeAndMagnetometerMseRule mseRule) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.mseRule = mseRule;
    }

    /**
     * Gets the minimum threshold factor.
     *
     * @return minimum threshold factor.
     */
    public double getMinThresholdFactor() {
        return minThresholdFactor;
    }

    /**
     * Gets the maximum threshold factor.
     *
     * @return maximum threshold factor.
     */
    public double getMaxThresholdFactor() {
        return maxThresholdFactor;
    }

    /**
     * Sets a range of threshold factor values to get an optimized
     * threshold factor value.
     *
     * @param minThresholdFactor minimum threshold.
     * @param maxThresholdFactor maximum threshold.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if either minimum or maximum values are
     *                                  negative, or if the minimum value is larger
     *                                  than the maximum one.
     */
    public void setThresholdFactorRange(final double minThresholdFactor, final double maxThresholdFactor)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (minThresholdFactor < 0.0 || maxThresholdFactor < 0.0 || minThresholdFactor >= maxThresholdFactor) {
            throw new IllegalArgumentException();
        }

        this.minThresholdFactor = minThresholdFactor;
        this.maxThresholdFactor = maxThresholdFactor;
    }

    /**
     * Indicates whether this optimizer is ready to start optimization.
     *
     * @return true if this optimizer is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && accelerometerCalibrator != null
                && gyroscopeCalibrator != null
                && magnetometerCalibrator != null
                && accelerometerQualityScoreMapper != null
                && gyroscopeQualityScoreMapper != null
                && magnetometerQualityScoreMapper != null
                && mseRule != null;
    }

    /**
     * Gets the time interval between input measurements provided to the
     * {@link #getDataSource()} expressed in seconds (s).
     *
     * @return time interval between input measurements.
     */
    public double getTimeInterval() {
        return generator.getTimeInterval();
    }

    /**
     * Sets time interval between input measurements provided to the
     * {@link #getDataSource()} expressed in seconds (s).
     *
     * @param timeInterval time interval between input measurements.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setTimeInterval(timeInterval);
    }

    /**
     * Gets the time interval between input measurements provided to the
     * {@link #getDataSource()}.
     *
     * @return time interval between input measurements.
     */
    public Time getTimeIntervalAsTime() {
        return generator.getTimeIntervalAsTime();
    }

    /**
     * Gets the time interval between input measurements provided to the
     * {@link #getDataSource()}.
     *
     * @param result instance where the time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        generator.getTimeIntervalAsTime(result);
    }

    /**
     * Sets time interval between input measurements provided to the
     * {@link #getDataSource()}.
     *
     * @param timeInterval time interval between input measurements.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is negative.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setTimeInterval(timeInterval);
    }

    /**
     * Gets minimum number of input measurements provided to the
     * {@link #getDataSource()} required in a static interval to be taken
     * into account.
     * Smaller static intervals will be discarded.
     *
     * @return a minimum number of input measurements required in a static interval
     * to be taken into account.
     */
    public int getMinStaticSamples() {
        return generator.getMinStaticSamples();
    }

    /**
     * Sets minimum number of input measurements provided to the
     * {@link #getDataSource()} required in a static interval to be taken
     * into account.
     * Smaller static intervals will be discarded.
     *
     * @param minStaticSamples a minimum number of input measurements required in
     *                         a static interval to be taken into account.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMinStaticSamples(final int minStaticSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setMinStaticSamples(minStaticSamples);
    }

    /**
     * Gets maximum number of input measurements provided to the
     * {@link #getDataSource()} allowed in dynamic intervals.
     *
     * @return maximum number of input measurements allowed in dynamic intervals.
     */
    public int getMaxDynamicSamples() {
        return generator.getMaxDynamicSamples();
    }

    /**
     * Sets maximum number of input measurements provided to the
     * {@link #getDataSource()} allowed in dynamic intervals.
     *
     * @param maxDynamicSamples maximum number of input measurements allowed in
     *                          dynamic intervals.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is less than 2.
     */
    public void setMaxDynamicSamples(final int maxDynamicSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setMaxDynamicSamples(maxDynamicSamples);
    }

    /**
     * Gets length of number of input measurements provided to the
     * {@link #getDataSource()} to keep within the window being processed
     * to determine instantaneous accelerometer noise level.
     *
     * @return length of input measurements to keep within the window.
     */
    public int getWindowSize() {
        return generator.getWindowSize();
    }

    /**
     * Sets length of number of input measurements provided to the
     * {@link #getDataSource()} to keep within the window being processed
     * to determine instantaneous noise level.
     * Window size must always be larger than the allowed minimum value, which is 2
     * and must have an odd value.
     *
     * @param windowSize length of number of samples to keep within the window.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is not valid.
     */
    public void setWindowSize(final int windowSize) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setWindowSize(windowSize);
    }

    /**
     * Gets number of input measurements provided to the
     * {@link #getDataSource()} to be processed initially while keeping the sensor
     * static to find the base noise level when the device is static.
     *
     * @return number of samples to be processed initially.
     */
    public int getInitialStaticSamples() {
        return generator.getInitialStaticSamples();
    }

    /**
     * Sets number of input parameters provided to the {@link #getDataSource()}
     * to be processed initially while keeping the sensor static to
     * find the base noise level when the device is static.
     *
     * @param initialStaticSamples number of samples ot be processed initially.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is less than
     *                                  {@link TriadStaticIntervalDetector#MINIMUM_INITIAL_STATIC_SAMPLES}.
     */
    public void setInitialStaticSamples(final int initialStaticSamples) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setInitialStaticSamples(initialStaticSamples);
    }

    /**
     * Gets factor to determine that a sudden movement has occurred during
     * initialization if instantaneous noise level exceeds accumulated noise
     * level by this factor amount.
     * This factor is unit-less.
     *
     * @return factor to determine that a sudden movement has occurred.
     */
    public double getInstantaneousNoiseLevelFactor() {
        return generator.getInstantaneousNoiseLevelFactor();
    }

    /**
     * Sets factor to determine that a sudden movement has occurred during
     * initialization if instantaneous noise level exceeds accumulated noise
     * level by this factor amount.
     * This factor is unit-less.
     *
     * @param instantaneousNoiseLevelFactor factor to determine that a sudden
     *                                      movement has occurred during
     *                                      initialization.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setInstantaneousNoiseLevelFactor(final double instantaneousNoiseLevelFactor) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setInstantaneousNoiseLevelFactor(instantaneousNoiseLevelFactor);
    }

    /**
     * Gets the overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @return overall absolute threshold to determine whether there has been
     * excessive motion.
     */
    public double getBaseNoiseLevelAbsoluteThreshold() {
        return generator.getBaseNoiseLevelAbsoluteThreshold();
    }

    /**
     * Sets the overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     * This threshold is expressed in meters per squared second (m/s^2).
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(final double baseNoiseLevelAbsoluteThreshold)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets the overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     *
     * @return overall absolute threshold to determine whether there has been
     * excessive motion.
     */
    public Acceleration getBaseNoiseLevelAbsoluteThresholdAsMeasurement() {
        return generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement();
    }

    /**
     * Gets the overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     *
     * @param result instance where the result will be stored.
     */
    public void getBaseNoiseLevelAbsoluteThresholdAsMeasurement(final Acceleration result) {
        generator.getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result);
    }

    /**
     * Sets the overall absolute threshold to determine whether there has been
     * excessive motion during the whole initialization phase.
     *
     * @param baseNoiseLevelAbsoluteThreshold overall absolute threshold to
     *                                        determine whether there has been
     *                                        excessive motion.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setBaseNoiseLevelAbsoluteThreshold(final Acceleration baseNoiseLevelAbsoluteThreshold)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        generator.setBaseNoiseLevelAbsoluteThreshold(baseNoiseLevelAbsoluteThreshold);
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found expressed in
     * meters per squared second (m/s^2).
     * This is equal to the standard deviation of the accelerometer measurements
     * during the initialization phase.
     *
     * @return accelerometer base noise level of the best solution that has been
     * found.
     */
    public double getAccelerometerBaseNoiseLevel() {
        return baseNoiseLevel;
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found.
     * This is equal to the standard deviation of the accelerometer measurements
     * during the initialization phase.
     *
     * @return accelerometer base noise level of the best solution that has been
     * found.
     */
    public Acceleration getAccelerometerBaseNoiseLevelAsMeasurement() {
        return createMeasurement(baseNoiseLevel, getDefaultUnit());
    }

    /**
     * Gets accelerometer base noise level that has been detected during
     * initialization of the best solution that has been found.
     * This is equal to the standard deviation of the accelerometer measurements
     * during the initialization phase.
     *
     * @param result instance where the result will be stored.
     */
    public void getAccelerometerBaseNoiseLevelAsMeasurement(final Acceleration result) {
        result.setValue(baseNoiseLevel);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets gyroscope base noise level PSD (Power Spectral Density)
     * expressed in (rad^2/s).
     *
     * @return gyroscope base noise level PSD.
     */
    public double getGyroscopeBaseNoiseLevelPsd() {
        return angularSpeedNoiseRootPsd * angularSpeedNoiseRootPsd;
    }

    /**
     * Gets gyroscope base noise level root PSD (Power Spectral Density)
     * expressed in (rad * s^-0.5)
     *
     * @return gyroscope base noise level root PSD.
     */
    @Override
    public double getGyroscopeBaseNoiseLevelRootPsd() {
        return angularSpeedNoiseRootPsd;
    }

    /**
     * Gets accelerometer base noise level PSD (Power Spectral Density)
     * expressed in (m^2 * s^-3).
     *
     * @return accelerometer base noise level PSD.
     */
    public double getAccelerometerBaseNoiseLevelPsd() {
        return baseNoiseLevel * baseNoiseLevel * getTimeInterval();
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5).
     *
     * @return accelerometer base noise level root PSD.
     */
    @Override
    public double getAccelerometerBaseNoiseLevelRootPsd() {
        return baseNoiseLevel * Math.sqrt(getTimeInterval());
    }

    /**
     * Gets the threshold to determine static/dynamic period changes expressed in
     * meters per squared second (m/s^2) for the best calibration solution that
     * has been found.
     *
     * @return threshold to determine static/dynamic period changes for the best
     * solution.
     */
    public double getThreshold() {
        return threshold;
    }

    /**
     * Gets the threshold to determine static/dynamic period changes for the best
     * calibration solution that has been found.
     *
     * @return threshold to determine static/dynamic period changes for the best
     * solution.
     */
    public Acceleration getThresholdAsMeasurement() {
        return createMeasurement(threshold, getDefaultUnit());
    }

    /**
     * Get the threshold to determine static/dynamic period changes for the best
     * calibration solution that has been found.
     *
     * @param result instance where the result will be stored.
     */
    public void getThresholdAsMeasurement(final Acceleration result) {
        result.setValue(threshold);
        result.setUnit(getDefaultUnit());
    }

    /**
     * Gets estimated standard deviation norm of accelerometer bias expressed in
     * meters per squared second (m/s^2).
     * This can be used as the initial accelerometer bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return estimated standard deviation norm of accelerometer bias or null
     * if not available.
     */
    public Double getEstimatedAccelerometerBiasStandardDeviationNorm() {
        return estimatedAccelerometerCovariance != null
                ? Math.sqrt(getEstimatedAccelerometerBiasFxVariance()
                + getEstimatedAccelerometerBiasFyVariance()
                + getEstimatedAccelerometerBiasFzVariance())
                : null;
    }

    /**
     * Gets estimated x coordinate variance of accelerometer bias expressed in (m^2/s^4).
     *
     * @return estimated x coordinate variance of accelerometer bias or null if not available.
     */
    public Double getEstimatedAccelerometerBiasFxVariance() {
        return estimatedAccelerometerCovariance != null
                ? estimatedAccelerometerCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y coordinate variance of accelerometer bias expressed in (m^2/s^4).
     *
     * @return estimated y coordinate variance of accelerometer bias or null if not available.
     */
    public Double getEstimatedAccelerometerBiasFyVariance() {
        return estimatedAccelerometerCovariance != null
                ? estimatedAccelerometerCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z coordinate variance of accelerometer bias expressed in (m^2/s^4).
     *
     * @return estimated z coordinate variance of accelerometer bias or null if not available.
     */
    public Double getEstimatedAccelerometerBiasFzVariance() {
        return estimatedAccelerometerCovariance != null
                ? estimatedAccelerometerCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets the array containing x,y,z components of estimated accelerometer biases
     * expressed in meters per squared second (m/s^2).
     *
     * @return array containing x,y,z components of estimated accelerometer biases.
     */
    public double[] getEstimatedAccelerometerBiases() {
        return estimatedAccelerometerBiases;
    }

    /**
     * Gets estimated accelerometer scale factors and cross-coupling errors.
     * This is the product of matrix Ta containing cross-coupling errors and Ka
     * containing scaling factors.
     * So that:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Ka = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Ta = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Ma = [sx    mxy  mxz] = Ta*Ka =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix. However, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Ma matrix
     * becomes upper diagonal:
     * <pre>
     *     Ma = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @return estimated accelerometer scale factors and cross-coupling errors, or null
     * if not available.
     */
    public Matrix getEstimatedAccelerometerMa() {
        return estimatedAccelerometerMa;
    }

    /**
     * Gets estimated standard deviation norm of gyroscope bias expressed in
     * radians per second (rad/s).
     * This can be used as the initial gyroscope bias uncertainty for
     * {@link INSLooselyCoupledKalmanInitializerConfig} or {@link INSTightlyCoupledKalmanInitializerConfig}.
     *
     * @return estimated standard deviation norm of gyroscope bias or null
     * if not available.
     */
    public Double getEstimatedGyroscopeBiasStandardDeviationNorm() {
        return estimatedGyroscopeCovariance != null
                ? Math.sqrt(getEstimatedGyroscopeBiasXVariance()
                + getEstimatedGyroscopeBiasYVariance()
                + getEstimatedGyroscopeBiasZVariance())
                : null;
    }

    /**
     * Gets estimated x coordinate variance of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return estimated x coordinate variance of gyroscope bias or null if not available.
     */
    public Double getEstimatedGyroscopeBiasXVariance() {
        return estimatedGyroscopeCovariance != null
                ? estimatedGyroscopeCovariance.getElementAt(0, 0) : null;
    }

    /**
     * Gets estimated y coordinate variance of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return estimated y coordinate variance of gyroscope bias or null if not available.
     */
    public Double getEstimatedGyroscopeBiasYVariance() {
        return estimatedGyroscopeCovariance != null
                ? estimatedGyroscopeCovariance.getElementAt(1, 1) : null;
    }

    /**
     * Gets estimated z coordinate variance of gyroscope bias expressed in (rad^2/s^2).
     *
     * @return estimated z coordinate variance of gyroscope bias or null if not available.
     */
    public Double getEstimatedGyroscopeBiasZVariance() {
        return estimatedGyroscopeCovariance != null
                ? estimatedGyroscopeCovariance.getElementAt(2, 2) : null;
    }

    /**
     * Gets the array containing x,y,z components of estimated gyroscope biases
     * expressed in radians per second (rad/s).
     *
     * @return array containing x,y,z components of estimated gyroscope biases.
     */
    public double[] getEstimatedGyroscopeBiases() {
        return estimatedGyroscopeBiases;
    }

    /**
     * Gets estimated gyroscope scale factors and cross-coupling errors.
     * This is the product of matrix Tg containing cross-coupling errors and Kg
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Kg = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tg = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mg = [sx    mxy  mxz] = Tg*Kg =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix. However, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the gyroscope z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mg matrix
     * becomes upper diagonal:
     * <pre>
     *     Mg = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @return estimated gyroscope scale factors and cross-coupling errors.
     */
    public Matrix getEstimatedGyroscopeMg() {
        return estimatedGyroscopeMg;
    }

    /**
     * Gets estimated G-dependent cross-biases introduced on the gyroscope by the
     * specific forces sensed by the accelerometer.
     *
     * @return a 3x3 matrix containing g-dependent cross biases.
     */
    public Matrix getEstimatedGyroscopeGg() {
        return estimatedGyroscopeGg;
    }

    /**
     * Gets the array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @return array containing x,y,z components of estimated magnetometer
     * hard-iron biases.
     */
    public double[] getEstimatedMagnetometerHardIron() {
        return estimatedMagnetometerHardIron;
    }

    /**
     * Gets estimated magnetometer soft-iron matrix containing scale factors
     * and cross-coupling errors.
     * This is the product of matrix Tm containing cross-coupling errors and Km
     * containing scaling factors.
     * So that:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km
     *          [myx   sy   myz]
     *          [mzx   mzy  sz ]
     * </pre>
     * Where:
     * <pre>
     *     Km = [sx 0   0 ]
     *          [0  sy  0 ]
     *          [0  0   sz]
     * </pre>
     * and
     * <pre>
     *     Tm = [1          -alphaXy    alphaXz ]
     *          [alphaYx    1           -alphaYz]
     *          [-alphaZx   alphaZy     1       ]
     * </pre>
     * Hence:
     * <pre>
     *     Mm = [sx    mxy  mxz] = Tm*Km =  [sx             -sy * alphaXy   sz * alphaXz ]
     *          [myx   sy   myz]            [sx * alphaYx   sy              -sz * alphaYz]
     *          [mzx   mzy  sz ]            [-sx * alphaZx  sy * alphaZy    sz           ]
     * </pre>
     * This instance allows any 3x3 matrix. However, typically alphaYx, alphaZx and alphaZy
     * are considered to be zero if the accelerometer z-axis is assumed to be the same
     * as the body z-axis. When this is assumed, myx = mzx = mzy = 0 and the Mm matrix
     * becomes upper diagonal:
     * <pre>
     *     Mm = [sx    mxy  mxz]
     *          [0     sy   myz]
     *          [0     0    sz ]
     * </pre>
     * Values of this matrix are unit-less.
     *
     * @return estimated magnetometer soft-iron scale factors and cross-coupling errors,
     * or null if not available.
     */
    public Matrix getEstimatedMagnetometerMm() {
        return estimatedMagnetometerMm;
    }

    /**
     * Evaluates calibration Mean Square Error (MSE) for the provided threshold factor.
     *
     * @param thresholdFactor threshold factor to be used for interval detection
     *                        and measurement generation to be used for
     *                        calibration.
     * @return calibration MSE.
     * @throws LockedException                                   if the generator is busy.
     * @throws CalibrationException                              if calibration fails.
     * @throws NotReadyException                                 if the calibrator is not ready.
     * @throws IntervalDetectorThresholdFactorOptimizerException interval detection failed.
     */
    protected double evaluateForThresholdFactor(final double thresholdFactor) throws LockedException,
            CalibrationException, NotReadyException, IntervalDetectorThresholdFactorOptimizerException {
        if (accelerometerMeasurements == null) {
            accelerometerMeasurements = new ArrayList<>();
        } else {
            accelerometerMeasurements.clear();
        }

        if (gyroscopeSequences == null) {
            gyroscopeSequences = new ArrayList<>();
        } else {
            gyroscopeSequences.clear();
        }

        if (magnetometerMeasurements == null) {
            magnetometerMeasurements = new ArrayList<>();
        } else {
            magnetometerMeasurements.clear();
        }

        generator.reset();
        generator.setThresholdFactor(thresholdFactor);

        var count = dataSource.count();
        var failed = false;
        for (var i = 0; i < count; i++) {
            final var timedBodyKinematics = dataSource.getAt(i);
            if (!generator.process(timedBodyKinematics)) {
                failed = true;
                break;
            }
        }

        if (failed || generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
            // interval detection failed
            return Double.MAX_VALUE;
        }

        // check that enough measurements have been obtained
        if (accelerometerMeasurements.size() < accelerometerCalibrator.getMinimumRequiredMeasurements()
                || gyroscopeSequences.size() < gyroscopeCalibrator.getMinimumRequiredMeasurementsOrSequences()
                || magnetometerMeasurements.size() < magnetometerCalibrator.getMinimumRequiredMeasurements()) {
            return Double.MAX_VALUE;
        }

        // set calibrator measurements
        switch (accelerometerCalibrator.getMeasurementType()) {
            case STANDARD_DEVIATION_BODY_KINEMATICS:
                if (accelerometerCalibrator.isOrderedMeasurementsRequired()) {
                    final var calibrator =
                            (OrderedStandardDeviationBodyKinematicsAccelerometerCalibrator) accelerometerCalibrator;

                    calibrator.setMeasurements(accelerometerMeasurements);

                } else {
                    final var calibrator =
                            (UnorderedStandardDeviationBodyKinematicsAccelerometerCalibrator) accelerometerCalibrator;

                    calibrator.setMeasurements(accelerometerMeasurements);
                }

                if (accelerometerCalibrator.isQualityScoresRequired()) {
                    final var calibrator = (QualityScoredAccelerometerCalibrator) accelerometerCalibrator;

                    final var size = accelerometerMeasurements.size();
                    final var qualityScores = new double[size];
                    for (var i = 0; i < size; i++) {
                        qualityScores[i] = accelerometerQualityScoreMapper.map(accelerometerMeasurements.get(i));
                    }
                    calibrator.setQualityScores(qualityScores);
                }
                break;
            case FRAME_BODY_KINEMATICS, STANDARD_DEVIATION_FRAME_BODY_KINEMATICS:
                // Throw exception. Cannot use frames
            default:
                throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        if (gyroscopeCalibrator.getMeasurementOrSequenceType()
                == GyroscopeCalibratorMeasurementOrSequenceType.BODY_KINEMATICS_SEQUENCE) {
            final var calibrator = (OrderedBodyKinematicsSequenceGyroscopeCalibrator) gyroscopeCalibrator;

            calibrator.setSequences(gyroscopeSequences);
        } else {
            throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        if (gyroscopeCalibrator.isQualityScoresRequired()) {
            final var calibrator = (QualityScoredGyroscopeCalibrator) gyroscopeCalibrator;

            final var size = gyroscopeSequences.size();
            final var qualityScores = new double[size];
            for (var i = 0; i < size; i++) {
                qualityScores[i] = gyroscopeQualityScoreMapper.map(gyroscopeSequences.get(i));
            }
            calibrator.setQualityScores(qualityScores);
        }

        switch (magnetometerCalibrator.getMeasurementType()) {
            case STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY:
                if (magnetometerCalibrator.isOrderedMeasurementsRequired()) {
                    final var calibrator = (OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator)
                            magnetometerCalibrator;

                    calibrator.setMeasurements(magnetometerMeasurements);
                } else {
                    final var calibrator = (UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator)
                            magnetometerCalibrator;

                    calibrator.setMeasurements(magnetometerMeasurements);
                }

                if (magnetometerCalibrator.isQualityScoresRequired()) {
                    final var calibrator = (QualityScoredMagnetometerCalibrator) magnetometerCalibrator;

                    final var size = magnetometerMeasurements.size();
                    final var qualityScores = new double[size];
                    for (var i = 0; i < size; i++) {
                        qualityScores[i] = magnetometerQualityScoreMapper.map(magnetometerMeasurements.get(i));
                    }
                    calibrator.setQualityScores(qualityScores);
                }
                break;
            case FRAME_BODY_MAGNETIC_FLUX_DENSITY, STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY:
                // Throw exception. Cannot use frames
            default:
                throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        accelerometerCalibrator.calibrate();

        // once we have accelerometer estimations, we can set known
        // accelerometer bias and cross-coupling errors to the gyroscope calibrator
        if (gyroscopeCalibrator instanceof AccelerometerDependentGyroscopeCalibrator accelGyroCalibrator) {

            final double[] bias;
            if (accelerometerCalibrator instanceof UnknownBiasAccelerometerCalibrator unknownBiasAccelerometerCalibrator) {
                bias = unknownBiasAccelerometerCalibrator.getEstimatedBiases();

            } else if (accelerometerCalibrator instanceof KnownBiasAccelerometerCalibrator knownBiasAccelerometerCalibrator) {
                bias = knownBiasAccelerometerCalibrator.getBias();
            } else {
                bias = null;
            }

            if (bias != null) {
                accelGyroCalibrator.setAccelerometerBias(bias);
                accelGyroCalibrator.setAccelerometerMa(accelerometerCalibrator.getEstimatedMa());
            }
        }

        gyroscopeCalibrator.calibrate();
        magnetometerCalibrator.calibrate();

        final var accelerometerMse = accelerometerCalibrator.getEstimatedMse();
        final var gyroscopeMse = gyroscopeCalibrator.getEstimatedMse();
        final var magnetometerMse = magnetometerCalibrator.getEstimatedMse();

        // convert accelerometer and gyroscope mse to global mse
        final var mse = mseRule.evaluate(accelerometerMse, gyroscopeMse, magnetometerMse);
        if (mse < minMse) {
            keepBestResult(mse, thresholdFactor);
        }
        return mse;
    }

    /**
     * Initializes accelerometer, gyroscope and magnetometer measurement generator to
     * convert {@link TimedBodyKinematicsAndMagneticFluxDensity} measurements after
     * interval detection into measurements and sequences used for accelerometer,
     * gyroscope and magnetometer calibration.
     */
    private void initialize() {
        final var generatorListener = new AccelerometerGyroscopeAndMagnetometerMeasurementsGeneratorListener() {
            @Override
            public void onInitializationStarted(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onInitializationCompleted(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                    final double accelerometerBaseNoiseLevel) {
                // not needed
            }

            @Override
            public void onError(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                    final TriadStaticIntervalDetector.ErrorReason reason) {
                // not needed
            }

            @Override
            public void onStaticIntervalDetected(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onDynamicIntervalDetected(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onStaticIntervalSkipped(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onDynamicIntervalSkipped(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onGeneratedAccelerometerMeasurement(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                    final StandardDeviationBodyKinematics measurement) {
                accelerometerMeasurements.add(measurement);
            }

            @Override
            public void onGeneratedGyroscopeMeasurement(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                    final BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> sequence) {
                gyroscopeSequences.add(sequence);
            }

            @Override
            public void onGeneratedMagnetometerMeasurement(
                    final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
                    final StandardDeviationBodyMagneticFluxDensity measurement) {
                magnetometerMeasurements.add(measurement);
            }

            @Override
            public void onReset(final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator) {
                // not needed
            }
        };

        generator = new AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator(generatorListener);
    }

    /**
     * Keeps the best calibration solution found so far.
     *
     * @param mse             Estimated Mean Square Error during calibration.
     * @param thresholdFactor threshold factor to be kept.
     */
    private void keepBestResult(final double mse, final double thresholdFactor) {
        minMse = mse;
        optimalThresholdFactor = thresholdFactor;

        angularSpeedNoiseRootPsd = generator.getGyroscopeBaseNoiseLevelRootPsd();
        baseNoiseLevel = generator.getAccelerometerBaseNoiseLevel();
        threshold = generator.getThreshold();

        if (estimatedAccelerometerCovariance == null) {
            estimatedAccelerometerCovariance = new Matrix(accelerometerCalibrator.getEstimatedCovariance());
        } else {
            estimatedAccelerometerCovariance.copyFrom(accelerometerCalibrator.getEstimatedCovariance());
        }
        if (estimatedAccelerometerMa == null) {
            estimatedAccelerometerMa = new Matrix(accelerometerCalibrator.getEstimatedMa());
        } else {
            estimatedAccelerometerMa.copyFrom(accelerometerCalibrator.getEstimatedMa());
        }
        if (accelerometerCalibrator instanceof UnknownBiasAccelerometerCalibrator unknownBiasAccelerometerCalibrator) {
            estimatedAccelerometerBiases = unknownBiasAccelerometerCalibrator.getEstimatedBiases();
        } else if (accelerometerCalibrator instanceof KnownBiasAccelerometerCalibrator knownBiasAccelerometerCalibrator) {
            estimatedAccelerometerBiases = knownBiasAccelerometerCalibrator.getBias();
        }

        if (estimatedGyroscopeCovariance == null) {
            estimatedGyroscopeCovariance = new Matrix(gyroscopeCalibrator.getEstimatedCovariance());
        } else {
            estimatedGyroscopeCovariance.copyFrom(gyroscopeCalibrator.getEstimatedCovariance());
        }
        if (estimatedGyroscopeMg == null) {
            estimatedGyroscopeMg = new Matrix(gyroscopeCalibrator.getEstimatedMg());
        } else {
            estimatedGyroscopeMg.copyFrom(gyroscopeCalibrator.getEstimatedMg());
        }
        if (estimatedGyroscopeGg == null) {
            estimatedGyroscopeGg = new Matrix(gyroscopeCalibrator.getEstimatedGg());
        } else {
            estimatedGyroscopeGg.copyFrom(gyroscopeCalibrator.getEstimatedGg());
        }
        if (gyroscopeCalibrator instanceof UnknownBiasGyroscopeCalibrator unknownBiasGyroscopeCalibrator) {
            estimatedGyroscopeBiases = unknownBiasGyroscopeCalibrator.getEstimatedBiases();
        } else if (gyroscopeCalibrator instanceof KnownBiasAccelerometerCalibrator knownBiasAccelerometerCalibrator) {
            estimatedGyroscopeBiases = knownBiasAccelerometerCalibrator.getBias();
        }

        if (estimatedMagnetometerCovariance == null) {
            estimatedMagnetometerCovariance = new Matrix(magnetometerCalibrator.getEstimatedCovariance());
        } else {
            estimatedMagnetometerCovariance.copyFrom(magnetometerCalibrator.getEstimatedCovariance());
        }
        if (estimatedMagnetometerMm == null) {
            estimatedMagnetometerMm = new Matrix(magnetometerCalibrator.getEstimatedMm());
        } else {
            estimatedMagnetometerMm.copyFrom(magnetometerCalibrator.getEstimatedMm());
        }
        if (magnetometerCalibrator instanceof UnknownHardIronMagnetometerCalibrator unknownHardIronMagnetometerCalibrator) {
            estimatedMagnetometerHardIron = unknownHardIronMagnetometerCalibrator.getEstimatedHardIron();
        } else if (magnetometerCalibrator instanceof KnownHardIronMagnetometerCalibrator knownHardIronMagnetometerCalibrator) {
            estimatedMagnetometerHardIron = knownHardIronMagnetometerCalibrator.getHardIron();
        }
    }

    /**
     * Creates an acceleration instance using the provided value and unit.
     *
     * @param value value of measurement.
     * @param unit  unit of value.
     * @return created acceleration.
     */
    private Acceleration createMeasurement(final double value, final AccelerationUnit unit) {
        return new Acceleration(value, unit);
    }

    /**
     * Gets the default unit for acceleration, which is meters per
     * squared second (m/s^2).
     *
     * @return default unit for acceleration.
     */
    private AccelerationUnit getDefaultUnit() {
        return AccelerationUnit.METERS_PER_SQUARED_SECOND;
    }
}
