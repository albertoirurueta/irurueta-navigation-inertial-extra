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
import com.irurueta.navigation.inertial.BodyKinematicsAndMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.AccelerometerNoiseRootPsdSource;
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.MagnetometerMeasurementsGeneratorListener;
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
 * Optimizes the threshold factor for interval detection of magnetometer data based
 * on results of calibration.
 * Implementations of this class will attempt to find the best threshold factor
 * between the provided range of values.
 * Only magnetometer calibrators based on unknown orientation are supported, in other terms,
 * calibrators must be {@link MagnetometerNonLinearCalibrator} and must support
 * {@link MagnetometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY}.
 */
public abstract class MagnetometerIntervalDetectorThresholdFactorOptimizer extends
        IntervalDetectorThresholdFactorOptimizer<BodyKinematicsAndMagneticFluxDensity,
                MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource> implements
        AccelerometerNoiseRootPsdSource {

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
     * Magnetometer calibrator.
     */
    private MagnetometerNonLinearCalibrator calibrator;

    /**
     * A measurement generator for magnetometer calibrators.
     */
    private MagnetometerMeasurementsGenerator generator;

    /**
     * Generated measurements to be used for magnetometer calibration.
     */
    private List<StandardDeviationBodyMagneticFluxDensity> measurements;

    /**
     * Mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements
     * into quality scores.
     */
    private QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> qualityScoreMapper =
            new DefaultMagnetometerQualityScoreMapper();

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
     * Estimated covariance matrix for estimated parameters.
     */
    private Matrix estimatedCovariance;

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
    private Matrix estimatedMm;

    /**
     * Estimated magnetometer hard-iron biases for each magnetometer axis
     * expressed in Teslas (T).
     */
    private double[] estimatedHardIron;

    /**
     * Constructor.
     */
    protected MagnetometerIntervalDetectorThresholdFactorOptimizer() {
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    protected MagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
        initialize();
    }

    /**
     * Constructor.
     *
     * @param calibrator a magnetometer calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    protected MagnetometerIntervalDetectorThresholdFactorOptimizer(final MagnetometerNonLinearCalibrator calibrator) {
        try {
            setCalibrator(calibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     * @param calibrator a magnetometer calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    protected MagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final MagnetometerNonLinearCalibrator calibrator) {
        super(dataSource);
        try {
            setCalibrator(calibrator);
        } catch (final LockedException ignore) {
            // never happens
        }
        initialize();
    }

    /**
     * Gets provided magnetometer calibrator to be used to optimize its Mean Square Error (MSE).
     *
     * @return magnetometer calibrator to be used to optimize its MSE.
     */
    public MagnetometerNonLinearCalibrator getCalibrator() {
        return calibrator;
    }

    /**
     * Sets a magnetometer calibrator to be used to optimize its Mean Square Error.
     *
     * @param calibrator magnetometer calibrator to be used to optimize its MSE.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public void setCalibrator(final MagnetometerNonLinearCalibrator calibrator) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (calibrator != null && calibrator.getMeasurementType()
                != MagnetometerCalibratorMeasurementType.STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY) {
            throw new IllegalArgumentException();
        }

        this.calibrator = calibrator;
    }

    /**
     * Gets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements into
     * quality scores.
     *
     * @return mapper to convert measurements into quality scores.
     */
    public QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> getQualityScoreMapper() {
        return qualityScoreMapper;
    }

    /**
     * Sets mapper to convert {@link StandardDeviationBodyMagneticFluxDensity} measurements into
     * quality scores.
     *
     * @param qualityScoreMapper mapper to convert measurements into quality scores.
     * @throws LockedException if optimizer is already running.
     */
    public void setQualityScoreMapper(
            final QualityScoreMapper<StandardDeviationBodyMagneticFluxDensity> qualityScoreMapper)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.qualityScoreMapper = qualityScoreMapper;
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
        return super.isReady() && calibrator != null && qualityScoreMapper != null;
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
    public void setMinStaticSamples(final int minStaticSamples)
            throws LockedException {
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
     * Gets accelerometer base noise level PSD (Power Spectral Density)
     * expressed in (m^2 * s^-3) of the best solution that has been found.
     *
     * @return accelerometer base noise level PSD of the best solution that has
     * been found.
     */
    public double getAccelerometerBaseNoiseLevelPsd() {
        return baseNoiseLevel * baseNoiseLevel * getTimeInterval();
    }

    /**
     * Gets accelerometer base noise level root PSD (Power Spectral Density)
     * expressed in (m * s^-1.5) of the best solution that has been found.
     *
     * @return accelerometer base noise level root PSD of the best solution that has
     * been found.
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
     * Gets the array containing x,y,z components of estimated magnetometer
     * hard-iron biases expressed in Teslas (T).
     *
     * @return array containing x,y,z components of estimated magnetometer
     * hard-iron biases.
     */
    public double[] getEstimatedHardIron() {
        return estimatedHardIron;
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
    public Matrix getEstimatedMm() {
        return estimatedMm;
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
        if (measurements == null) {
            measurements = new ArrayList<>();
        } else {
            measurements.clear();
        }

        generator.reset();
        generator.setThresholdFactor(thresholdFactor);

        var count = dataSource.count();
        var failed = false;
        for (var i = 0; i < count; i++) {
            final var bodyKinematics = dataSource.getAt(i);
            if (!generator.process(bodyKinematics)) {
                failed = true;
                break;
            }
        }

        if (failed || generator.getStatus() == TriadStaticIntervalDetector.Status.FAILED) {
            // interval detection failed
            return Double.MAX_VALUE;
        }

        // check that enough measurements have been obtained
        if (measurements.size() < calibrator.getMinimumRequiredMeasurements()) {
            return Double.MAX_VALUE;
        }

        // set calibrator measurements
        switch (calibrator.getMeasurementType()) {
            case STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY:
                if (calibrator.isOrderedMeasurementsRequired()) {
                    final var cal =
                            (OrderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator) this.calibrator;

                    cal.setMeasurements(measurements);
                } else {
                    final var cal =
                            (UnorderedStandardDeviationBodyMagneticFluxDensityMagnetometerCalibrator) this.calibrator;

                    cal.setMeasurements(measurements);
                }

                if (calibrator.isQualityScoresRequired()) {
                    final var cal = (QualityScoredMagnetometerCalibrator) this.calibrator;

                    final var size = measurements.size();
                    final var qualityScores = new double[size];
                    for (var i = 0; i < size; i++) {
                        qualityScores[i] = qualityScoreMapper.map(measurements.get(i));
                    }
                    cal.setQualityScores(qualityScores);
                }
                break;
            case FRAME_BODY_MAGNETIC_FLUX_DENSITY, STANDARD_DEVIATION_FRAME_BODY_MAGNETIC_FLUX_DENSITY:
                // Throw exception. Cannot use frames
            default:
                throw new IntervalDetectorThresholdFactorOptimizerException();
        }

        calibrator.calibrate();

        final var mse = calibrator.getEstimatedMse();
        if (mse < minMse) {
            keepBestResult(mse, thresholdFactor);
        }
        return mse;
    }

    /**
     * Initializes magnetometer measurement generator to convert
     * body kinematics and magnetic flux measurements after interval detection into
     * measurements used for magnetometer calibration.
     */
    private void initialize() {
        final var generatorListener = new MagnetometerMeasurementsGeneratorListener() {
            @Override
            public void onInitializationStarted(final MagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onInitializationCompleted(
                    final MagnetometerMeasurementsGenerator generator, final double baseNoiseLevel) {
                // not needed
            }

            @Override
            public void onError(
                    final MagnetometerMeasurementsGenerator generator,
                    final TriadStaticIntervalDetector.ErrorReason reason) {
                // not needed
            }

            @Override
            public void onStaticIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onDynamicIntervalDetected(final MagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onStaticIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onDynamicIntervalSkipped(final MagnetometerMeasurementsGenerator generator) {
                // not needed
            }

            @Override
            public void onGeneratedMeasurement(
                    final MagnetometerMeasurementsGenerator generator,
                    final StandardDeviationBodyMagneticFluxDensity measurement) {
                measurements.add(measurement);
            }

            @Override
            public void onReset(final MagnetometerMeasurementsGenerator generator) {
                // not needed
            }
        };

        generator = new MagnetometerMeasurementsGenerator(generatorListener);
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

        baseNoiseLevel = generator.getAccelerometerBaseNoiseLevel();
        threshold = generator.getThreshold();

        if (estimatedCovariance == null) {
            estimatedCovariance = new Matrix(calibrator.getEstimatedCovariance());
        } else {
            estimatedCovariance.copyFrom(calibrator.getEstimatedCovariance());
        }
        if (estimatedMm == null) {
            estimatedMm = new Matrix(calibrator.getEstimatedMm());
        } else {
            estimatedMm.copyFrom(calibrator.getEstimatedMm());
        }
        if (calibrator instanceof UnknownHardIronMagnetometerCalibrator unknownHardIronMagnetometerCalibrator) {
            estimatedHardIron = unknownHardIronMagnetometerCalibrator.getEstimatedHardIron();
        } else if (calibrator instanceof KnownHardIronMagnetometerCalibrator knownHardIronMagnetometerCalibrator) {
            estimatedHardIron = knownHardIronMagnetometerCalibrator.getHardIron();
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
