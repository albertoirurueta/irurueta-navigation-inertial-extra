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
package com.irurueta.navigation.inertial.calibration;

import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer;

/**
 * Utility class to create {@link INSLooselyCoupledKalmanConfig} by combining
 * different sources of estimated data.
 * Sources of data can be any measurement generator, static interval detector or
 * noise estimator implementing {@link AccelerometerNoiseRootPsdSource}
 * or {@link GyroscopeNoiseRootPsdSource}.
 */
public class INSLooselyCoupledKalmanConfigCreator {

    /**
     * A source of estimated accelerometer noise root PSD.
     */
    private AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource;

    /**
     * A source of estimated gyroscope noise root PSD.
     */
    private GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource;

    /**
     * A source of estimated accelerometer bias random walk PSD.
     */
    private AccelerometerBiasRandomWalkSource accelerometerBiasRandomWalkSource;

    /**
     * A source of estimated gyroscope bias random walk PSD.
     */
    private GyroscopeBiasRandomWalkSource gyroscopeBiasRandomWalkSource;

    /**
     * A source of position noise standard deviation.
     */
    private PositionNoiseStandardDeviationSource positionNoiseStandardDeviationSource;

    /**
     * A source of velocity noise standard deviation.
     */
    private VelocityNoiseStandardDeviationSource velocityNoiseStandardDeviationSource;

    /**
     * Constructor.
     */
    public INSLooselyCoupledKalmanConfigCreator() {
    }

    /**
     * Constructor.
     *
     * @param accelerometerNoiseRootPsdSource      a source of estimated accelerometer noise root PSD.
     * @param gyroscopeNoiseRootPsdSource          a source of estimated gyroscope noise root PSD.
     * @param accelerometerBiasRandomWalkSource    a source of estimated accelerometer bias random walk PSD.
     * @param gyroscopeBiasRandomWalkSource        a source of estimated gyroscope bias random walk PSD.
     * @param positionNoiseStandardDeviationSource a source of position noise standard deviation.
     * @param velocityNoiseStandardDeviationSource a source of velocity noise standard deviation.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource,
            final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource,
            final AccelerometerBiasRandomWalkSource accelerometerBiasRandomWalkSource,
            final GyroscopeBiasRandomWalkSource gyroscopeBiasRandomWalkSource,
            final PositionNoiseStandardDeviationSource positionNoiseStandardDeviationSource,
            final VelocityNoiseStandardDeviationSource velocityNoiseStandardDeviationSource) {
        this.accelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
        this.gyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
        this.accelerometerBiasRandomWalkSource = accelerometerBiasRandomWalkSource;
        this.gyroscopeBiasRandomWalkSource = gyroscopeBiasRandomWalkSource;
        this.positionNoiseStandardDeviationSource = positionNoiseStandardDeviationSource;
        this.velocityNoiseStandardDeviationSource = velocityNoiseStandardDeviationSource;
    }

    /**
     * Constructor.
     *
     * @param generator           an accelerometer + gyroscope measurement
     *                            generator.
     * @param randomWalkEstimator a random walk estimator.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerAndGyroscopeMeasurementsGenerator generator,
            final RandomWalkEstimator randomWalkEstimator) {
        this(generator, generator, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Constructor.
     *
     * @param generator           an accelerometer + gyroscope + magnetometer
     *                            measurement generator.
     * @param randomWalkEstimator a random walk estimator.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerGyroscopeAndMagnetometerMeasurementsGenerator generator,
            final RandomWalkEstimator randomWalkEstimator) {
        this(generator, generator, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Constructor.
     *
     * @param optimizer           an accelerometer and gyroscope threshold factor
     *                            optimizer.
     * @param randomWalkEstimator a random walk estimator.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer optimizer,
            final RandomWalkEstimator randomWalkEstimator) {
        this(optimizer, optimizer, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Constructor.
     *
     * @param optimizer           an accelerometer + gyroscope + magnetometer
     *                            threshold factor optimizer.
     * @param randomWalkEstimator a random walk estimator.
     */
    public INSLooselyCoupledKalmanConfigCreator(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer optimizer,
            final RandomWalkEstimator randomWalkEstimator) {
        this(optimizer, optimizer, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Gets the source of estimated accelerometer noise root PSD.
     *
     * @return source of estimated accelerometer noise root PSD.
     */
    public AccelerometerNoiseRootPsdSource getAccelerometerNoiseRootPsdSource() {
        return accelerometerNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated accelerometer noise root PSD.
     *
     * @param accelerometerNoiseRootPsdSource source of estimated accelerometer
     *                                        noise root PSD.
     */
    public void setAccelerometerNoiseRootPsdSource(
            final AccelerometerNoiseRootPsdSource accelerometerNoiseRootPsdSource) {
        this.accelerometerNoiseRootPsdSource = accelerometerNoiseRootPsdSource;
    }

    /**
     * Gets the source of estimated gyroscope noise root PSD.
     *
     * @return source of estimated gyroscope noise root PSD.
     */
    public GyroscopeNoiseRootPsdSource getGyroscopeNoiseRootPsdSource() {
        return gyroscopeNoiseRootPsdSource;
    }

    /**
     * Sets source of estimated gyroscope noise root PSD.
     *
     * @param gyroscopeNoiseRootPsdSource source of estimated gyroscope noise
     *                                    root PSD.
     */
    public void sstGyroscopeNoiseRootPsdSource(final GyroscopeNoiseRootPsdSource gyroscopeNoiseRootPsdSource) {
        this.gyroscopeNoiseRootPsdSource = gyroscopeNoiseRootPsdSource;
    }

    /**
     * Gets the source of estimated accelerometer bias random walk PSD.
     *
     * @return source of estimated accelerometer bias random walk PSD.
     */
    public AccelerometerBiasRandomWalkSource getAccelerometerBiasRandomWalkSource() {
        return accelerometerBiasRandomWalkSource;
    }

    /**
     * Sets source of estimated accelerometer bias random walk PSD.
     *
     * @param accelerometerBiasRandomWalkSource source of estimated accelerometer
     *                                          bias random walk PSD.
     */
    public void setAccelerometerBiasRandomWalkSource(
            final AccelerometerBiasRandomWalkSource accelerometerBiasRandomWalkSource) {
        this.accelerometerBiasRandomWalkSource = accelerometerBiasRandomWalkSource;
    }

    /**
     * Gets the source of estimated gyroscope bias random walk PSD.
     *
     * @return source of estimated gyroscope bias random walk PSD.
     */
    public GyroscopeBiasRandomWalkSource getGyroscopeBiasRandomWalkSource() {
        return gyroscopeBiasRandomWalkSource;
    }

    /**
     * Sets source of estimated gyroscope bias random walk PSD.
     *
     * @param gyroscopeBiasRandomWalkSource source of estimated gyroscope bias
     *                                      random walk PSD.
     */
    public void setGyroscopeBiasRandomWalkSource(final GyroscopeBiasRandomWalkSource gyroscopeBiasRandomWalkSource) {
        this.gyroscopeBiasRandomWalkSource = gyroscopeBiasRandomWalkSource;
    }

    /**
     * Gets the source of position noise standard deviation.
     *
     * @return source of position noise standard deviation.
     */
    public PositionNoiseStandardDeviationSource getPositionNoiseStandardDeviationSource() {
        return positionNoiseStandardDeviationSource;
    }

    /**
     * Sets source of position noise standard deviation.
     *
     * @param positionUncertaintySource source of position noise standard
     *                                  deviation.
     */
    public void setPositionNoiseStandardDeviationSource(
            final PositionNoiseStandardDeviationSource positionUncertaintySource) {
        positionNoiseStandardDeviationSource = positionUncertaintySource;
    }

    /**
     * Gets the source of velocity noise standard deviation.
     *
     * @return source of velocity noise standard deviation.
     */
    public VelocityNoiseStandardDeviationSource getVelocityNoiseStandardDeviationSource() {
        return velocityNoiseStandardDeviationSource;
    }

    /**
     * Sets source of velocity noise standard deviation.
     *
     * @param velocityUncertaintySource source of velocity noise standard
     *                                  deviation.
     */
    public void setVelocityNoiseStandardDeviationSource(
            final VelocityNoiseStandardDeviationSource velocityUncertaintySource) {
        velocityNoiseStandardDeviationSource = velocityUncertaintySource;
    }

    /**
     * Indicates whether all sources have been provided to be able to
     * create a {@link INSLooselyCoupledKalmanConfig} instance.
     *
     * @return true if the creator is ready, false otherwise.
     */
    public boolean isReady() {
        return accelerometerNoiseRootPsdSource != null
                && gyroscopeNoiseRootPsdSource != null
                && accelerometerBiasRandomWalkSource != null
                && gyroscopeBiasRandomWalkSource != null
                && positionNoiseStandardDeviationSource != null
                && velocityNoiseStandardDeviationSource != null;
    }

    /**
     * Creates a {@link INSLooselyCoupledKalmanConfig} instance containing estimated
     * parameters during calibration.
     *
     * @return instance containing configuration data.
     * @throws NotReadyException if the creator is not ready.
     */
    public INSLooselyCoupledKalmanConfig create() throws NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var gyroNoisePsd = Math.pow(gyroscopeNoiseRootPsdSource.getGyroscopeBaseNoiseLevelRootPsd(), 2.0);
        final var accelNoisePsd = Math.pow(accelerometerNoiseRootPsdSource.getAccelerometerBaseNoiseLevelRootPsd(),
                2.0);
        final var accelRandomWalkBiasPsd = accelerometerBiasRandomWalkSource.getAccelerometerBiasPSD();
        final var gyroRandomWalkBiasPsd = gyroscopeBiasRandomWalkSource.getGyroBiasPSD();
        final var positionNoiseSd = positionNoiseStandardDeviationSource.getPositionNoiseStandardDeviation();
        final var velocityNoiseSd = velocityNoiseStandardDeviationSource.getVelocityNoiseStandardDeviation();
        return new INSLooselyCoupledKalmanConfig(gyroNoisePsd, accelNoisePsd, accelRandomWalkBiasPsd,
                gyroRandomWalkBiasPsd, positionNoiseSd, velocityNoiseSd);
    }
}
