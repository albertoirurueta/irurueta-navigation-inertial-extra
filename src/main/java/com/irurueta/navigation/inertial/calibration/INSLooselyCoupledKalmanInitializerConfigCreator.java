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
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;

/**
 * Utility class to create {@link INSLooselyCoupledKalmanInitializerConfig} by combining
 * different sources of estimated data.
 * Sources of data can be any accelerometer calibrator implementing
 * {@link AccelerometerBiasUncertaintySource}, or any gyroscope calibrator implementing
 * {@link GyroscopeBiasUncertaintySource}.
 */
public class INSLooselyCoupledKalmanInitializerConfigCreator {

    /**
     * A source of estimated accelerometer bias uncertainty.
     */
    private AccelerometerBiasUncertaintySource accelerometerBiasUncertaintySource;

    /**
     * A source of estimated gyroscope bias uncertainty.
     */
    private GyroscopeBiasUncertaintySource gyroscopeBiasUncertaintySource;

    /**
     * A source of attitude uncertainty.
     */
    private AttitudeUncertaintySource attitudeUncertaintySource;

    /**
     * A source of velocity uncertainty.
     */
    private VelocityUncertaintySource velocityUncertaintySource;

    /**
     * A source of position uncertainty.
     */
    private PositionUncertaintySource positionUncertaintySource;

    /**
     * Constructor.
     */
    public INSLooselyCoupledKalmanInitializerConfigCreator() {
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiasUncertaintySource a source of estimated accelerometer bias uncertainty.
     * @param gyroscopeBiasUncertaintySource     a source of estimated gyroscope bias uncertainty.
     * @param attitudeUncertaintySource          a source of attitude uncertainty.
     * @param velocityUncertaintySource          a source of velocity uncertainty.
     * @param positionUncertaintySource          a source of position uncertainty.
     */
    public INSLooselyCoupledKalmanInitializerConfigCreator(
            final AccelerometerBiasUncertaintySource accelerometerBiasUncertaintySource,
            final GyroscopeBiasUncertaintySource gyroscopeBiasUncertaintySource,
            final AttitudeUncertaintySource attitudeUncertaintySource,
            final VelocityUncertaintySource velocityUncertaintySource,
            final PositionUncertaintySource positionUncertaintySource) {
        this.accelerometerBiasUncertaintySource = accelerometerBiasUncertaintySource;
        this.gyroscopeBiasUncertaintySource = gyroscopeBiasUncertaintySource;
        this.attitudeUncertaintySource = attitudeUncertaintySource;
        this.velocityUncertaintySource = velocityUncertaintySource;
        this.positionUncertaintySource = positionUncertaintySource;
    }

    /**
     * Constructor.
     *
     * @param accelerometerBiasUncertaintySource a source of estimated accelerometer bias uncertainty.
     * @param gyroscopeBiasUncertaintySource     a source of estimated gyroscope bias uncertainty.
     * @param randomWalkEstimator                a random walk estimator.
     */
    public INSLooselyCoupledKalmanInitializerConfigCreator(
            final AccelerometerBiasUncertaintySource accelerometerBiasUncertaintySource,
            final GyroscopeBiasUncertaintySource gyroscopeBiasUncertaintySource,
            final RandomWalkEstimator randomWalkEstimator) {
        this(accelerometerBiasUncertaintySource, gyroscopeBiasUncertaintySource, randomWalkEstimator,
                randomWalkEstimator, randomWalkEstimator);
    }

    /**
     * Gets the source of estimated accelerometer bias uncertainty.
     *
     * @return source of estimated accelerometer bias uncertainty.
     */
    public AccelerometerBiasUncertaintySource getAccelerometerBiasUncertaintySource() {
        return accelerometerBiasUncertaintySource;
    }

    /**
     * Sets source of estimated accelerometer bias uncertainty.
     *
     * @param accelerometerBiasUncertaintySource source of estimated accelerometer bias uncertainty.
     */
    public void setAccelerometerBiasUncertaintySource(
            final AccelerometerBiasUncertaintySource accelerometerBiasUncertaintySource) {
        this.accelerometerBiasUncertaintySource = accelerometerBiasUncertaintySource;
    }

    /**
     * Gets the source of estimated gyroscope bias uncertainty.
     *
     * @return source of estimated gyroscope bias uncertainty.
     */
    public GyroscopeBiasUncertaintySource getGyroscopeBiasUncertaintySource() {
        return gyroscopeBiasUncertaintySource;
    }

    /**
     * Sets source of estimated gyroscope bias uncertainty.
     *
     * @param gyroscopeBiasUncertaintySource source of estimated gyroscope bias uncertainty.
     */
    public void setGyroscopeBiasUncertaintySource(final GyroscopeBiasUncertaintySource gyroscopeBiasUncertaintySource) {
        this.gyroscopeBiasUncertaintySource = gyroscopeBiasUncertaintySource;
    }

    /**
     * Gets source of attitude uncertainty.
     *
     * @return source of attitude uncertainty.
     */
    public AttitudeUncertaintySource getAttitudeUncertaintySource() {
        return attitudeUncertaintySource;
    }

    /**
     * Sets source of attitude uncertainty.
     *
     * @param attitudeUncertaintySource source of attitude uncertainty.
     */
    public void setAttitudeUncertaintySource(final AttitudeUncertaintySource attitudeUncertaintySource) {
        this.attitudeUncertaintySource = attitudeUncertaintySource;
    }

    /**
     * Gets the source of velocity uncertainty.
     *
     * @return source of velocity uncertainty.
     */
    public VelocityUncertaintySource getVelocityUncertaintySource() {
        return velocityUncertaintySource;
    }

    /**
     * Sets source of velocity uncertainty.
     *
     * @param velocityUncertaintySource source of velocity uncertainty.
     */
    public void setVelocityUncertaintySource(final VelocityUncertaintySource velocityUncertaintySource) {
        this.velocityUncertaintySource = velocityUncertaintySource;
    }

    /**
     * Gets the source of position uncertainty.
     *
     * @return source of position uncertainty.
     */
    public PositionUncertaintySource getPositionUncertaintySource() {
        return positionUncertaintySource;
    }

    /**
     * Sets source of position uncertainty.
     *
     * @param positionUncertaintySource source of position uncertainty.
     */
    public void setPositionUncertaintySource(final PositionUncertaintySource positionUncertaintySource) {
        this.positionUncertaintySource = positionUncertaintySource;
    }

    /**
     * Indicates whether all sources have been provided to be able to
     * create a {@link INSLooselyCoupledKalmanInitializerConfig} instance.
     *
     * @return true if the creator is ready, false otherwise.
     */
    public boolean isReady() {
        return accelerometerBiasUncertaintySource != null
                && accelerometerBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm() != null
                && gyroscopeBiasUncertaintySource != null
                && gyroscopeBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm() != null
                && attitudeUncertaintySource != null
                && velocityUncertaintySource != null
                && positionUncertaintySource != null;
    }

    /**
     * Creates a {@link INSLooselyCoupledKalmanInitializerConfig} instance containing estimated
     * parameters during calibration.
     *
     * @return instance containing initial configuration data.
     * @throws NotReadyException if the creator is not ready.
     */
    public INSLooselyCoupledKalmanInitializerConfig create() throws NotReadyException {
        if (!isReady()) {
            throw new NotReadyException();
        }

        final var attitudeUncertainty = attitudeUncertaintySource.getAttitudeUncertainty();
        final var velocityUncertainty = velocityUncertaintySource.getVelocityUncertainty();
        final var positionUncertainty = positionUncertaintySource.getPositionUncertainty();
        final var accelerationBiasUncertainty =
                accelerometerBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm();
        final var gyroBiasUncertainty = gyroscopeBiasUncertaintySource.getEstimatedBiasStandardDeviationNorm();

        return new INSLooselyCoupledKalmanInitializerConfig(attitudeUncertainty, velocityUncertainty,
                positionUncertainty, accelerationBiasUncertainty, gyroBiasUncertainty);
    }
}
