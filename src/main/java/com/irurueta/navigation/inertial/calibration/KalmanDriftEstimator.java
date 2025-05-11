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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.INSException;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanFilteredEstimator;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanState;

/**
 * Estimates accumulated drift in body orientation, position and velocity per
 * unit of time using an INS Kalman filtered solution.
 */
public class KalmanDriftEstimator extends DriftEstimator {

    /**
     * Configuration parameters for INS Loosely Coupled Kalman filter obtained
     * through calibration.
     */
    private INSLooselyCoupledKalmanConfig kalmanConfig;

    /**
     * Configuration parameters determining initial system noise covariance matrix.
     */
    private INSLooselyCoupledKalmanInitializerConfig initConfig;

    /**
     * Internal INS loosely coupled Kalman filtered estimator to estimate
     * accumulated body position, velocity and orientation.
     */
    private INSLooselyCoupledKalmanFilteredEstimator kalmanEstimator;

    /**
     * Current Kalman filter state.
     */
    private INSLooselyCoupledKalmanState state;

    /**
     * Constructor.
     */
    public KalmanDriftEstimator() {
        super();
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events.
     */
    public KalmanDriftEstimator(final DriftEstimatorListener listener) {
        super(listener);
    }

    /**
     * Constructor.
     *
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     */
    public KalmanDriftEstimator(
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig) {
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     */
    public KalmanDriftEstimator(
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) {
        super(listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig) {
        super(referenceFrame);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) {
        super(referenceFrame, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig) {
        super(referenceFrame);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) {
        super(referenceFrame, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg, gg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, gg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(ba, ma, bg, mg, gg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param ba           acceleration bias to be set expressed in meters per squared second
     *                     (m/s`2). Must be 3x1.
     * @param ma           acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg           angular speed bias to be set expressed in radians per second
     *                     (rad/s). Must be 3x1.
     * @param mg           angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg           angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig configuration parameters obtained through calibration.
     * @param initConfig   configuration parameters determining initial system
     *                     noise covariance matrix.
     * @param listener     listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(ba, ma, bg, mg, gg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig)
            throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param ba             acceleration bias to be set expressed in meters per squared second
     *                       (m/s`2). Must be 3x1.
     * @param ma             acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg             angular speed bias to be set expressed in radians per second
     *                       (rad/s). Must be 3x1.
     * @param mg             angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg             angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param kalmanConfig   configuration parameters obtained through calibration.
     * @param initConfig     configuration parameters determining initial system
     *                       noise covariance matrix.
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any of the provided matrices are not 3x3.
     */
    public KalmanDriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final INSLooselyCoupledKalmanConfig kalmanConfig,
            final INSLooselyCoupledKalmanInitializerConfig initConfig,
            final DriftEstimatorListener listener) throws AlgebraException {
        super(referenceFrame, ba, ma, bg, mg, gg, listener);
        this.kalmanConfig = kalmanConfig;
        this.initConfig = initConfig;
    }

    /**
     * Gets configuration parameters for INS Loosely Coupled Kalman filter obtained
     * through calibration.
     *
     * @return configuration parameters for Kalman filter or null.
     */
    public INSLooselyCoupledKalmanConfig getKalmanConfig() {
        return kalmanConfig;
    }

    /**
     * Sets configuration parameters for INS Loosely Coupled Kalman filter obtained
     * through calibration.
     *
     * @param kalmanConfig configuration parameters for Kalman filter.
     * @throws LockedException if estimator is already running.
     */
    public void setKalmanConfig(final INSLooselyCoupledKalmanConfig kalmanConfig) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.kalmanConfig = kalmanConfig;
    }

    /**
     * Gets configuration parameters determining initial system noise covariance
     * matrix.
     *
     * @return configuration parameters determining initial system noise covariance
     * matrix or null.
     */
    public INSLooselyCoupledKalmanInitializerConfig getInitConfig() {
        return initConfig;
    }

    /**
     * Sets configuration parameters determining initial system noise covariance
     * matrix.
     *
     * @param initConfig configuration parameters determining initial system noise
     *                   covariance matrix.
     * @throws LockedException if estimator is already running.
     */
    public void setInitConfig(final INSLooselyCoupledKalmanInitializerConfig initConfig) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.initConfig = initConfig;
    }

    /**
     * Indicates if estimator is ready to start processing additional kinematics
     * measurements.
     *
     * @return true if ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && kalmanConfig != null && initConfig != null;
    }

    /**
     * Adds a sample of measured body kinematics (accelerometer and gyroscope readings)
     * obtained from an IMU, fixes their values and uses fixed values to estimate
     * current drift and their average values.
     *
     * @param kinematics measured body kinematics.
     * @throws LockedException          if estimator is currently running.
     * @throws NotReadyException        if estimator is not ready.
     * @throws DriftEstimationException if estimation fails for some reason.
     */
    @Override
    public void addBodyKinematics(final BodyKinematics kinematics) throws LockedException, NotReadyException,
            DriftEstimationException {
        if (isRunning()) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            running = true;

            if (numberOfProcessedSamples == 0) {
                if (listener != null) {
                    listener.onStart(this);
                }

                initialize();
            }

            if (fixKinematics) {
                // fix kinematics and update kalman filter
                fixer.fix(kinematics, fixedKinematics);
            } else {
                // only update kalman filter
                fixedKinematics.copyFrom(kinematics);
            }

            final var timestamp = getElapsedTimeSeconds();
            kalmanEstimator.update(fixedKinematics, timestamp);

            if (state == null) {
                state = kalmanEstimator.getState();
            } else {
                kalmanEstimator.getState(state);
            }

            // estimate drift values
            computeCurrentPositionDrift();
            computeCurrentVelocityDrift();
            computeCurrentOrientationDrift();

            numberOfProcessedSamples++;

            if (listener != null) {
                listener.onBodyKinematicsAdded(this, kinematics, fixedKinematics);
            }
        } catch (final AlgebraException | InvalidRotationMatrixException | INSException e) {
            throw new DriftEstimationException(e);
        } finally {
            running = false;
        }
    }

    /**
     * Resets this estimator to its initial state.
     *
     * @throws LockedException if estimator is currently running.
     */
    @Override
    public void reset() throws LockedException {
        if (isRunning()) {
            throw new LockedException();
        }

        kalmanEstimator = null;
        state = null;
        running = false;

        super.reset();
    }

    /**
     * Gets state of internal Kalman filter containing current body position,
     * orientation and velocity after last provided body kinematics measurement.
     *
     * @return state of internal Kalman filter.
     */
    public INSLooselyCoupledKalmanState getState() {
        return kalmanEstimator != null ? kalmanEstimator.getState() : null;
    }

    /**
     * Gets state of internal Kalman filter containing current body position,
     * orientation and velocity after last provided body kinematics measurement.
     *
     * @param result instance where the result will be stored.
     * @return true if the internal Kalman filter state is available and the result is
     * updated, false otherwise.
     */
    public boolean getState(final INSLooselyCoupledKalmanState result) {
        if (kalmanEstimator != null) {
            return kalmanEstimator.getState(result);
        } else {
            return false;
        }
    }

    /**
     * Initializes internal frame and Kalman estimator when the first body kinematics
     * measurement is provided.
     *
     * @throws InvalidRotationMatrixException if orientation cannot be determined for
     *                                        numerical reasons.
     */
    private void initialize() throws InvalidRotationMatrixException {
        final var c = referenceFrame.getCoordinateTransformation();
        c.asRotation(refQ);
        refQ.inverse(invRefQ);

        frame.copyFrom(referenceFrame);

        kalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, frame);
    }

    /**
     * Computes current position drift.
     */
    @Override
    protected void computeCurrentPositionDrift() {
        final var initX = referenceFrame.getX();
        final var initY = referenceFrame.getY();
        final var initZ = referenceFrame.getZ();

        final var currentX = state.getX();
        final var currentY = state.getY();
        final var currentZ = state.getZ();

        final var diffX = currentX - initX;
        final var diffY = currentY - initY;
        final var diffZ = currentZ - initZ;

        currentPositionDrift.setCoordinates(diffX, diffY, diffZ);

        currentPositionDriftMeters = currentPositionDrift.getNorm();
    }

    /**
     * Computes current velocity drift.
     */
    @Override
    protected void computeCurrentVelocityDrift() {
        final var initVx = referenceFrame.getVx();
        final var initVy = referenceFrame.getVy();
        final var initVz = referenceFrame.getVz();

        final var currentVx = state.getVx();
        final var currentVy = state.getVy();
        final var currentVz = state.getVz();

        final var diffVx = currentVx - initVx;
        final var diffVy = currentVy - initVy;
        final var diffVz = currentVz - initVz;

        currentVelocityDrift.setCoordinates(diffVx, diffVy, diffVz);

        currentVelocityDriftMetersPerSecond = currentVelocityDrift.getNorm();
    }

    /**
     * Computes current orientation drift.
     *
     * @throws InvalidRotationMatrixException if rotation cannot be accurately
     *                                        estimated.
     */
    @Override
    protected void computeCurrentOrientationDrift() throws InvalidRotationMatrixException {
        currentC = state.getBodyToEcefCoordinateTransformationMatrix();

        q.fromMatrix(currentC);
        q.combine(invRefQ);

        currentOrientationDriftRadians = q.getRotationAngle();
    }
}
