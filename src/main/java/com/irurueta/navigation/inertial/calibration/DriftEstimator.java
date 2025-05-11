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
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.Rotation3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.ECEFVelocity;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.frames.NEDVelocity;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.units.Acceleration;
import com.irurueta.units.AccelerationUnit;
import com.irurueta.units.Angle;
import com.irurueta.units.AngleUnit;
import com.irurueta.units.AngularSpeed;
import com.irurueta.units.AngularSpeedUnit;
import com.irurueta.units.Distance;
import com.irurueta.units.DistanceUnit;
import com.irurueta.units.Speed;
import com.irurueta.units.SpeedUnit;
import com.irurueta.units.Time;
import com.irurueta.units.TimeConverter;
import com.irurueta.units.TimeUnit;

/**
 * Estimates accumulated drift in body orientation, position and velocity per
 * unit of time.
 * This estimator must be executed while the body where the IMU is placed remains
 * static.
 */
public class DriftEstimator {

    /**
     * Default time interval between kinematics samples expressed in seconds (s).
     */
    public static final double DEFAULT_TIME_INTERVAL_SECONDS = 0.02;

    /**
     * Indicates whether this estimator is running.
     */
    protected boolean running;

    /**
     * Number of processed body kinematics samples.
     */
    protected int numberOfProcessedSamples;

    /**
     * Time interval expressed in seconds (s) between body kinematics samples.
     */
    protected double timeInterval = DEFAULT_TIME_INTERVAL_SECONDS;

    /**
     * Fixes body kinematics measurements using accelerometer and gyroscope
     * calibration data to fix measurements.
     */
    protected final BodyKinematicsFixer fixer = new BodyKinematicsFixer();

    /**
     * Instance containing the last fixed body kinematics to be reused.
     */
    protected final BodyKinematics fixedKinematics = new BodyKinematics();

    /**
     * Indicates whether measured kinematics must be fixed or not.
     * When enabled, provided calibration data is used; otherwise it is
     * ignored.
     * By default, this is enabled.
     */
    protected boolean fixKinematics = true;

    /**
     * Listener to handle events raised by this estimator.
     */
    protected DriftEstimatorListener listener;

    /**
     * Initial frame containing body position, velocity and orientation expressed
     * in ECEF coordinates before starting drift estimation.
     */
    protected ECEFFrame referenceFrame;

    /**
     * Contains the current frame after one navigation step.
     * This is reused for efficiency.
     */
    protected final ECEFFrame frame = new ECEFFrame();

    /**
     * Contains orientation of a reference frame.
     * This is reused for efficiency.
     */
    protected final Quaternion refQ = new Quaternion();

    /**
     * Contains orientation inverse of the reference frame.
     * This is reused for efficiency.
     */
    protected final Quaternion invRefQ = new Quaternion();

    /**
     * Contains current frame orientation drift.
     * This is reused for efficiency.
     */
    protected final Quaternion q = new Quaternion();

    /**
     * Contains current position drift.
     */
    protected final ECEFPosition currentPositionDrift = new ECEFPosition();

    /**
     * Contains current velocity drift.
     */
    protected final ECEFVelocity currentVelocityDrift = new ECEFVelocity();

    /**
     * Contains current orientation expressed as a 3D rotation matrix.
     */
    protected Matrix currentC;

    /**
     * Current position drift expressed in meters (m).
     */
    protected double currentPositionDriftMeters;

    /**
     * Current velocity drift expressed in meters per second (m/s).
     */
    protected double currentVelocityDriftMetersPerSecond;

    /**
     * Current orientation drift expressed in radians (rad).
     */
    protected double currentOrientationDriftRadians;

    /**
     * Constructor.
     */
    public DriftEstimator() {
    }

    /**
     * Constructor.
     *
     * @param listener listener to handle events.
     */
    public DriftEstimator(final DriftEstimatorListener listener) {
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     */
    public DriftEstimator(final ECEFFrame referenceFrame) {
        this.referenceFrame = referenceFrame;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in ECEF coordinates.
     * @param listener       listener to handle events.
     */
    public DriftEstimator(final ECEFFrame referenceFrame,
                          final DriftEstimatorListener listener) {
        this.referenceFrame = referenceFrame;
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     */
    public DriftEstimator(final NEDFrame referenceFrame) {
        try {
            setReferenceNedFrame(referenceFrame);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param referenceFrame initial frame containing body position, velocity and
     *                       orientation expressed in NED coordinates.
     * @param listener       listener to handle events.
     */
    public DriftEstimator(final NEDFrame referenceFrame,
                          final DriftEstimatorListener listener) {
        this(referenceFrame);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set.
     * @param ma acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set.
     * @param mg angular speed cross-coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(final AccelerationTriad ba,
                          final Matrix ma,
                          final AngularSpeedTriad bg,
                          final Matrix mg) throws AlgebraException {
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set.
     * @param ma       acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set.
     * @param mg       angular speed cross-coupling errors matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set.
     * @param ma acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set.
     * @param mg angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg);
        try {
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set.
     * @param ma       acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set.
     * @param mg       angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg       angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set expressed in meters per squared second
     *           (m/s`2). Must be 3x1.
     * @param ma acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set expressed in radians per second
     *           (rad/s). Must be 3x1.
     * @param mg angular speed cross-coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg) throws AlgebraException {
        try {
            setAccelerationBias(ba);
            setAccelerationCrossCouplingErrors(ma);
            setAngularSpeedBias(bg);
            setAngularSpeedCrossCouplingErrors(mg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set expressed in meters per squared second
     *                 (m/s`2). Must be 3x1.
     * @param ma       acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set expressed in radians per second
     *                 (rad/s). Must be 3x1.
     * @param mg       angular speed cross-coupling errors matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg);
        this.listener = listener;
    }

    /**
     * Constructor.
     *
     * @param ba acceleration bias to be set expressed in meters per squared second
     *           (m/s`2). Must be 3x1.
     * @param ma acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg angular speed bias to be set expressed in radians per second
     *           (rad/s). Must be 3x1.
     * @param mg angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg);
        try {
            setAngularSpeedGDependantCrossBias(gg);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param ba       acceleration bias to be set expressed in meters per squared second
     *                 (m/s`2). Must be 3x1.
     * @param ma       acceleration cross-coupling errors matrix. Must be 3x3.
     * @param bg       angular speed bias to be set expressed in radians per second
     *                 (rad/s). Must be 3x1.
     * @param mg       angular speed cross-coupling errors matrix. Must be 3x3.
     * @param gg       angular speed g-dependent cross-biases matrix. Must be 3x3.
     * @param listener listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        this.listener = listener;
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(final ECEFFrame referenceFrame,
                          final AccelerationTriad ba,
                          final Matrix ma,
                          final AngularSpeedTriad bg,
                          final Matrix mg) throws AlgebraException {
        this(ba, ma, bg, mg);
        this.referenceFrame = referenceFrame;
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, listener);
        this.referenceFrame = referenceFrame;
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        this.referenceFrame = referenceFrame;
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg, listener);
        this.referenceFrame = referenceFrame;
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg) throws AlgebraException {
        this(ba, ma, bg, mg);
        this.referenceFrame = referenceFrame;
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, listener);
        this.referenceFrame = referenceFrame;
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(ba, ma, bg, mg, gg);
        this.referenceFrame = referenceFrame;
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final ECEFFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(ba, ma, bg, mg, gg, listener);
        this.referenceFrame = referenceFrame;
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(final NEDFrame referenceFrame,
                          final AccelerationTriad ba,
                          final Matrix ma,
                          final AngularSpeedTriad bg,
                          final Matrix mg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg);
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg, listener);
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg, gg);
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final AccelerationTriad ba,
            final Matrix ma,
            final AngularSpeedTriad bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg, gg, listener);
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg);
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg, listener);
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
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg, gg);
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
     * @param listener       listener to handle events.
     * @throws AlgebraException         if provided cross-coupling matrices cannot
     *                                  be inverted.
     * @throws IllegalArgumentException if any provided matrices are not 3x3.
     */
    public DriftEstimator(
            final NEDFrame referenceFrame,
            final Matrix ba,
            final Matrix ma,
            final Matrix bg,
            final Matrix mg,
            final Matrix gg,
            final DriftEstimatorListener listener) throws AlgebraException {
        this(NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceFrame), ba, ma, bg, mg, gg, listener);
    }

    /**
     * Gets listener to handle events raised by this estimator.
     *
     * @return listener to handle events raised by this estimator.
     */
    public DriftEstimatorListener getListener() {
        return listener;
    }

    /**
     * Sets listener to handle events raised by this estimator.
     *
     * @param listener listener to handle events raised by this estimator.
     * @throws LockedException if estimator is running.
     */
    public void setListener(final DriftEstimatorListener listener) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        this.listener = listener;
    }

    /**
     * Gets initial frame containing body position, velocity and orientation
     * expressed in ECEF coordinates before starting drift estimation.
     *
     * @return initial body frame or null.
     */
    public ECEFFrame getReferenceFrame() {
        return referenceFrame;
    }

    /**
     * Sets initial frame containing body position, velocity and orientation
     * expressed in ECEF coordinates before starting drift estimation.
     *
     * @param referenceFrame initial frame or null.
     * @throws LockedException if estimator is already running.
     */
    public void setReferenceFrame(final ECEFFrame referenceFrame) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.referenceFrame = referenceFrame;
    }

    /**
     * Gets initial frame containing body position, velocity and orientation
     * expressed in NED coordinates before starting drift estimation.
     *
     * @return initial body frame or null.
     */
    public NEDFrame getReferenceNedFrame() {
        return referenceFrame != null
                ? ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(referenceFrame)
                : null;
    }

    /**
     * Gets initial frame containing body position, velocity and orientation
     * expressed in NED coordinates before starting drift estimation.
     *
     * @param result instance where the result will be stored.
     * @return true if the initial frame was available and the result was updated, false
     * otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedFrame(final NEDFrame result) {
        if (referenceFrame != null) {
            ECEFtoNEDFrameConverter.convertECEFtoNED(referenceFrame, result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets an initial frame containing body position, velocity and orientation
     * expressed in NED coordinates before starting drift estimation.
     *
     * @param referenceNedFrame initial body frame or null.
     * @throws LockedException if estimator is already running.
     */
    public void setReferenceNedFrame(final NEDFrame referenceNedFrame) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (referenceNedFrame != null) {
            if (referenceFrame != null) {
                NEDtoECEFFrameConverter.convertNEDtoECEF(referenceNedFrame, referenceFrame);
            } else {
                referenceFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(referenceNedFrame);
            }
        } else {
            referenceFrame = null;
        }
    }

    /**
     * Gets initial body position, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @return initial body position or null.
     */
    public ECEFPosition getReferenceEcefPosition() {
        return referenceFrame != null ? referenceFrame.getECEFPosition() : null;
    }

    /**
     * Gets initial body position, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param result instance where the result will be stored.
     * @return true if the initial body position was available and the result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceEcefPosition(final ECEFPosition result) {
        if (referenceFrame != null) {
            referenceFrame.getECEFPosition(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body position, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param referenceEcefPosition initial body position.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if provided position is null.
     */
    public void setReferenceEcefPosition(final ECEFPosition referenceEcefPosition) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (referenceFrame != null) {
            referenceFrame.setPosition(referenceEcefPosition);
        } else {
            referenceFrame = new ECEFFrame(referenceEcefPosition);
        }
    }

    /**
     * Gets initial body velocity, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @return initial body velocity or null.
     */
    public ECEFVelocity getReferenceEcefVelocity() {
        return referenceFrame != null ? referenceFrame.getECEFVelocity() : null;
    }

    /**
     * Gets initial body velocity, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param result instance where the result will be stored.
     * @return true if initial body velocity was available and the result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceEcefVelocity(final ECEFVelocity result) {
        if (referenceFrame != null) {
            referenceFrame.getECEFVelocity(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body velocity, expressed in ECEF coordinates, before starting
     * drift estimation.
     *
     * @param referenceEcefVelocity initial body velocity.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if velocity is null.
     */
    public void setReferenceEcefVelocity(final ECEFVelocity referenceEcefVelocity) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (referenceFrame == null) {
            referenceFrame = new ECEFFrame();
        }

        referenceFrame.setVelocity(referenceEcefVelocity);
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in ECEF coordinates, before starting estimation.
     *
     * @return initial body orientation or null.
     */
    public CoordinateTransformation getReferenceEcefCoordinateTransformation() {
        return referenceFrame != null ?
                referenceFrame.getCoordinateTransformation() : null;
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in ECEF coordinates, before starting estimation.
     *
     * @param result instance where the result will be stored.
     * @return true if initial body orientation was available and a result was
     * updated, false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceEcefCoordinateTransformation(final CoordinateTransformation result) {
        if (referenceFrame != null) {
            referenceFrame.getCoordinateTransformation(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body coordinate transformation, containing body orientation
     * expressed in ECEF coordinates, before starting estimation.
     *
     * @param referenceEcefCoordinateTransformation initial body orientation.
     * @throws LockedException                               if estimator is already running.
     * @throws InvalidSourceAndDestinationFrameTypeException if source and
     *                                                       destination types are invalid. Source type must be
     *                                                       {@link com.irurueta.navigation.frames.FrameType#BODY_FRAME} and the destination
     *                                                       type must be {@link com.irurueta.navigation.frames.FrameType#EARTH_CENTERED_EARTH_FIXED_FRAME}
     *                                                       indicating that body orientation is expressed respect ECEF coordinates.
     * @throws NullPointerException                          if orientation is null.
     */
    public void setReferenceEcefCoordinateTransformation(
            final CoordinateTransformation referenceEcefCoordinateTransformation) throws LockedException,
            InvalidSourceAndDestinationFrameTypeException {
        if (running) {
            throw new LockedException();
        }

        if (referenceFrame == null) {
            referenceFrame = new ECEFFrame(referenceEcefCoordinateTransformation);
        } else {
            referenceFrame.setCoordinateTransformation(referenceEcefCoordinateTransformation);
        }
    }

    /**
     * Gets initial body position, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @return initial body position or null.
     */
    public NEDPosition getReferenceNedPosition() {
        final var nedFrame = getReferenceNedFrame();
        return nedFrame != null ? nedFrame.getPosition() : null;
    }

    /**
     * Gets initial body position, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param result instance where the result will be stored.
     * @return true if the initial body position was available and the result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedPosition(final NEDPosition result) {
        if (referenceFrame != null) {
            final var nedFrame = getReferenceNedFrame();
            nedFrame.getPosition(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body position, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param referenceNedPosition initial body position.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if provided position is null.
     */
    public void setReferenceNedPosition(final NEDPosition referenceNedPosition) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (referenceFrame != null) {
            final var nedFrame = getReferenceNedFrame();
            nedFrame.setPosition(referenceNedPosition);
            setReferenceNedFrame(nedFrame);
        } else {
            setReferenceNedFrame(new NEDFrame(referenceNedPosition));
        }
    }

    /**
     * Gets initial body velocity, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @return initial body velocity or null.
     */
    public NEDVelocity getReferenceNedVelocity() {
        final var nedFrame = getReferenceNedFrame();
        return nedFrame != null ? nedFrame.getVelocity() : null;
    }

    /**
     * Gets initial body velocity, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param result instance where the result will be stored.
     * @return true if initial body velocity was available and the result was updated,
     * false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedVelocity(final NEDVelocity result) {
        if (referenceFrame != null) {
            final var nedFrame = getReferenceNedFrame();
            nedFrame.getVelocity(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body velocity, expressed in NED coordinates, before starting
     * drift estimation.
     *
     * @param referenceNedVelocity initial body velocity.
     * @throws LockedException      if estimator is already running.
     * @throws NullPointerException if velocity is null.
     */
    public void setReferenceNedVelocity(final NEDVelocity referenceNedVelocity) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        final NEDFrame nedFrame;
        if (referenceFrame != null) {
            nedFrame = getReferenceNedFrame();
        } else {
            nedFrame = new NEDFrame();
        }

        nedFrame.setVelocity(referenceNedVelocity);
        setReferenceNedFrame(nedFrame);
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in NED coordinates, before starting estimation.
     *
     * @return initial body orientation or null.
     */
    public CoordinateTransformation getReferenceNedCoordinateTransformation() {
        final var nedFrame = getReferenceNedFrame();
        return nedFrame != null ? nedFrame.getCoordinateTransformation() : null;
    }

    /**
     * Gets initial body coordinate transformation, containing body orientation
     * expressed in NED coordinates, before starting estimation.
     *
     * @param result instance where the result will be stored.
     * @return true if initial body orientation was available and the result was
     * updated, false otherwise.
     * @throws NullPointerException if provided result instance is null.
     */
    public boolean getReferenceNedCoordinateTransformation(final CoordinateTransformation result) {
        if (referenceFrame != null) {
            final var nedFrame = getReferenceNedFrame();
            nedFrame.getCoordinateTransformation(result);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Sets initial body coordinate transformation, containing body orientation
     * expressed in NED coordinates, before starting estimation.
     *
     * @param referenceNedCoordinateTransformation initial body orientation.
     * @throws LockedException                               if estimator is already running.
     * @throws InvalidSourceAndDestinationFrameTypeException if source and
     *                                                       destination types are invalid. Source type must be
     *                                                       {@link com.irurueta.navigation.frames.FrameType#BODY_FRAME} and the destination
     *                                                       type must be {@link com.irurueta.navigation.frames.FrameType#LOCAL_NAVIGATION_FRAME}
     *                                                       indicating that body orientation is expressed respect NED coordinates.
     * @throws NullPointerException                          if orientation is null.
     */
    public void setReferenceNedCoordinateTransformation(
            final CoordinateTransformation referenceNedCoordinateTransformation) throws LockedException,
            InvalidSourceAndDestinationFrameTypeException {
        if (running) {
            throw new LockedException();
        }

        final NEDFrame nedFrame;
        if (referenceFrame == null) {
            nedFrame = new NEDFrame(referenceNedCoordinateTransformation);
            setReferenceNedFrame(nedFrame);
        } else {
            nedFrame = getReferenceNedFrame();
            nedFrame.setCoordinateTransformation(referenceNedCoordinateTransformation);
        }
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public Matrix getAccelerationBias() {
        return fixer.getAccelerationBias();
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where the result will be stored.
     */
    public void getAccelerationBias(final Matrix result) {
        fixer.getAccelerationBias(result);
    }

    /**
     * Sets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second.
     *             Must be 3x1.
     * @throws LockedException          if estimator is running.
     * @throws IllegalArgumentException if any provided matrix is not 3x1.
     */
    public void setAccelerationBias(final Matrix bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBias(bias);
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @return bias values expressed in meters per squared second.
     */
    public double[] getAccelerationBiasArray() {
        return fixer.getAccelerationBiasArray();
    }

    /**
     * Gets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAccelerationBiasArray(final double[] result) {
        fixer.getAccelerationBiasArray(result);
    }

    /**
     * Sets acceleration bias values expressed in meters per squared second (m/s^2).
     *
     * @param bias bias values expressed in meters per squared second (m/s^2).
     *             Must have length 3.
     * @throws IllegalArgumentException if provided array does not have length 3.
     * @throws LockedException          if estimator is running.
     */
    public void setAccelerationBias(final double[] bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBias(bias);
    }

    /**
     * Gets acceleration bias.
     *
     * @return acceleration bias.
     */
    public AccelerationTriad getAccelerationBiasAsTriad() {
        return fixer.getAccelerationBiasAsTriad();
    }

    /**
     * Gets acceleration bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAccelerationBiasAsTriad(final AccelerationTriad result) {
        fixer.getAccelerationBiasAsTriad(result);
    }

    /**
     * Sets acceleration bias.
     *
     * @param bias acceleration bias to be set.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBias(final AccelerationTriad bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBias(bias);
    }

    /**
     * Gets acceleration x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return x-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasX() {
        return fixer.getAccelerationBiasX();
    }

    /**
     * Sets acceleration x-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x-coordinate of bias expressed in meters per squared second
     *              (m/s^2).
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasX(final double biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBiasX(biasX);
    }

    /**
     * Gets acceleration y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return y-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasY() {
        return fixer.getAccelerationBiasY();
    }

    /**
     * Sets acceleration y-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasY y-coordinate of bias expressed in meters per squared second
     *              (m/s^2).
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasY(final double biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBiasY(biasY);
    }

    /**
     * Gets acceleration z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @return z-coordinate of bias expressed in meters per squared second (m/s^2).
     */
    public double getAccelerationBiasZ() {
        return fixer.getAccelerationBiasZ();
    }

    /**
     * Sets acceleration z-coordinate of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasZ z-coordinate of bias expressed in meters per squared second (m/s^2).
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasZ(final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBiasZ(biasZ);
    }

    /**
     * Sets acceleration coordinates of bias expressed in meters per squared
     * second (m/s^2).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBias(final double biasX, final double biasY, final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBias(biasX, biasY, biasZ);
    }

    /**
     * Gets acceleration x-coordinate of bias.
     *
     * @return acceleration x-coordinate of bias.
     */
    public Acceleration getAccelerationBiasXAsAcceleration() {
        return fixer.getAccelerationBiasXAsAcceleration();
    }

    /**
     * Gets acceleration x-coordinate of bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAccelerationBiasXAsAcceleration(final Acceleration result) {
        fixer.getAccelerationBiasXAsAcceleration(result);
    }

    /**
     * Sets acceleration x-coordinate of bias.
     *
     * @param biasX acceleration x-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasX(final Acceleration biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBiasX(biasX);
    }

    /**
     * Gets acceleration y-coordinate of bias.
     *
     * @return acceleration y-coordinate of bias.
     */
    public Acceleration getAccelerationBiasYAsAcceleration() {
        return fixer.getAccelerationBiasYAsAcceleration();
    }

    /**
     * Gets acceleration y-coordinate of bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAccelerationBiasYAsAcceleration(final Acceleration result) {
        fixer.getAccelerationBiasYAsAcceleration(result);
    }

    /**
     * Sets acceleration y-coordinate of bias.
     *
     * @param biasY acceleration y-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasY(final Acceleration biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBiasY(biasY);
    }

    /**
     * Gets acceleration z-coordinate of bias.
     *
     * @return acceleration z-coordinate of bias.
     */
    public Acceleration getAccelerationBiasZAsAcceleration() {
        return fixer.getAccelerationBiasZAsAcceleration();
    }

    /**
     * Gets acceleration z-coordinate of bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAccelerationBiasZAsAcceleration(final Acceleration result) {
        fixer.getAccelerationBiasZAsAcceleration(result);
    }

    /**
     * Sets acceleration z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBiasZ(final Acceleration biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBiasZ(biasZ);
    }

    /**
     * Sets acceleration coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAccelerationBias(
            final Acceleration biasX,
            final Acceleration biasY,
            final Acceleration biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationBias(biasX, biasY, biasZ);
    }

    /**
     * Gets acceleration cross-coupling errors matrix.
     *
     * @return acceleration cross-coupling errors matrix.
     */
    public Matrix getAccelerationCrossCouplingErrors() {
        return fixer.getAccelerationCrossCouplingErrors();
    }

    /**
     * Gets acceleration cross-coupling errors matrix.
     *
     * @param result instance where the result will be stored.
     */
    public void getAccelerationCrossCouplingErrors(final Matrix result) {
        fixer.getAccelerationCrossCouplingErrors(result);
    }

    /**
     * Sets acceleration cross-coupling errors matrix.
     *
     * @param crossCouplingErrors acceleration cross-coupling errors matrix.
     *                            Must be 3x3.
     * @throws LockedException          if estimator is running.
     * @throws AlgebraException         if matrix cannot be inverted.
     * @throws IllegalArgumentException if matrix is not 3x3.
     */
    public void setAccelerationCrossCouplingErrors(final Matrix crossCouplingErrors) throws AlgebraException,
            LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationCrossCouplingErrors(crossCouplingErrors);
    }

    /**
     * Gets acceleration x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getAccelerationSx() {
        return fixer.getAccelerationSx();
    }

    /**
     * Sets acceleration x scaling factor.
     *
     * @param sx x scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationSx(final double sx) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationSx(sx);
    }

    /**
     * Gets acceleration y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getAccelerationSy() {
        return fixer.getAccelerationSy();
    }

    /**
     * Sets acceleration y scaling factor.
     *
     * @param sy y scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationSy(final double sy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationSy(sy);
    }

    /**
     * Gets acceleration z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getAccelerationSz() {
        return fixer.getAccelerationSz();
    }

    /**
     * Sets acceleration z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationSz(final double sz) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationSz(sz);
    }

    /**
     * Gets acceleration x-y cross-coupling error.
     *
     * @return acceleration x-y cross-coupling error.
     */
    public double getAccelerationMxy() {
        return fixer.getAccelerationMxy();
    }

    /**
     * Sets acceleration x-y cross-coupling error.
     *
     * @param mxy acceleration x-y cross-coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationMxy(final double mxy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationMxy(mxy);
    }

    /**
     * Gets acceleration x-z cross-coupling error.
     *
     * @return acceleration x-z cross-coupling error.
     */
    public double getAccelerationMxz() {
        return fixer.getAccelerationMxz();
    }

    /**
     * Sets acceleration x-z cross-coupling error.
     *
     * @param mxz acceleration x-z cross-coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationMxz(final double mxz) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationMxz(mxz);
    }

    /**
     * Gets acceleration y-x cross-coupling error.
     *
     * @return acceleration y-x cross-coupling error.
     */
    public double getAccelerationMyx() {
        return fixer.getAccelerationMyx();
    }

    /**
     * Sets acceleration y-x cross-coupling error.
     *
     * @param myx acceleration y-x cross-coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationMyx(final double myx) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationMyx(myx);
    }

    /**
     * Gets acceleration y-z cross-coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getAccelerationMyz() {
        return fixer.getAccelerationMyz();
    }

    /**
     * Sets acceleration y-z cross-coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationMyz(final double myz) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationMyz(myz);
    }

    /**
     * Gets acceleration z-x cross-coupling error.
     *
     * @return acceleration z-x cross-coupling error.
     */
    public double getAccelerationMzx() {
        return fixer.getAccelerationMzx();
    }

    /**
     * Sets acceleration z-x cross coupling error.
     *
     * @param mzx acceleration z-x cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationMzx(final double mzx) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationMzx(mzx);
    }

    /**
     * Gets acceleration z-y cross-coupling error.
     *
     * @return acceleration z-y cross-coupling error.
     */
    public double getAccelerationMzy() {
        return fixer.getAccelerationMzy();
    }

    /**
     * Sets acceleration z-y cross-coupling error.
     *
     * @param mzy acceleration z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationMzy(final double mzy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationMzy(mzy);
    }

    /**
     * Sets acceleration scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationScalingFactors(final double sx, final double sy, final double sz)
            throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationScalingFactors(sx, sy, sz);
    }

    /**
     * Sets acceleration cross-coupling errors.
     *
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Sets acceleration scaling factors and cross-coupling errors.
     *
     * @param sx  x scaling factor.
     * @param sy  y scaling factor.
     * @param sz  z scaling factor.
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make acceleration cross-coupling matrix non-invertible.
     */
    public void setAccelerationScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAccelerationScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @return angular speed bias values expressed in radians per second.
     */
    public Matrix getAngularSpeedBias() {
        return fixer.getAngularSpeedBias();
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param result instance where the result will be stored.
     */
    public void getAngularSpeedBias(final Matrix result) {
        fixer.getAngularSpeedBias(result);
    }

    /**
     * Sets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second. Must be 3x1.
     * @throws IllegalArgumentException if matrix is not 3x1.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedBias(final Matrix bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBias(bias);
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @return bias values expressed in radians per second.
     */
    public double[] getAngularSpeedBiasArray() {
        return fixer.getAngularSpeedBiasArray();
    }

    /**
     * Gets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param result instance where result data will be stored.
     * @throws IllegalArgumentException if provided array does not have length 3.
     */
    public void getAngularSpeedBiasArray(final double[] result) {
        fixer.getAngularSpeedBiasArray(result);
    }

    /**
     * Sets angular speed bias values expressed in radians per second (rad/s).
     *
     * @param bias bias values expressed in radians per second (rad/s). Must
     *             have length 3.
     * @throws IllegalArgumentException if provided array does not have length 3.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedBias(final double[] bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBias(bias);
    }

    /**
     * Gets angular speed bias.
     *
     * @return angular speed bias.
     */
    public AngularSpeedTriad getAngularSpeedBiasAsTriad() {
        return fixer.getAngularSpeedBiasAsTriad();
    }

    /**
     * Gets angular speed bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAngularSpeedBiasAsTriad(final AngularSpeedTriad result) {
        fixer.getAngularSpeedBiasAsTriad(result);
    }

    /**
     * Sets angular speed bias.
     *
     * @param bias angular speed bias to be set.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBias(final AngularSpeedTriad bias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBias(bias);
    }

    /**
     * Gets angular speed x-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return x-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasX() {
        return fixer.getAngularSpeedBiasX();
    }

    /**
     * Sets angular speed x-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x-coordinate of bias expressed in radians per second (rad/s).
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasX(final double biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBiasX(biasX);
    }

    /**
     * Gets angular speed y-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return y-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasY() {
        return fixer.getAngularSpeedBiasY();
    }

    /**
     * Sets angular speed y-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasY y-coordinate of bias expressed in radians per second (rad/s).
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasY(final double biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBiasY(biasY);
    }

    /**
     * Gets angular speed z-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @return z-coordinate of bias expressed in radians per second (rad/s).
     */
    public double getAngularSpeedBiasZ() {
        return fixer.getAngularSpeedBiasZ();
    }

    /**
     * Sets angular speed z-coordinate of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasZ z-coordinate of bias expressed in radians per second (rad/s).
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasZ(final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBiasZ(biasZ);
    }

    /**
     * Sets angular speed coordinates of bias expressed in radians per second
     * (rad/s).
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBias(final double biasX, final double biasY, final double biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBias(biasX, biasY, biasZ);
    }

    /**
     * Gets angular speed x-coordinate of bias.
     *
     * @return x-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasXAsAngularSpeed() {
        return fixer.getAngularSpeedBiasXAsAngularSpeed();
    }

    /**
     * Gets angular speed x-coordinate of bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAngularSpeedBiasXAsAngularSpeed(final AngularSpeed result) {
        fixer.getAngularSpeedBiasXAsAngularSpeed(result);
    }

    /**
     * Sets angular speed x-coordinate of bias.
     *
     * @param biasX x-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasX(final AngularSpeed biasX) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBiasX(biasX);
    }

    /**
     * Gets angular speed y-coordinate of bias.
     *
     * @return y-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasYAsAngularSpeed() {
        return fixer.getAngularSpeedBiasYAsAngularSpeed();
    }

    /**
     * Gets angular speed y-coordinate of bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAngularSpeedBiasYAsAngularSpeed(final AngularSpeed result) {
        fixer.getAngularSpeedBiasYAsAngularSpeed(result);
    }

    /**
     * Sets angular speed y-coordinate of bias.
     *
     * @param biasY y-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasY(final AngularSpeed biasY) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBiasY(biasY);
    }

    /**
     * Gets angular speed z-coordinate of bias.
     *
     * @return z-coordinate of bias.
     */
    public AngularSpeed getAngularSpeedBiasZAsAngularSpeed() {
        return fixer.getAngularSpeedBiasZAsAngularSpeed();
    }

    /**
     * Gets angular speed z-coordinate of bias.
     *
     * @param result instance where the result will be stored.
     */
    public void getAngularSpeedBiasZAsAngularSpeed(final AngularSpeed result) {
        fixer.getAngularSpeedBiasZAsAngularSpeed(result);
    }

    /**
     * Sets angular speed z-coordinate of bias.
     *
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBiasZ(final AngularSpeed biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBiasZ(biasZ);
    }

    /**
     * Sets angular speed coordinates of bias.
     *
     * @param biasX x-coordinate of bias.
     * @param biasY y-coordinate of bias.
     * @param biasZ z-coordinate of bias.
     * @throws LockedException if estimator is running.
     */
    public void setAngularSpeedBias(
            final AngularSpeed biasX,
            final AngularSpeed biasY,
            final AngularSpeed biasZ) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedBias(biasX, biasY, biasZ);
    }

    /**
     * Gets angular speed cross-coupling errors matrix.
     *
     * @return cross coupling errors matrix.
     */
    public Matrix getAngularSpeedCrossCouplingErrors() {
        return fixer.getAngularSpeedCrossCouplingErrors();
    }

    /**
     * Gets angular speed cross-coupling errors matrix.
     *
     * @param result instance where the result will be stored.
     */
    public void getAngularSpeedCrossCouplingErrors(final Matrix result) {
        fixer.getAngularSpeedCrossCouplingErrors(result);
    }

    /**
     * Sets angular speed cross-coupling errors matrix.
     *
     * @param crossCouplingErrors cross-coupling errors matrix. Must be 3x3.
     * @throws AlgebraException         if matrix cannot be inverted.
     * @throws IllegalArgumentException if matrix is not 3x3.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedCrossCouplingErrors(final Matrix crossCouplingErrors) throws AlgebraException,
            LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedCrossCouplingErrors(crossCouplingErrors);
    }

    /**
     * Gets angular speed x scaling factor.
     *
     * @return x scaling factor.
     */
    public double getAngularSpeedSx() {
        return fixer.getAngularSpeedSx();
    }

    /**
     * Sets angular speed x scaling factor.
     *
     * @param sx x scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedSx(final double sx) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedSx(sx);
    }

    /**
     * Gets angular speed y scaling factor.
     *
     * @return y scaling factor.
     */
    public double getAngularSpeedSy() {
        return fixer.getAngularSpeedSy();
    }

    /**
     * Sets angular speed y-scaling factor.
     *
     * @param sy y scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedSy(final double sy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedSy(sy);
    }

    /**
     * Gets angular speed z scaling factor.
     *
     * @return z scaling factor.
     */
    public double getAngularSpeedSz() {
        return fixer.getAngularSpeedSz();
    }

    /**
     * Sets angular speed z scaling factor.
     *
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedSz(final double sz) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedSz(sz);
    }

    /**
     * Gets angular speed x-y cross-coupling error.
     *
     * @return x-y cross coupling error.
     */
    public double getAngularSpeedMxy() {
        return fixer.getAngularSpeedMxy();
    }

    /**
     * Sets angular speed x-y cross-coupling error.
     *
     * @param mxy x-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedMxy(final double mxy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedMxy(mxy);
    }

    /**
     * Gets angular speed x-z cross-coupling error.
     *
     * @return x-z cross coupling error.
     */
    public double getAngularSpeedMxz() {
        return fixer.getAngularSpeedMxz();
    }

    /**
     * Sets angular speed x-z cross-coupling error.
     *
     * @param mxz x-z cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedMxz(final double mxz) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedMxz(mxz);
    }

    /**
     * Gets angular speed y-x cross-coupling error.
     *
     * @return y-x cross coupling error.
     */
    public double getAngularSpeedMyx() {
        return fixer.getAngularSpeedMyx();
    }

    /**
     * Sets angular speed y-x cross-coupling error.
     *
     * @param myx y-x cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedMyx(final double myx) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedMyx(myx);
    }

    /**
     * Gets angular speed y-z cross-coupling error.
     *
     * @return y-z cross coupling error.
     */
    public double getAngularSpeedMyz() {
        return fixer.getAngularSpeedMyz();
    }

    /**
     * Sets angular speed y-z cross-coupling error.
     *
     * @param myz y-z cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedMyz(final double myz) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedMyz(myz);
    }

    /**
     * Gets angular speed z-x cross-coupling error.
     *
     * @return z-x cross coupling error.
     */
    public double getAngularSpeedMzx() {
        return fixer.getAngularSpeedMzx();
    }

    /**
     * Sets angular speed z-x cross-coupling error.
     *
     * @param mzx z-x cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedMzx(final double mzx) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedMzx(mzx);
    }

    /**
     * Gets angular speed z-y cross-coupling error.
     *
     * @return z-y cross coupling error.
     */
    public double getAngularSpeedMzy() {
        return fixer.getAngularSpeedMzy();
    }

    /**
     * Sets angular speed z-y cross-coupling error.
     *
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided value makes angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedMzy(final double mzy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedMzy(mzy);
    }

    /**
     * Sets angular speed scaling factors.
     *
     * @param sx x scaling factor.
     * @param sy y scaling factor.
     * @param sz z scaling factor.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedScalingFactors(final double sx, final double sy, final double sz) throws LockedException,
            AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedScalingFactors(sx, sy, sz);
    }

    /**
     * Sets angular speed cross-coupling errors.
     *
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedCrossCouplingErrors(
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Sets angular speed scaling factors and cross-coupling errors.
     *
     * @param sx  x scaling factor.
     * @param sy  y scaling factor.
     * @param sz  z scaling factor.
     * @param mxy x-y cross coupling error.
     * @param mxz x-z cross coupling error.
     * @param myx y-x cross coupling error.
     * @param myz y-z cross coupling error.
     * @param mzx z-x cross coupling error.
     * @param mzy z-y cross coupling error.
     * @throws LockedException  if estimator is running.
     * @throws AlgebraException if provided values make angular speed
     *                          cross-coupling matrix non-invertible.
     */
    public void setAngularSpeedScalingFactorsAndCrossCouplingErrors(
            final double sx, final double sy, final double sz,
            final double mxy, final double mxz,
            final double myx, final double myz,
            final double mzx, final double mzy) throws LockedException, AlgebraException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);
    }

    /**
     * Gets angular speed g-dependant cross biases matrix.
     *
     * @return g-dependant cross biases matrix.
     */
    public Matrix getAngularSpeedGDependantCrossBias() {
        return fixer.getAngularSpeedGDependantCrossBias();
    }

    /**
     * Gets angular speed g-dependant cross biases matrix.
     *
     * @param result instance where the result will be stored.
     */
    public void getAngularSpeedGDependantCrossBias(final Matrix result) {
        fixer.getAngularSpeedGDependantCrossBias(result);
    }

    /**
     * Sets angular speed g-dependant cross biases matrix.
     *
     * @param gDependantCrossBias g-dependant cross biases matrix.
     * @throws IllegalArgumentException if matrix is not 3x3.
     * @throws LockedException          if estimator is running.
     */
    public void setAngularSpeedGDependantCrossBias(final Matrix gDependantCrossBias) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        fixer.setAngularSpeedGDependantCrossBias(gDependantCrossBias);
    }

    /**
     * Indicates whether measured kinematics must be fixed or not.
     * When enabled, provided calibration data is used; otherwise it is
     * ignored.
     * By default, this is enabled.
     *
     * @return indicates whether measured kinematics must be fixed or not.
     */
    public boolean isFixKinematicsEnabled() {
        return fixKinematics;
    }

    /**
     * Specifies whether measured kinematics must be fixed or not.
     * When enabled, provided calibration data is used; otherwise it is
     * ignored.
     *
     * @param fixKinematics true if measured kinematics must be fixed or not.
     * @throws LockedException if estimator is currently running.
     */
    public void setFixKinematicsEnabled(final boolean fixKinematics) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        this.fixKinematics = fixKinematics;
    }

    /**
     * Gets the time interval between body kinematics (IMU acceleration and gyroscope)
     * samples expressed in seconds (s).
     *
     * @return time interval between body kinematics samples.
     */
    public double getTimeInterval() {
        return timeInterval;
    }

    /**
     * Sets a time interval between body kinematics (IMU acceleration and gyroscope)
     * samples expressed in seconds (s).
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final double timeInterval) throws LockedException {
        if (running) {
            throw new LockedException();
        }

        if (timeInterval < 0.0) {
            throw new IllegalArgumentException();
        }

        this.timeInterval = timeInterval;
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration and gyroscope)
     * samples.
     *
     * @return time interval between body kinematics samples.
     */
    public Time getTimeIntervalAsTime() {
        return new Time(timeInterval, TimeUnit.SECOND);
    }

    /**
     * Gets time interval between body kinematics (IMU acceleration and gyroscope)
     * samples.
     *
     * @param result instance where the time interval will be stored.
     */
    public void getTimeIntervalAsTime(final Time result) {
        result.setValue(timeInterval);
        result.setUnit(TimeUnit.SECOND);
    }

    /**
     * Sets time interval between body kinematics (IMU acceleration and gyroscope)
     * samples.
     *
     * @param timeInterval time interval between body kinematics samples.
     * @throws LockedException if estimator is currently running.
     */
    public void setTimeInterval(final Time timeInterval) throws LockedException {
        setTimeInterval(convertTime(timeInterval));
    }

    /**
     * Indicates if estimator is ready to start processing additional kinematics
     * measurements.
     *
     * @return true if ready, false otherwise.
     */
    public boolean isReady() {
        return referenceFrame != null;
    }

    /**
     * Indicates whether this estimator is running.
     *
     * @return true if this estimator is running, false otherwise.
     */
    public boolean isRunning() {
        return running;
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
    public void addBodyKinematics(final BodyKinematics kinematics) throws LockedException, NotReadyException,
            DriftEstimationException {
        if (running) {
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

                final var c = referenceFrame.getCoordinateTransformation();
                c.asRotation(refQ);
                refQ.inverse(invRefQ);

                frame.copyFrom(referenceFrame);
            }

            if (fixKinematics) {
                fixer.fix(kinematics, fixedKinematics);
            } else {
                fixedKinematics.copyFrom(kinematics);
            }

            // estimate navigation variation respect to previous frame
            ECEFInertialNavigator.navigateECEF(timeInterval, frame, fixedKinematics, frame);

            // estimate drift values
            computeCurrentPositionDrift();
            computeCurrentVelocityDrift();
            computeCurrentOrientationDrift();

            numberOfProcessedSamples++;

            if (listener != null) {
                listener.onBodyKinematicsAdded(this, kinematics, fixedKinematics);
            }

        } catch (final AlgebraException | InertialNavigatorException | InvalidRotationMatrixException e) {
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
    public void reset() throws LockedException {
        if (running) {
            throw new LockedException();
        }

        running = true;
        numberOfProcessedSamples = 0;

        currentPositionDriftMeters = 0.0;
        currentVelocityDriftMetersPerSecond = 0.0;
        currentOrientationDriftRadians = 0.0;

        if (listener != null) {
            listener.onReset(this);
        }

        running = false;
    }

    /**
     * Gets the number of samples that have been processed so far.
     *
     * @return number of samples that have been processed so far.
     */
    public int getNumberOfProcessedSamples() {
        return numberOfProcessedSamples;
    }

    /**
     * Gets elapsed time since the first processed measurement expressed in seconds.
     *
     * @return elapsed time.
     */
    public double getElapsedTimeSeconds() {
        return numberOfProcessedSamples * timeInterval;
    }

    /**
     * Gets elapsed time since the first processed measurement.
     *
     * @return elapsed time.
     */
    public Time getElapsedTime() {
        return numberOfProcessedSamples > 0 ? new Time(getElapsedTimeSeconds(), TimeUnit.SECOND) : null;
    }

    /**
     * Gets elapsed time since the first processed measurement.
     *
     * @param result instance where the result will be stored.
     * @return true if elapsed time is available and the result is updated,
     * false otherwise.
     */
    public boolean getElapsedTime(final Time result) {
        if (numberOfProcessedSamples > 0) {
            result.setValue(getElapsedTimeSeconds());
            result.setUnit(TimeUnit.SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current position drift for last processed body kinematics
     * measurement expressed in meters (m) respect ECEF coordinates.
     *
     * @return current position drift or null.
     */
    public ECEFPosition getCurrentPositionDrift() {
        return numberOfProcessedSamples > 0 ? new ECEFPosition(currentPositionDrift) : null;
    }

    /**
     * Gets current position drift for last processed body kinematics
     * measurement expressed in meters (m) respect ECEF coordinates.
     *
     * @param result instance where the result will be stored.
     * @return true if the current position drift is available and the result is updated,
     * false otherwise.
     */
    public boolean getCurrentPositionDrift(final ECEFPosition result) {
        if (numberOfProcessedSamples > 0) {
            result.copyFrom(currentPositionDrift);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current velocity drift for the last processed body kinematics
     * measurement expressed in meters per second (m/s) respect ECEF coordinates.
     *
     * @return current velocity drift or null.
     */
    public ECEFVelocity getCurrentVelocityDrift() {
        return numberOfProcessedSamples > 0 ? new ECEFVelocity(currentVelocityDrift) : null;
    }

    /**
     * Gets current velocity drift for the last processed body kinematics
     * measurement expressed in meters per second (m/s) respect ECEF coordinates.
     *
     * @param result instance where the result will be stored.
     * @return true if the current velocity drift is available and the result is updated,
     * false otherwise.
     */
    public boolean getCurrentVelocityDrift(final ECEFVelocity result) {
        if (numberOfProcessedSamples > 0) {
            result.copyFrom(currentVelocityDrift);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current orientation drift as a 3D rotation for the last processed body
     * kinematics measurement respect ECEF coordinates.
     *
     * @return current orientation drift or null.
     */
    public Rotation3D getCurrentOrientationDrift() {
        return numberOfProcessedSamples > 0 ? new Quaternion(q) : null;
    }

    /**
     * Gets current orientation drift as a 3D rotation for the last processed body
     * kinematics measurement respect ECEF coordinates.
     *
     * @param result instance where the result will be stored.
     * @return true if the current orientation drift is available and the result is
     * updated, false otherwise.
     */
    public boolean getCurrentOrientationDrift(final Rotation3D result) {
        if (numberOfProcessedSamples > 0) {
            result.fromRotation(q);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of position drift for last processed body kinematics
     * measurement expressed in meters (m).
     *
     * @return the current amount of position drift or null.
     */
    public Double getCurrentPositionDriftNormMeters() {
        return numberOfProcessedSamples > 0 ? currentPositionDriftMeters : null;
    }

    /**
     * Gets current amount of position drift for last processed body kinematics
     * measurement.
     *
     * @return the current amount of position drift or null.
     */
    public Distance getCurrentPositionDriftNorm() {
        final var positionDrift = getCurrentPositionDriftNormMeters();
        return positionDrift != null ? new Distance(positionDrift, DistanceUnit.METER) : null;
    }

    /**
     * Gets current amount of position drift for last processed body kinematics
     * measurement.
     *
     * @param result instance where the result will be stored.
     * @return true if the current position drift is available and the result is updated,
     * false otherwise.
     */
    public boolean getCurrentPositionDriftNorm(final Distance result) {
        final var positionDrift = getCurrentPositionDriftNormMeters();
        if (positionDrift != null) {
            result.setValue(positionDrift);
            result.setUnit(DistanceUnit.METER);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of velocity drift for the last processed body kinematics
     * measurement expressed in meters per second (m/s).
     *
     * @return current amount of velocity drift or null.
     */
    public Double getCurrentVelocityDriftNormMetersPerSecond() {
        return numberOfProcessedSamples > 0 ? currentVelocityDriftMetersPerSecond : null;
    }

    /**
     * Gets current amount of velocity drift for last processed body kinematics
     * measurement.
     *
     * @return current amount of velocity drift or null.
     */
    public Speed getCurrentVelocityDriftNorm() {
        final var velocityDrift = getCurrentVelocityDriftNormMetersPerSecond();
        return velocityDrift != null ? new Speed(velocityDrift, SpeedUnit.METERS_PER_SECOND) : null;
    }

    /**
     * Gets current amount of velocity drift for last processed body kinematics
     * measurement.
     *
     * @param result instance where the result will be stored.
     * @return true if the current velocity drift is available and the result is updated,
     * false otherwise.
     */
    public boolean getCurrentVelocityDriftNorm(final Speed result) {
        final var velocityDrift = getCurrentVelocityDriftNormMetersPerSecond();
        if (velocityDrift != null) {
            result.setValue(velocityDrift);
            result.setUnit(SpeedUnit.METERS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets current amount of orientation drift for last processed body kinematics
     * measurement expressed in radians (rad).
     *
     * @return current amount of orientation drift or null.
     */
    public Double getCurrentOrientationDriftRadians() {
        return numberOfProcessedSamples > 0 ? currentOrientationDriftRadians : null;
    }

    /**
     * Gets current amount of orientation drift for last processed body kinematics
     * measurement.
     *
     * @return current amount of orientation drift or null.
     */
    public Angle getCurrentOrientationDriftAngle() {
        final var orientationDrift = getCurrentOrientationDriftRadians();
        return orientationDrift != null ? new Angle(orientationDrift, AngleUnit.RADIANS) : null;
    }

    /**
     * Gets current amount of orientation drift for last processed body kinematics
     * measurement.
     *
     * @param result instance where the result will be stored.
     * @return true if the current orientation drift is available and the result is
     * updated, false otherwise.
     */
    public boolean getCurrentOrientationDriftAngle(final Angle result) {
        final var orientationDrift = getCurrentOrientationDriftRadians();
        if (orientationDrift != null) {
            result.setValue(orientationDrift);
            result.setUnit(AngleUnit.RADIANS);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets the current amount of position drift per time unit expressed in meters
     * per second (m/s).
     *
     * @return current amount of position drift per time unit or null.
     */
    public Double getCurrentPositionDriftPerTimeUnit() {
        final var positionDrift = getCurrentPositionDriftNormMeters();
        final var elapsedTime = getElapsedTimeSeconds();
        return positionDrift != null ? positionDrift / elapsedTime : null;
    }

    /**
     * Gets the current amount of position drift per time unit.
     *
     * @return current amount of position drift per time unit or null.
     */
    public Speed getCurrentPositionDriftPerTimeUnitAsSpeed() {
        final var positionDriftPerTimeUnit = getCurrentPositionDriftPerTimeUnit();
        return positionDriftPerTimeUnit != null ? new Speed(positionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)
                : null;
    }

    /**
     * Gets the current amount of position drift per time unit.
     *
     * @param result instance where the result will be stored.
     * @return true if the current amount of position drift per time unit is
     * available and the result is updated, false otherwise.
     */
    public boolean getCurrentPositionDriftPerTimeUnitAsSpeed(final Speed result) {
        final var positionDriftPerTimeUnit = getCurrentPositionDriftPerTimeUnit();
        if (positionDriftPerTimeUnit != null) {
            result.setValue(positionDriftPerTimeUnit);
            result.setUnit(SpeedUnit.METERS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets the current amount of velocity drift per time unit expressed in meters
     * per squared second (m/s^2).
     *
     * @return the current amount of velocity drift per time unit or null.
     */
    public Double getCurrentVelocityDriftPerTimeUnit() {
        final var velocityDrift = getCurrentVelocityDriftNormMetersPerSecond();
        final var elapsedTime = getElapsedTimeSeconds();
        return velocityDrift != null ? velocityDrift / elapsedTime : null;
    }

    /**
     * Gets the current amount of velocity drift per time unit.
     *
     * @return the current amount of velocity drift per time unit or null.
     */
    public Acceleration getCurrentVelocityDriftPerTimeUnitAsAcceleration() {
        final var velocityDriftPerTimeUnit = getCurrentVelocityDriftPerTimeUnit();
        return velocityDriftPerTimeUnit != null
                ? new Acceleration(velocityDriftPerTimeUnit, AccelerationUnit.METERS_PER_SQUARED_SECOND) : null;
    }

    /**
     * Gets the current amount of velocity drift per time unit.
     *
     * @param result instance where the result will be stored.
     * @return true if the current amount of velocity drift per time unit is available
     * and the result is updated, false otherwise.
     */
    public boolean getCurrentVelocityDriftPerTimeUnitAsAcceleration(final Acceleration result) {
        final var velocityDriftPerTimeUnit = getCurrentVelocityDriftPerTimeUnit();
        if (velocityDriftPerTimeUnit != null) {
            result.setValue(velocityDriftPerTimeUnit);
            result.setUnit(AccelerationUnit.METERS_PER_SQUARED_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Gets the current amount of orientation drift per time unit expressed in radians
     * per second (rad/s).
     *
     * @return amount of orientation drift per time unit or null.
     */
    public Double getCurrentOrientationDriftPerTimeUnit() {
        final var orientationDrift = getCurrentOrientationDriftRadians();
        final var elapsedTime = getElapsedTimeSeconds();
        return orientationDrift != null ? orientationDrift / elapsedTime : null;
    }

    /**
     * Gets the current amount of orientation drift per time unit.
     *
     * @return amount of orientation drift per time unit or null.
     */
    public AngularSpeed getCurrentOrientationDriftPerTimeUnitAsAngularSpeed() {
        final var orientationDriftPerTimeUnit = getCurrentOrientationDriftPerTimeUnit();
        return orientationDriftPerTimeUnit != null ?
                new AngularSpeed(orientationDriftPerTimeUnit, AngularSpeedUnit.RADIANS_PER_SECOND) : null;
    }

    /**
     * Gets the current amount of orientation drift per time unit.
     *
     * @param result instance where the result will be stored.
     * @return true if the current amount of orientation drift is available and the result
     * is updated, false otherwise.
     */
    public boolean getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(final AngularSpeed result) {
        final var orientationDriftPerTimeUnit = getCurrentOrientationDriftPerTimeUnit();
        if (orientationDriftPerTimeUnit != null) {
            result.setValue(orientationDriftPerTimeUnit);
            result.setUnit(AngularSpeedUnit.RADIANS_PER_SECOND);
            return true;
        } else {
            return false;
        }
    }

    /**
     * Computes current position drift.
     */
    protected void computeCurrentPositionDrift() {
        final var initX = referenceFrame.getX();
        final var initY = referenceFrame.getY();
        final var initZ = referenceFrame.getZ();

        final var currentX = frame.getX();
        final var currentY = frame.getY();
        final var currentZ = frame.getZ();

        final var diffX = currentX - initX;
        final var diffY = currentY - initY;
        final var diffZ = currentZ - initZ;

        currentPositionDrift.setCoordinates(diffX, diffY, diffZ);

        currentPositionDriftMeters = currentPositionDrift.getNorm();
    }

    /**
     * Computes current velocity drift.
     */
    protected void computeCurrentVelocityDrift() {
        final var initVx = referenceFrame.getVx();
        final var initVy = referenceFrame.getVy();
        final var initVz = referenceFrame.getVz();

        final var currentVx = frame.getVx();
        final var currentVy = frame.getVy();
        final var currentVz = frame.getVz();

        final var diffVx = currentVx - initVx;
        final var diffVy = currentVy - initVy;
        final var diffVz = currentVz - initVz;

        currentVelocityDrift.setCoordinates(diffVx, diffVy, diffVz);

        currentVelocityDriftMetersPerSecond = currentVelocityDrift.getNorm();
    }

    /**
     * Computes current orientation drift.
     *
     * @throws AlgebraException               if there are numerical instabilities.
     * @throws InvalidRotationMatrixException if rotation cannot be accurately
     *                                        estimated.
     */
    protected void computeCurrentOrientationDrift() throws AlgebraException, InvalidRotationMatrixException {
        if (currentC == null) {
            currentC = new Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS);
        }
        frame.getCoordinateTransformationMatrix(currentC);

        q.fromMatrix(currentC);
        q.combine(invRefQ);

        currentOrientationDriftRadians = q.getRotationAngle();
    }

    /**
     * Converts the provided time instance to seconds.
     *
     * @param time instance to be converted.
     * @return conversion in seconds.
     */
    private static double convertTime(final Time time) {
        return TimeConverter.convert(time.getValue().doubleValue(), time.getUnit(), TimeUnit.SECOND);
    }
}
