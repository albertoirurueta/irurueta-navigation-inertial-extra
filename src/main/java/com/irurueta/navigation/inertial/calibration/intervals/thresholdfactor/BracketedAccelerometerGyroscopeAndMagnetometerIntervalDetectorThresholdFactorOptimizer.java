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

import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NavigationException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerCalibratorMeasurementType;
import com.irurueta.navigation.inertial.calibration.accelerometer.AccelerometerNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeCalibratorMeasurementOrSequenceType;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerCalibratorMeasurementType;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator;
import com.irurueta.numerical.EvaluationException;
import com.irurueta.numerical.InvalidBracketRangeException;
import com.irurueta.numerical.NumericalException;
import com.irurueta.numerical.SingleDimensionFunctionEvaluatorListener;
import com.irurueta.numerical.optimization.BracketedSingleOptimizer;
import com.irurueta.numerical.optimization.BrentSingleOptimizer;
import com.irurueta.numerical.optimization.OnIterationCompletedListener;

/**
 * Optimizes the threshold factor for interval detection of accelerometer and gyroscope
 * data based on results of calibration.
 * Only accelerometer calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link AccelerometerNonLinearCalibrator} and must support
 * {@link AccelerometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_KINEMATICS}).
 * Only gyroscope calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link GyroscopeNonLinearCalibrator} and must support
 * {@link GyroscopeCalibratorMeasurementOrSequenceType#BODY_KINEMATICS_SEQUENCE}).
 * Only magnetometer calibrators based on unknown orientation are supported, in other terms,
 * calibrators must be {@link MagnetometerNonLinearCalibrator} and must support
 * {@link MagnetometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY}.
 * This implementation uses a {@link BracketedSingleOptimizer} to find the threshold
 * factor value that minimizes Mean Square Error (MSE) for calibration parameters.
 */
public class BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer extends
        AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer {

    /**
     * A bracketed single optimizer to find the threshold factor value that
     * minimizes the Mean Square Error (MSE) for calibration parameters.
     */
    private BracketedSingleOptimizer mseOptimizer;

    /**
     * Listener for optimizer.
     */
    private SingleDimensionFunctionEvaluatorListener optimizerListener;

    /**
     * Iteration listener for {@link BracketedSingleOptimizer}.
     */
    private OnIterationCompletedListener iterationCompletedListener;

    /**
     * Constructor.
     */
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer() {
        super();
        initializeOptimizerListeners();
        try {
            setMseOptimizer(new BrentSingleOptimizer());
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
        initializeOptimizerListeners();
        try {
            setMseOptimizer(new BrentSingleOptimizer());
        } catch (final LockedException ignore) {
            // never happens
        }
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
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator) {
        super(accelerometerCalibrator, gyroscopeCalibrator, magnetometerCalibrator);
        initializeOptimizerListeners();
        try {
            setMseOptimizer(new BrentSingleOptimizer());
        } catch (final LockedException ignore) {
            // never happens
        }
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
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator) {
        super(dataSource, accelerometerCalibrator, gyroscopeCalibrator, magnetometerCalibrator);
        initializeOptimizerListeners();
        try {
            setMseOptimizer(new BrentSingleOptimizer());
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param mseOptimizer optimizer to find the threshold factor value that
     *                     minimizes MSE for calibration parameters.
     */
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final BracketedSingleOptimizer mseOptimizer) {
        super();
        initializeOptimizerListeners();
        try {
            setMseOptimizer(mseOptimizer);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Constructor.
     *
     * @param dataSource   instance in charge of retrieving data for this optimizer.
     * @param mseOptimizer optimizer to find the threshold value that minimizes
     *                     MSE for calibration parameters.
     */
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final BracketedSingleOptimizer mseOptimizer) {
        super(dataSource);
        initializeOptimizerListeners();
        try {
            setMseOptimizer(mseOptimizer);
        } catch (final LockedException ignore) {
            // never happens
        }
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
     * @param mseOptimizer            optimizer to find the threshold value that
     *                                minimizes MSE for calibration parameters.
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements,
     *                                  if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics} or
     *                                  if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator,
            final BracketedSingleOptimizer mseOptimizer) {
        super(accelerometerCalibrator, gyroscopeCalibrator, magnetometerCalibrator);
        initializeOptimizerListeners();
        try {
            setMseOptimizer(mseOptimizer);
        } catch (final LockedException ignore) {
            // never happens
        }
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
     * @param mseOptimizer            optimizer to find the threshold value that
     *                                minimizes MSE for calibration parameters.
     * @throws IllegalArgumentException if accelerometer calibrator does not use
     *                                  {@link StandardDeviationBodyKinematics} measurements,
     *                                  if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences of
     *                                  {@link StandardDeviationTimedBodyKinematics} or
     *                                  if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public BracketedAccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final AccelerometerNonLinearCalibrator accelerometerCalibrator,
            final GyroscopeNonLinearCalibrator gyroscopeCalibrator,
            final MagnetometerNonLinearCalibrator magnetometerCalibrator,
            final BracketedSingleOptimizer mseOptimizer) {
        super(dataSource, accelerometerCalibrator, gyroscopeCalibrator, magnetometerCalibrator);
        initializeOptimizerListeners();
        try {
            setMseOptimizer(mseOptimizer);
        } catch (final LockedException ignore) {
            // never happens
        }
    }

    /**
     * Gets the bracketed single optimizer used to find the threshold factor value
     * that minimizes the Mean Square Error (MSE) for calibration parameters.
     *
     * @return optimizer to find the threshold factor value that minimizes the
     * MSE for calibration parameters.
     */
    public BracketedSingleOptimizer getMseOptimizer() {
        return mseOptimizer;
    }

    /**
     * Sets the bracketed single optimizer used to find the threshold factor value
     * that minimizes the Mean Square Error (MSE) for calibration parameters.
     *
     * @param optimizer optimizer to find the threshold factor value that minimizes
     *                  the MSE for calibration parameters.
     * @throws LockedException if optimizer is already running.
     */
    public void setMseOptimizer(final BracketedSingleOptimizer optimizer)
            throws LockedException {
        if (running) {
            throw new LockedException();
        }

        try {
            if (optimizer != null) {
                optimizer.setBracket(minThresholdFactor, minThresholdFactor, maxThresholdFactor);
                optimizer.setListener(optimizerListener);
                optimizer.setOnIterationCompletedListener(iterationCompletedListener);
            }
            mseOptimizer = optimizer;
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final InvalidBracketRangeException ignore) {
            // never happens
        }
    }

    /**
     * Indicates whether this optimizer is ready to start optimization.
     *
     * @return true if this optimizer is ready, false otherwise.
     */
    @Override
    public boolean isReady() {
        return super.isReady() && mseOptimizer != null;
    }

    /**
     * Optimizes the threshold factor for a static interval detector or measurement
     * generator to minimize MSE (Minimum Squared Error) of estimated
     * calibration parameters.
     *
     * @return optimized threshold factor.
     * @throws NotReadyException                                 if this optimizer is not ready to start optimization.
     * @throws LockedException                                   if optimizer is already running.
     * @throws IntervalDetectorThresholdFactorOptimizerException if optimization fails for
     *                                                           some reason.
     */
    @Override
    public double optimize() throws NotReadyException, LockedException,
            IntervalDetectorThresholdFactorOptimizerException {
        if (running) {
            throw new LockedException();
        }

        if (!isReady()) {
            throw new NotReadyException();
        }

        try {
            running = true;

            initProgress();

            if (listener != null) {
                listener.onOptimizeStart(this);
            }

            minMse = Double.MAX_VALUE;
            mseOptimizer.minimize();

            if (listener != null) {
                listener.onOptimizeEnd(this);
            }

            return optimalThresholdFactor;
        } catch (final NumericalException e) {
            throw new IntervalDetectorThresholdFactorOptimizerException(e);
        } finally {
            running = false;
        }
    }

    /**
     * Initializes optimizer listener.
     */
    private void initializeOptimizerListeners() {
        optimizerListener = point -> {
            try {
                return evaluateForThresholdFactor(point);
            } catch (final NavigationException e) {
                throw new EvaluationException(e);
            }
        };

        iterationCompletedListener = (optimizer, iteration, maxIterations) -> {
            if (maxIterations == null) {
                return;
            }

            progress = (float) iteration / (float) maxIterations;
            checkAndNotifyProgress();
        };
    }
}
