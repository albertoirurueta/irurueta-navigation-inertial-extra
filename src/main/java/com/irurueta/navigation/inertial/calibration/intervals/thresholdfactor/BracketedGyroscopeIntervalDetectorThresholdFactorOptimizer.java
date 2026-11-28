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
import com.irurueta.navigation.inertial.calibration.CalibrationException;
import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeCalibratorMeasurementOrSequenceType;
import com.irurueta.navigation.inertial.calibration.gyroscope.GyroscopeNonLinearCalibrator;
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
 * Only gyroscope calibrators based on unknown orientation are supported (in other terms,
 * calibrators must be {@link GyroscopeNonLinearCalibrator} and must support
 * {@link GyroscopeCalibratorMeasurementOrSequenceType#BODY_KINEMATICS_SEQUENCE}).
 * This implementation uses a {@link BracketedSingleOptimizer} to find the threshold
 * factor value that minimizes Mean Square Error (MSE) for calibration parameters.
 * <p>
 * The search range is the one defined by the minimum and maximum threshold factors
 * (see {@code setThresholdFactorRange}). The bracket of the underlying optimizer is updated
 * with that range (using its centre as intermediate point) each time an optimizer is set and
 * before each optimization, so changes of the range are always honoured.
 * <p>
 * Notice that the objective function (MSE as a function of the threshold factor) is piecewise
 * constant, since the detected static intervals only change at discrete threshold factor
 * values. Hence, it contains plateaus where gradient-free bracketed methods such as Brent's
 * cannot make progress, and the returned threshold factor is the best one <i>evaluated</i> by
 * the optimizer, which is not necessarily the minimum within the range.
 * For a reliable result, the {@code Exhaustive*} threshold factor optimizers are recommended
 * as the default choice.
 */
public class BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer extends
        GyroscopeIntervalDetectorThresholdFactorOptimizer {

    /**
     * Large finite value returned as the objective function value for threshold factors that
     * are not valid (detection or calibration failed). A finite value is used instead of
     * {@link Double#MAX_VALUE} to avoid numerical overflow within the bracketed optimizer.
     */
    private static final double INVALID_MSE_PENALTY = 1e30;

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
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer() {
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
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
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
     * @param calibrator a gyroscope calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences.
     */
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer(final GyroscopeNonLinearCalibrator calibrator) {
        super(calibrator);
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
     * @param calibrator a gyroscope calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences.
     */
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final GyroscopeNonLinearCalibrator calibrator) {
        super(dataSource, calibrator);
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
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer(final BracketedSingleOptimizer mseOptimizer) {
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
     * @param mseOptimizer optimizer to find the threshold factor value that
     *                     minimizes MSE for calibration parameters.
     */
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
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
     * @param calibrator   a gyroscope calibrator to be used to optimize its
     *                     Mean Square Error (MSE).
     * @param mseOptimizer optimizer to find the threshold factor value that
     *                     minimizes MSE for calibration parameters.
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences.
     */
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeNonLinearCalibrator calibrator, final BracketedSingleOptimizer mseOptimizer) {
        super(calibrator);
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
     * @param calibrator   a gyroscope calibrator to be used to optimize its
     *                     Mean Square Error (MSE).
     * @param mseOptimizer optimizer to find the threshold factor value that
     *                     minimizes MSE for calibration parameters.
     * @throws IllegalArgumentException if gyroscope calibrator does not use
     *                                  {@link BodyKinematicsSequence} sequences.
     */
    public BracketedGyroscopeIntervalDetectorThresholdFactorOptimizer(
            final GyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final GyroscopeNonLinearCalibrator calibrator,
            final BracketedSingleOptimizer mseOptimizer) {
        super(dataSource, calibrator);
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
                optimizer.setListener(optimizerListener);
                optimizer.setOnIterationCompletedListener(iterationCompletedListener);
            }
            mseOptimizer = optimizer;
            if (optimizer != null) {
                updateBracket();
            }
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
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
     * generator to minimize MSE (Mean Square Error) of estimated
     * calibration parameters.
     *
     * @return optimized threshold factor.
     * @throws NotReadyException                                 if this optimizer is not ready to start optimization.
     * @throws LockedException                                   if optimizer is already running.
     * @throws IntervalDetectorThresholdFactorOptimizerException if optimization fails for
     *                                                           some reason, or if no threshold
     *                                                           factor produced a valid calibration
     *                                                           result (in which case no result is kept).
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

            resetResults();
            updateBracket();
            mseOptimizer.minimize();

            if (minMse >= INVALID_MSE_PENALTY) {
                // no threshold factor produced a valid calibration result
                throw new IntervalDetectorThresholdFactorOptimizerException();
            }

            if (listener != null) {
                // the bracketed optimizer can converge in so few iterations that no progress change is
                // ever notified, so completion is always notified
                notifyProgressCompleted();
                listener.onOptimizeEnd(this);
            }

            return optimalThresholdFactor;
        } catch (final com.irurueta.numerical.LockedException e) {
            throw new LockedException(e);
        } catch (final NumericalException e) {
            throw new IntervalDetectorThresholdFactorOptimizerException(e);
        } finally {
            running = false;
        }
    }

    /**
     * Updates the bracket of the MSE optimizer so that it spans the current threshold factor
     * range, using the centre of the range as intermediate point.
     *
     * @throws com.irurueta.numerical.LockedException if the MSE optimizer is locked.
     */
    private void updateBracket() throws com.irurueta.numerical.LockedException {
        final var centre = 0.5 * (minThresholdFactor + maxThresholdFactor);
        try {
            mseOptimizer.setBracket(minThresholdFactor, centre, maxThresholdFactor);
        } catch (final InvalidBracketRangeException ignore) {
            // never happens because the threshold factor range is validated when set
        }
    }

    /**
     * Initializes optimizer listener.
     */
    private void initializeOptimizerListeners() {
        optimizerListener = point -> {
            try {
                final var mse = evaluateForThresholdFactor(point);
                // invalid evaluations (MAX_VALUE) and non-finite values are mapped to a finite penalty
                return Double.isFinite(mse) && mse < INVALID_MSE_PENALTY ? mse : INVALID_MSE_PENALTY;
            } catch (final CalibrationException | IntervalDetectorThresholdFactorOptimizerException ignore) {
                // detection or calibration failed for this threshold factor, so it is not valid
                return INVALID_MSE_PENALTY;
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
