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
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyMagneticFluxDensity;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerCalibratorMeasurementType;
import com.irurueta.navigation.inertial.calibration.magnetometer.MagnetometerNonLinearCalibrator;

/**
 * Optimizes the threshold factor for interval detection of magnetometer data based on
 * results of calibration.
 * Only magnetometer calibrators based on unknown orientation are supported, in other terms,
 * calibrators must be {@link MagnetometerNonLinearCalibrator} and must support
 * {@link MagnetometerCalibratorMeasurementType#STANDARD_DEVIATION_BODY_MAGNETIC_FLUX_DENSITY}.
 * This implementation makes an exhaustive search between minimum and maximum
 * threshold factor values to find the threshold value that produces the
 * minimum Mean Square Error (MSE) for calibration parameters.
 */
public class ExhaustiveMagnetometerIntervalDetectorThresholdFactorOptimizer extends
        MagnetometerIntervalDetectorThresholdFactorOptimizer {

    /**
     * Default step value to make exhaustive search of threshold factor values.
     */
    public static final double DEFAULT_STEP = 1.0;

    /**
     * Step to make exhaustive search of threshold factor values between
     * {@link #getMaxThresholdFactor()} and {@link #getMaxThresholdFactor()}.
     */
    private double thresholdFactorStep = DEFAULT_STEP;

    /**
     * Constructor.
     */
    public ExhaustiveMagnetometerIntervalDetectorThresholdFactorOptimizer() {
        super();
    }

    /**
     * Constructor.
     *
     * @param dataSource instance in charge of retrieving data for this optimizer.
     */
    public ExhaustiveMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource) {
        super(dataSource);
    }

    /**
     * Constructor.
     *
     * @param calibrator a magnetometer calibrator to be used to optimize its
     *                   Mean Square Error (MSE).
     * @throws IllegalArgumentException if magnetometer calibrator does not use
     *                                  {@link StandardDeviationBodyMagneticFluxDensity} measurements.
     */
    public ExhaustiveMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerNonLinearCalibrator calibrator) {
        super(calibrator);
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
    public ExhaustiveMagnetometerIntervalDetectorThresholdFactorOptimizer(
            final MagnetometerIntervalDetectorThresholdFactorOptimizerDataSource dataSource,
            final MagnetometerNonLinearCalibrator calibrator) {
        super(dataSource, calibrator);
    }

    /**
     * Gets the step to make exhaustive search of threshold values between
     * {@link #getMaxThresholdFactor()} and {@link #getMaxThresholdFactor()}.
     *
     * @return step to make exhaustive search of threshold values.
     */
    public double getThresholdFactorStep() {
        return thresholdFactorStep;
    }

    /**
     * Sets step to make exhaustive search of threshold values between
     * {@link #getMaxThresholdFactor()} and {@link #getMaxThresholdFactor()}.
     *
     * @param thresholdStep step to make exhaustive search of threshold values.
     * @throws LockedException          if optimizer is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    public void setThresholdFactorStep(final double thresholdStep) throws LockedException {
        if (running) {
            throw new LockedException();
        }
        if (thresholdStep <= 0.0) {
            throw new IllegalArgumentException();
        }

        thresholdFactorStep = thresholdStep;
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

        var hasResult = false;
        minMse = Double.MAX_VALUE;
        try {
            running = true;

            initProgress();
            final var progressStep = (float) (thresholdFactorStep
                    / (thresholdFactorStep + maxThresholdFactor - minThresholdFactor));

            if (listener != null) {
                listener.onOptimizeStart(this);
            }

            for (var thresholdFactor = minThresholdFactor;
                 thresholdFactor <= maxThresholdFactor;
                 thresholdFactor += thresholdFactorStep) {
                try {
                    evaluateForThresholdFactor(thresholdFactor);
                    hasResult = true;
                } catch (final Exception ignore) {
                    // when an error occurs, skip to the next iteration
                }

                progress += progressStep;
                checkAndNotifyProgress();
            }

            if (!hasResult) {
                throw new IntervalDetectorThresholdFactorOptimizerException();
            }

            if (listener != null) {
                listener.onOptimizeEnd(this);
            }

        } finally {
            running = false;
        }

        return optimalThresholdFactor;
    }
}
