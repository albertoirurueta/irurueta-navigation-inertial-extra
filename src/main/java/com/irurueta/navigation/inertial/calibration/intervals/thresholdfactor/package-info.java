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
/**
 * Threshold factor optimizers, quality score mappers and Mean Square Error (MSE) rules used to select the
 * threshold factor of interval detectors during inertial sensor calibration.
 * <p>
 * Interval detectors classify measurement sequences as static or dynamic, and the threshold factor that
 * separates both states is estimated by optimizing a calibration error. This package provides:
 * <ul>
 * <li>Optimizers that search for the best threshold factor for a single sensor or a combination of sensors,
 * such as {@link com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.GyroscopeIntervalDetectorThresholdFactorOptimizer},
 * {@link com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer}
 * and
 * {@link com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.AccelerometerGyroscopeAndMagnetometerIntervalDetectorThresholdFactorOptimizer}.
 * Exhaustive and bracketed variants of these optimizers are also provided.</li>
 * <li>Data sources and listeners used to provide data to, and to be notified about the progress of, the
 * optimizers.</li>
 * <li>Quality score mappers that convert calibration sequences into quality scores, such as
 * {@link com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultGyroscopeQualityScoreMapper}.</li>
 * <li>MSE rules that combine the calibration Mean Square Errors of each sensor into a single value, such as
 * {@link com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerAndGyroscopeMseRule}
 * and
 * {@link com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.DefaultAccelerometerGyroscopeAndMagnetometerMseRule}.</li>
 * </ul>
 */
package com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor;
