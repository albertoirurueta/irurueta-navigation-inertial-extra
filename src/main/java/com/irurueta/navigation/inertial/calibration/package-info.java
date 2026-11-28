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
 * Common types and estimators used during inertial sensor calibration and random walk and drift estimation.
 * <p>
 * This package provides:
 * <ul>
 * <li>Estimators of accumulated drift in body orientation, position and velocity, such as
 * {@link com.irurueta.navigation.inertial.calibration.RandomWalkEstimator} and
 * {@link com.irurueta.navigation.inertial.calibration.KalmanDriftEstimator}, together with the
 * {@link com.irurueta.navigation.inertial.calibration.DriftEstimator} they build upon and their listeners.</li>
 * <li>Sources of estimated random walk, uncertainty and noise values, such as
 * {@link com.irurueta.navigation.inertial.calibration.AccelerometerBiasRandomWalkSource} and
 * {@link com.irurueta.navigation.inertial.calibration.PositionNoiseStandardDeviationSource}.</li>
 * <li>Creators that combine estimated data into configurations for INS loosely coupled Kalman filters, such as
 * {@link com.irurueta.navigation.inertial.calibration.INSLooselyCoupledKalmanConfigCreator}.</li>
 * <li>Exceptions reported by drift and random walk estimation, such as
 * {@link com.irurueta.navigation.inertial.calibration.DriftEstimationException} and
 * {@link com.irurueta.navigation.inertial.calibration.RandomWalkEstimationException}.</li>
 * </ul>
 * Interval detection and threshold factor optimization used by calibration are provided in the
 * {@code intervals} subpackages.
 */
package com.irurueta.navigation.inertial.calibration;
