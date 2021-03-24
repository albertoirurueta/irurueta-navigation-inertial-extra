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

import com.irurueta.navigation.inertial.BodyKinematics;

/**
 * Listener to handle events generated by {@link RandomWalkEstimator}.
 */
public interface RandomWalkEstimatorListener {

    /**
     * Called when estimation starts for a single body kinematics measurement.
     *
     * @param estimator estimator that raised the event.
     */
    void onStart(final RandomWalkEstimator estimator);

    /**
     * Called when a measured body kinematics sample is added containing new
     * measured accelerometer and gyroscope measures.
     *
     * @param estimator estimator that raised the event.
     * @param measuredKinematics measured kinematics being added.
     * @param fixedKinematics fixed kinematics taking into account calibration data.
     */
    void onBodyKinematicsAdded(final RandomWalkEstimator estimator,
                               final BodyKinematics measuredKinematics,
                               final BodyKinematics fixedKinematics);

    /**
     * Called when estimation is reset.
     *
     * @param estimator estimator that raised the event.
     */
    void onReset(final RandomWalkEstimator estimator);
}