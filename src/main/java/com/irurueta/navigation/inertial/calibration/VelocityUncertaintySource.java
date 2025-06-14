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

import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanConfig;
import com.irurueta.navigation.inertial.INSLooselyCoupledKalmanInitializerConfig;

/**
 * Defines a source for velocity uncertainty.
 */
public interface VelocityUncertaintySource {

    /**
     * Gets velocity uncertainty expressed in meters per second (m/s).
     * Velocity uncertainty can be measured using a RandomWalkEstimator,
     * which is appropriate to create a {@link INSLooselyCoupledKalmanConfig} or
     * a {@link INSLooselyCoupledKalmanInitializerConfig}.
     * However, to create a {@link INSLooselyCoupledKalmanInitializerConfig},
     * typically position, velocity and attitude are externally measured by other
     * means introducing a different amount of uncertainty (e.g., GPS).
     *
     * @return velocity uncertainty expressed in meters per second (m/s).
     */
    double getVelocityUncertainty();
}
