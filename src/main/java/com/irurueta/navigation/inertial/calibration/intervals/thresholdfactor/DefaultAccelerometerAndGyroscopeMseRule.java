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

/**
 * Provides a rule or function to convert estimated accelerometer calibration
 * Mean Square Error (MSE) and estimated gyroscope calibration Mean Square Error
 * (MSE) into a single MSE value.
 * This implementation returns the accelerometer MSE and ignores the gyroscope
 * MSE.
 * <p>
 * Units reasoning: the accelerometer MSE is expressed in (m/s^2)^4, the
 * gyroscope MSE is related to a unit-vector distance and the magnetometer MSE
 * is expressed in T^4. Those magnitudes are not commensurable, so adding them
 * makes the term with the largest numeric scale dominate the result in an
 * arbitrary way.
 * <p>
 * This implementation follows the behaviour of imu_tk, where the threshold
 * factor is selected using the accelerometer calibration error only, and
 * returns the accelerometer MSE ignoring the remaining terms.
 * <p>
 * Custom rules can be provided to the optimizers if a different weighting of
 * the terms is desired.
 */
public class DefaultAccelerometerAndGyroscopeMseRule implements AccelerometerAndGyroscopeMseRule {

    /**
     * Evaluates provided accelerometer calibration MSE and gyroscope calibration
     * MSE to get a single MSE value.
     * This implementation returns the accelerometer MSE and ignores the gyroscope MSE.
     *
     * @param accelerometerMse accelerometer calibration MSE.
     * @param gyroscopeMse     gyroscope calibration MSE.
     * @return single MSE value, equal to the accelerometer MSE.
     */
    @Override
    public double evaluate(double accelerometerMse, double gyroscopeMse) {
        return accelerometerMse;
    }
}
