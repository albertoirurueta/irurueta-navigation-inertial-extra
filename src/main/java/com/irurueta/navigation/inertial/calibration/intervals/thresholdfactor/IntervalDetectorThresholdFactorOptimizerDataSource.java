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
 * Retrieves data for a {@link IntervalDetectorThresholdFactorOptimizer}.
 *
 * @param <T> type of data to be retrieved.
 */
public interface IntervalDetectorThresholdFactorOptimizerDataSource<T> {

    /**
     * Returns the total amount of data.
     *
     * @return total amount of data.
     */
    int count();

    /**
     * Gets data at the provided position.
     *
     * @param index position to retrieve data from.
     * @return retrieved data.
     */
    T getAt(int index);
}
