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

import com.irurueta.navigation.inertial.calibration.StandardDeviationBodyKinematics;
import com.irurueta.statistics.UniformRandomizer;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;

class DefaultAccelerometerQualityScoreMapperTest {

    private static final double MAX_SPECIFIC_FORCE = 9.81;
    private static final double MAX_ANGULAR_RATE_VALUE = 1.0;

    @Test
    public void testMap() {
        final var randomizer = new UniformRandomizer();
        final var specificForceStandardDeviation = randomizer.nextDouble(0.0, MAX_SPECIFIC_FORCE);
        final var angularRateStandardDeviation = randomizer.nextDouble(0.0, MAX_ANGULAR_RATE_VALUE);
        final var stdKinematics = new StandardDeviationBodyKinematics(specificForceStandardDeviation,
                angularRateStandardDeviation);

        final var mapper = new DefaultAccelerometerQualityScoreMapper();

        assertEquals(1.0 / (1.0 + specificForceStandardDeviation + angularRateStandardDeviation),
                mapper.map(stdKinematics), 0.0);
    }
}
