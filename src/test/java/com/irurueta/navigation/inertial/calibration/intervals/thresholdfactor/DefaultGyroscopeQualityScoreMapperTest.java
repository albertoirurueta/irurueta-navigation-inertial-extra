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

import com.irurueta.navigation.inertial.calibration.BodyKinematicsSequence;
import com.irurueta.navigation.inertial.calibration.StandardDeviationTimedBodyKinematics;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * Unit tests for {@link DefaultGyroscopeQualityScoreMapper}.
 */
class DefaultGyroscopeQualityScoreMapperTest {

    /**
     * Specific force standard deviation of generated items.
     */
    private static final double SPECIFIC_FORCE_STD = 0.5;

    /**
     * Angular rate standard deviation of generated items.
     */
    private static final double ANGULAR_RATE_STD = 0.1;

    /**
     * Time interval between generated items.
     */
    private static final double TIME_INTERVAL = 0.02;

    /**
     * Number of items of a short sequence.
     */
    private static final int SHORT_NUM_ITEMS = 50;

    /**
     * Number of items of a long sequence.
     */
    private static final int LONG_NUM_ITEMS = 200;

    /**
     * Absolute tolerance for comparisons.
     */
    private static final double ABSOLUTE_ERROR = 1e-12;

    /**
     * Checks that score is 1 / (1 + duration).
     */
    @Test
    void map_whenTimestampsAvailable_returnsInverseOfOnePlusDuration() {
        final var sequence = buildSequence(SHORT_NUM_ITEMS, TIME_INTERVAL);
        final var duration = TIME_INTERVAL * (SHORT_NUM_ITEMS - 1);

        final var mapper = new DefaultGyroscopeQualityScoreMapper();
        assertEquals(1.0 / (1.0 + duration), mapper.map(sequence), ABSOLUTE_ERROR);
    }

    /**
     * Checks that sequences with more items (longer duration) score strictly lower.
     */
    @Test
    void map_whenSequenceHasMoreItems_returnsLowerScore() {
        final var mapper = new DefaultGyroscopeQualityScoreMapper();

        final var shortScore = mapper.map(buildSequence(SHORT_NUM_ITEMS, TIME_INTERVAL));
        final var longScore = mapper.map(buildSequence(LONG_NUM_ITEMS, TIME_INTERVAL));

        assertTrue(longScore < shortScore);
    }

    /**
     * Checks that identical items with different duration give different scores.
     */
    @Test
    void map_whenSameItemsButDifferentDuration_returnsDifferentScores() {
        final var mapper = new DefaultGyroscopeQualityScoreMapper();

        final var fast = mapper.map(buildSequence(SHORT_NUM_ITEMS, TIME_INTERVAL));
        final var slow = mapper.map(buildSequence(SHORT_NUM_ITEMS, TIME_INTERVAL * 10.0));

        assertNotEquals(fast, slow);
        assertTrue(slow < fast);
    }

    /**
     * Checks that scores are within (0, 1].
     */
    @Test
    void map_whenValidSequence_returnsScoreInRange() {
        final var mapper = new DefaultGyroscopeQualityScoreMapper();

        for (final var numItems : new int[]{2, SHORT_NUM_ITEMS, LONG_NUM_ITEMS}) {
            final var score = mapper.map(buildSequence(numItems, TIME_INTERVAL));
            assertTrue(score > 0.0);
            assertTrue(score <= 1.0);
        }
    }

    /**
     * Checks that a sequence without items gives a zero score.
     */
    @Test
    void map_whenItemsAreNull_returnsZero() {
        final var mapper = new DefaultGyroscopeQualityScoreMapper();

        assertEquals(0.0, mapper.map(new BodyKinematicsSequence<>()), 0.0);
    }

    /**
     * Checks that a sequence with an empty list of items gives a zero score.
     */
    @Test
    void map_whenItemsAreEmpty_returnsZero() {
        final var mapper = new DefaultGyroscopeQualityScoreMapper();
        final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
        sequence.setItems(new ArrayList<>());

        assertEquals(0.0, mapper.map(sequence), 0.0);
    }

    /**
     * Checks that a sequence without a positive duration (equal timestamps or a single item) gives a zero score,
     * so that it never outranks a sequence that can be integrated.
     */
    @Test
    void map_whenDurationIsNotPositive_returnsZero() {
        final var mapper = new DefaultGyroscopeQualityScoreMapper();

        assertEquals(0.0, mapper.map(buildSequence(SHORT_NUM_ITEMS, 0.0)), 0.0);
        assertEquals(0.0, mapper.map(buildSequence(1, TIME_INTERVAL)), 0.0);
    }

    /**
     * Builds a sequence of items with constant standard deviations.
     *
     * @param numItems     number of items.
     * @param timeInterval time interval between consecutive items.
     * @return generated sequence.
     */
    private static BodyKinematicsSequence<StandardDeviationTimedBodyKinematics> buildSequence(
            final int numItems, final double timeInterval) {
        final var items = new ArrayList<StandardDeviationTimedBodyKinematics>();
        for (var i = 0; i < numItems; i++) {
            items.add(new StandardDeviationTimedBodyKinematics(timeInterval * i, SPECIFIC_FORCE_STD,
                    ANGULAR_RATE_STD));
        }
        return new BodyKinematicsSequence<>(items);
    }
}
