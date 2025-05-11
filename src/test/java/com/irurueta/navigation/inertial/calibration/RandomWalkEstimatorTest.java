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

import com.irurueta.algebra.AlgebraException;
import com.irurueta.algebra.Matrix;
import com.irurueta.algebra.WrongSizeException;
import com.irurueta.geometry.InhomogeneousPoint3D;
import com.irurueta.geometry.Point3D;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.CoordinateTransformation;
import com.irurueta.navigation.frames.ECEFFrame;
import com.irurueta.navigation.frames.FrameType;
import com.irurueta.navigation.frames.InvalidSourceAndDestinationFrameTypeException;
import com.irurueta.navigation.frames.NEDFrame;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.geodesic.Constants;
import com.irurueta.navigation.inertial.BodyKinematics;
import com.irurueta.navigation.frames.ECEFPosition;
import com.irurueta.navigation.frames.NEDPosition;
import com.irurueta.navigation.inertial.calibration.bias.BodyKinematicsBiasEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.jupiter.api.Test;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class RandomWalkEstimatorTest implements RandomWalkEstimatorListener {
    private static final double TIME_INTERVAL_SECONDS = 0.02;

    private static final double MICRO_G_TO_METERS_PER_SECOND_SQUARED = 9.80665E-6;
    private static final double DEG_TO_RAD = 0.01745329252;

    private static final double MIN_ANGLE_DEGREES = -180.0;
    private static final double MAX_ANGLE_DEGREES = 180.0;

    private static final double MIN_LATITUDE_DEGREES = -90.0;
    private static final double MAX_LATITUDE_DEGREES = 90.0;
    private static final double MIN_LONGITUDE_DEGREES = -180.0;
    private static final double MAX_LONGITUDE_DEGREES = 180.0;
    private static final double MIN_HEIGHT = -50.0;
    private static final double MAX_HEIGHT = 50.0;

    private static final int N_SAMPLES = 100000;

    private static final int TIMES = 100;

    private static final double ABSOLUTE_ERROR = 1e-8;
    private static final double LARGE_ABSOLUTE_ERROR = 1e-6;
    private static final double VERY_LARGE_ABSOLUTE_ERROR = 1e-3;

    private int start;
    private int bodyKinematicsAdded;
    private int reset;

    @Test
    void testConstructor1() throws WrongSizeException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES 
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES 
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var estimator = new RandomWalkEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    void testConstructor3() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, wrong, bgTriad, mg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, m1, bgTriad, mg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, m2, bgTriad, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m1));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m2));
    }

    @Test
    void testConstructor4() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, wrong, bgTriad, mg, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, wrong, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, m1, bgTriad, mg,
                this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, m2, bgTriad, mg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m1,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m2,
                this));
    }

    @Test
    void testConstructor5() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();
        final var estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, wrong, bgTriad, mg, gg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, wrong, gg));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, m1, bgTriad, mg, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, m2, bgTriad, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m1, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m2, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, mg, m1));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, mg, m2));
    }

    @Test
    void testConstructor6() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();
        final var estimator = new RandomWalkEstimator(baTriad, ma, bgTriad, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, wrong, bgTriad, mg, gg,
                this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, wrong, gg,
                this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, m1, bgTriad, mg, gg,
                this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad,
                m2, bgTriad, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m1, gg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, m2, gg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, mg, m1,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(baTriad, ma, bgTriad, mg, m2,
                this));
    }

    @Test
    void testConstructor7() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var estimator = new RandomWalkEstimator(ba, ma, bg, mg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, wrong, bg, mg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, ma, bg, wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m1, ma, bg, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m2, ma, bg, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m3, bg, mg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m4, bg, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m1, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m2, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m3));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m4));
    }

    @Test
    void testConstructor8() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var estimator = new RandomWalkEstimator(ba, ma, bg, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, wrong, bg, mg, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, ma, bg, wrong, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m1, ma, bg, mg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m2, ma, bg, mg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m3, bg, mg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m4, bg, mg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m1, mg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m3, mg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m3, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m4, this));
    }

    @Test
    void testConstructor9() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();
        final var estimator = new RandomWalkEstimator(ba, ma, bg, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, wrong, bg, mg, gg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, ma, bg, wrong, gg));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m1, ma, bg, mg, gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m2, ma, bg, mg, gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m3, bg, mg, gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m4, bg, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m1, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m3, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m3, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m4, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, mg, m3));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, mg, m4));
    }

    @Test
    void testConstructor10() throws AlgebraException {
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();
        final var estimator = new RandomWalkEstimator(ba, ma, bg, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame();
        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition1.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC1.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertEquals(nedC1, nedC2);

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, wrong, bg, mg, gg, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ba, ma, bg, wrong, gg, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m1, ma, bg, mg, gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(m2, ma, bg, mg, gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m3, bg, mg, gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, m4, bg, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m1, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, m3, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m3, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, m4, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, mg, m3, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ba, ma, bg, mg, m4, this));
    }

    @Test
    void testConstructor11() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    void testConstructor12() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    void testConstructor13() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2));
    }

    @Test
    void testConstructor14() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2, this));
    }

    @Test
    void testConstructor15() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baTriad.getMeasurementZ(), baz1);
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg, gg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong, gg));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m1));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m2));
    }

    @Test
    void testConstructor16() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg, gg, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong, gg, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg, gg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m1, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m2, this));
    }

    @Test
    void testConstructor17() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4));
    }

    @Test
    void testConstructor18() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg,
                this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong,
                this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4,
                this));
    }

    @Test
    void testConstructor19() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg, gg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong, gg));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg,
                gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg,
                gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg,
                gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m3));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m4));
    }

    @Test
    void testConstructor20() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg, gg,
                this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong, gg,
                this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg,
                gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg,
                gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg,
                gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m3, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m4, this));
    }

    @Test
    void testConstructor21() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    void testConstructor22() throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad3.getValueX(), 0.0);
        assertEquals(0.0, triad3.getValueY(), 0.0);
        assertEquals(0.0, triad3.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad3.getUnit());
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);
    }

    @Test
    void testConstructor23() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), 10.0 * LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, 10.0 * LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong, bgTriad,
                mg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad,
                wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m1,
                bgTriad, mg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                bgTriad, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m1));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m2));
    }

    @Test
    void testConstructor24() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), VERY_LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, VERY_LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), VERY_LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, VERY_LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), VERY_LARGE_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, VERY_LARGE_ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), VERY_LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, VERY_LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), VERY_LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, VERY_LARGE_ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), VERY_LARGE_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, VERY_LARGE_ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong, bgTriad,
                mg, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad,
                wrong, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m1,
                bgTriad, mg, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                bgTriad, mg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m1, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m2, this));
    }

    @Test
    void testConstructor25() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong, bgTriad,
                mg, gg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad,
                wrong, gg));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m1,
                bgTriad, mg, gg));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                bgTriad, mg, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m1, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m2, gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, mg, m1));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, mg, m2));
    }

    @Test
    void testConstructor26() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var nedPosition = createPosition();
            final var nedC = createOrientation();
            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
            final var ecefPosition = ecefFrame.getECEFPosition();

            final var ba = generateBa();
            final var baTriad = new AccelerationTriad();
            baTriad.setValueCoordinates(ba);
            final var ma = generateMaGeneral();
            final var bg = generateBg();
            final var bgTriad = new AngularSpeedTriad();
            bgTriad.setValueCoordinates(bg);
            final var mg = generateMg();
            final var gg = generateGg();

            final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg,
                    this);

            // check default values
            assertSame(this, estimator.getListener());

            final var ba1 = estimator.getAccelerationBias();
            assertEquals(ba, ba1);
            final var ba2 = new Matrix(3, 1);
            estimator.getAccelerationBias(ba2);
            assertEquals(ba1, ba2);

            final var ba3 = estimator.getAccelerationBiasArray();
            assertArrayEquals(ba.getBuffer(), ba3, 0.0);
            final var ba4 = new double[3];
            estimator.getAccelerationBiasArray(ba4);
            assertArrayEquals(ba3, ba4, 0.0);

            final var triad1 = estimator.getAccelerationBiasAsTriad();
            assertEquals(baTriad, triad1);
            final var triad2 = new AccelerationTriad();
            estimator.getAccelerationBiasAsTriad(triad2);
            assertEquals(triad1, triad2);

            assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
            assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
            assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

            final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
            assertEquals(bax1, baTriad.getMeasurementX());
            final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAccelerationBiasXAsAcceleration(bax2);
            assertEquals(bax1, bax2);

            final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
            assertEquals(bay1, baTriad.getMeasurementY());
            final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAccelerationBiasYAsAcceleration(bay2);
            assertEquals(bay1, bay2);

            final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
            assertEquals(baz1, baTriad.getMeasurementZ());
            final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAccelerationBiasZAsAcceleration(baz2);
            assertEquals(baz1, baz2);

            final var ma1 = estimator.getAccelerationCrossCouplingErrors();
            assertEquals(ma, ma1);

            final var ma2 = new Matrix(3, 3);
            estimator.getAccelerationCrossCouplingErrors(ma2);
            assertEquals(ma1, ma2);

            var sx = ma.getElementAt(0, 0);
            var sy = ma.getElementAt(1, 1);
            var sz = ma.getElementAt(2, 2);
            var mxy = ma.getElementAt(0, 1);
            var mxz = ma.getElementAt(0, 2);
            var myx = ma.getElementAt(1, 0);
            var myz = ma.getElementAt(1, 2);
            var mzx = ma.getElementAt(2, 0);
            var mzy = ma.getElementAt(2, 1);
            assertEquals(sx, estimator.getAccelerationSx(), 0.0);
            assertEquals(sy, estimator.getAccelerationSy(), 0.0);
            assertEquals(sz, estimator.getAccelerationSz(), 0.0);
            assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
            assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
            assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
            assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
            assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
            assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

            final var bg1 = estimator.getAngularSpeedBias();
            assertEquals(bg, bg1);
            final var bg2 = new Matrix(3, 1);
            estimator.getAngularSpeedBias(bg2);
            assertEquals(bg1, bg2);

            final var bg3 = estimator.getAngularSpeedBiasArray();
            assertArrayEquals(bg3, bg.getBuffer(), 0.0);
            final var bg4 = new double[3];
            estimator.getAngularSpeedBiasArray(bg4);
            assertArrayEquals(bg3, bg4, 0.0);

            final var triad3 = estimator.getAngularSpeedBiasAsTriad();
            assertEquals(bgTriad, triad3);
            final var triad4 = new AngularSpeedTriad();
            estimator.getAngularSpeedBiasAsTriad(triad4);
            assertEquals(triad3, triad4);

            assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
            assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
            assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

            final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
            assertEquals(bgx1, bgTriad.getMeasurementX());
            final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
            assertEquals(bgx1, bgx2);

            final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
            assertEquals(bgy1, bgTriad.getMeasurementY());
            final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
            assertEquals(bgy1, bgy2);

            final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
            assertEquals(bgz1, bgTriad.getMeasurementZ());
            final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
            assertEquals(bgz1, bgz2);

            final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
            assertEquals(mg, mg1);
            final var mg2 = new Matrix(3, 3);
            estimator.getAngularSpeedCrossCouplingErrors(mg2);
            assertEquals(mg1, mg2);

            sx = mg.getElementAt(0, 0);
            sy = mg.getElementAt(1, 1);
            sz = mg.getElementAt(2, 2);
            mxy = mg.getElementAt(0, 1);
            mxz = mg.getElementAt(0, 2);
            myx = mg.getElementAt(1, 0);
            myz = mg.getElementAt(1, 2);
            mzx = mg.getElementAt(2, 0);
            mzy = mg.getElementAt(2, 1);
            assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
            assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
            assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
            assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
            assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
            assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
            assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
            assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
            assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

            final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
            assertEquals(gg, gg1);
            final var gg2 = new Matrix(3, 3);
            estimator.getAngularSpeedGDependantCrossBias(gg2);
            assertEquals(gg1, gg2);

            assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

            final var t1 = estimator.getTimeIntervalAsTime();
            assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, t1.getUnit());

            final var t2 = new Time(1.0, TimeUnit.DAY);
            estimator.getTimeIntervalAsTime(t2);
            assertEquals(t1, t2);

            final var nedFrame1 = new NEDFrame(nedPosition, nedC);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefC1 = ecefFrame1.getCoordinateTransformation();

            if (!ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
            final var ecefPosition2 = new ECEFPosition();
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

            assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
            final var ecefFrame2 = new ECEFFrame();
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

            assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
            final var nedFrame2 = new NEDFrame();
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

            assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
            final var nedPosition2 = new NEDPosition();
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
            final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
            estimator.getEcefC(ecefC2);
            assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

            assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
            final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
            estimator.getNedC(nedC2);
            assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertTrue(estimator.isFixKinematicsEnabled());
            assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
            assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                            * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                    estimator.getDriftPeriodSeconds(), 0.0);
            final var driftPeriod1 = estimator.getDriftPeriod();
            assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                            * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                    driftPeriod1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
            final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
            estimator.getDriftPeriod(driftPeriod2);
            assertEquals(driftPeriod1, driftPeriod2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.isReady());

            assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
            assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
            assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
            assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
            assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
            final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
            assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
            final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
            estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
            assertEquals(positionNoiseStd1, positionNoiseStd2);
            assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
            final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
            assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
            final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
            assertEquals(velocityNoiseStd1, velocityNoiseStd2);
            assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
            final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
            assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
            final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
            estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
            assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
            assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
            final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
            assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
            final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
            estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
            assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
            final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
            assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
            final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
            assertEquals(velocityUncertainty1, velocityUncertainty2);
            assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
            final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
            assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
            final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
            estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
            assertEquals(attitudeUncertainty1, attitudeUncertainty2);

            final var kinematics1 = estimator.getFixedKinematics();
            assertEquals(new BodyKinematics(), kinematics1);
            final var kinematics2 = new BodyKinematics();
            estimator.getFixedKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);

            // Force AlgebraException
            final var wrong = Matrix.identity(3, 3);
            wrong.multiplyByScalar(-1.0);
            assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong,
                    bgTriad, mg, gg, this));
            assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                    bgTriad, wrong, gg, this));

            // Force IllegalArgumentException
            final var m1 = new Matrix(1, 3);
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m1,
                    bgTriad, mg, gg, this));
            final var m2 = new Matrix(3, 1);
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                    bgTriad, mg, gg, this));
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                    bgTriad, m1, gg, this));
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                    bgTriad, m2, gg, this));
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                    bgTriad, mg, m1, this));
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                    bgTriad, mg, m2, this));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testConstructor27() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m4));
    }

    @Test
    void testConstructor28() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg,
                this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong,
                this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg,
                this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg,
                this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg,
                this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3,
                this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m4,
                this));
    }

    @Test
    void testConstructor29() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg, gg);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg, gg));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong, gg));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg,
                gg));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg,
                gg));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg,
                gg));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m4,
                gg));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m3));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m4));
    }

    @Test
    void testConstructor30() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg, gg, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getDriftPeriodSeconds(),
                0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * BodyKinematicsBiasEstimator.DEFAULT_TIME_INTERVAL_SECONDS,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg, gg,
                this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong, gg,
                this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg,
                gg, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg,
                gg, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg,
                gg, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3,
                gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg,
                m4, gg, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m3, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m4, this));
    }

    @Test
    void testConstructor31() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final double timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementY(), bgy1);
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementZ(), bgz1);
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * timeInterval, estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES
                        * timeInterval, driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg, timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong, timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg, timeInterval));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2, timeInterval));
    }

    @Test
    void testConstructor32() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg, timeInterval, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong, timeInterval, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg, timeInterval, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2, timeInterval, this));
    }

    @Test
    void testConstructor33() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg, gg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg, gg, timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong, gg, timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg, gg, timeInterval));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg, gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1, gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2, gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m1, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m2, timeInterval));
    }

    @Test
    void testConstructor34() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad, mg, gg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, wrong, bgTriad,
                mg, gg, timeInterval, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma, bgTriad,
                wrong, gg, timeInterval, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m1,
                bgTriad, mg, gg, timeInterval, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, m2,
                bgTriad, mg, gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m1, gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, m2, gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m1, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, baTriad, ma,
                bgTriad, mg, m2, timeInterval, this));
    }

    @Test
    void testConstructor35() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg,
                timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong,
                timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg,
                timeInterval));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg,
                timeInterval));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg,
                timeInterval));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4,
                timeInterval));
    }

    @Test
    void testConstructor36() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg, timeInterval, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg,
                timeInterval, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong,
                timeInterval, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg,
                timeInterval, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg,
                timeInterval, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg,
                timeInterval, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4,
                timeInterval, this));
    }

    @Test
    void testConstructor37() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg, gg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg, gg,
                timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong, gg,
                timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg,
                gg, timeInterval));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg,
                gg, timeInterval));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg,
                gg, timeInterval));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m3, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m4, timeInterval));
    }

    @Test
    void testConstructor38() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg, gg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertEquals(ecefPosition1, ecefPosition2);

        assertEquals(ecefFrame1, estimator.getEcefFrame());
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertEquals(ecefC1, estimator.getEcefC());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertEquals(ecefC1, ecefC2);

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, wrong, bg, mg, gg,
                timeInterval, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, wrong, gg,
                timeInterval, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m1, ma, bg, mg,
                gg, timeInterval, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, m2, ma, bg, mg,
                gg, timeInterval, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m3, bg, mg,
                gg, timeInterval, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, m4, bg, mg,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m1, mg,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, m2, mg,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m3,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, m4,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m3, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(nedPosition, nedC, ba, ma, bg, mg,
                m4, timeInterval, this));
    }

    @Test
    void testConstructor39() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong, bgTriad,
                mg, timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad,
                wrong, timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m1,
                bgTriad, mg, timeInterval));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                bgTriad, mg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m1, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m2, timeInterval));
    }

    @Test
    void testConstructor40() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var nedPosition = createPosition();
            final var nedC = createOrientation();
            final var nedFrame = new NEDFrame(nedPosition, nedC);
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
            final var ecefPosition = ecefFrame.getECEFPosition();

            final var ba = generateBa();
            final var baTriad = new AccelerationTriad();
            baTriad.setValueCoordinates(ba);
            final var ma = generateMaGeneral();
            final var bg = generateBg();
            final var bgTriad = new AngularSpeedTriad();
            bgTriad.setValueCoordinates(bg);
            final var mg = generateMg();

            final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

            final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg, timeInterval,
                    this);

            // check default values
            assertSame(this, estimator.getListener());

            final var ba1 = estimator.getAccelerationBias();
            assertEquals(ba, ba1);
            final var ba2 = new Matrix(3, 1);
            estimator.getAccelerationBias(ba2);
            assertEquals(ba1, ba2);

            final var ba3 = estimator.getAccelerationBiasArray();
            assertArrayEquals(ba3, ba.getBuffer(), 0.0);
            final var ba4 = new double[3];
            estimator.getAccelerationBiasArray(ba4);
            assertArrayEquals(ba3, ba4, 0.0);

            final var triad1 = estimator.getAccelerationBiasAsTriad();
            assertEquals(baTriad, triad1);
            final var triad2 = new AccelerationTriad();
            estimator.getAccelerationBiasAsTriad(triad2);
            assertEquals(triad1, triad2);

            assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
            assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
            assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

            final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
            assertEquals(bax1, baTriad.getMeasurementX());
            final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAccelerationBiasXAsAcceleration(bax2);
            assertEquals(bax1, bax2);

            final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
            assertEquals(bay1, baTriad.getMeasurementY());
            final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAccelerationBiasYAsAcceleration(bay2);
            assertEquals(bay1, bay2);

            final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
            assertEquals(baz1, baTriad.getMeasurementZ());
            final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
            estimator.getAccelerationBiasZAsAcceleration(baz2);
            assertEquals(baz1, baz2);

            final var ma1 = estimator.getAccelerationCrossCouplingErrors();
            assertEquals(ma, ma1);

            final var ma2 = new Matrix(3, 3);
            estimator.getAccelerationCrossCouplingErrors(ma2);
            assertEquals(ma1, ma2);

            var sx = ma.getElementAt(0, 0);
            var sy = ma.getElementAt(1, 1);
            var sz = ma.getElementAt(2, 2);
            var mxy = ma.getElementAt(0, 1);
            var mxz = ma.getElementAt(0, 2);
            var myx = ma.getElementAt(1, 0);
            var myz = ma.getElementAt(1, 2);
            var mzx = ma.getElementAt(2, 0);
            var mzy = ma.getElementAt(2, 1);
            assertEquals(sx, estimator.getAccelerationSx(), 0.0);
            assertEquals(sy, estimator.getAccelerationSy(), 0.0);
            assertEquals(sz, estimator.getAccelerationSz(), 0.0);
            assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
            assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
            assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
            assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
            assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
            assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

            final var bg1 = estimator.getAngularSpeedBias();
            assertEquals(bg, bg1);
            final var bg2 = new Matrix(3, 1);
            estimator.getAngularSpeedBias(bg2);
            assertEquals(bg1, bg2);

            final var bg3 = estimator.getAngularSpeedBiasArray();
            assertArrayEquals(bg3, bg.getBuffer(), 0.0);
            final var bg4 = new double[3];
            estimator.getAngularSpeedBiasArray(bg4);
            assertArrayEquals(bg3, bg4, 0.0);

            final var triad3 = estimator.getAngularSpeedBiasAsTriad();
            assertEquals(bgTriad, triad3);
            final var triad4 = new AngularSpeedTriad();
            estimator.getAngularSpeedBiasAsTriad(triad4);
            assertEquals(triad3, triad4);

            assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
            assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
            assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

            final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
            assertEquals(bgx1, bgTriad.getMeasurementX());
            final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
            assertEquals(bgx1, bgx2);

            final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
            assertEquals(bgy1, bgTriad.getMeasurementY());
            final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
            assertEquals(bgy1, bgy2);

            final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
            assertEquals(bgz1, bgTriad.getMeasurementZ());
            final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
            estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
            assertEquals(bgz1, bgz2);

            final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
            assertEquals(mg, mg1);
            final var mg2 = new Matrix(3, 3);
            estimator.getAngularSpeedCrossCouplingErrors(mg2);
            assertEquals(mg1, mg2);

            sx = mg.getElementAt(0, 0);
            sy = mg.getElementAt(1, 1);
            sz = mg.getElementAt(2, 2);
            mxy = mg.getElementAt(0, 1);
            mxz = mg.getElementAt(0, 2);
            myx = mg.getElementAt(1, 0);
            myz = mg.getElementAt(1, 2);
            mzx = mg.getElementAt(2, 0);
            mzy = mg.getElementAt(2, 1);
            assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
            assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
            assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
            assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
            assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
            assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
            assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
            assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
            assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

            final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
            assertEquals(new Matrix(3, 3), gg1);
            final var gg2 = new Matrix(3, 3);
            estimator.getAngularSpeedGDependantCrossBias(gg2);
            assertEquals(gg1, gg2);

            assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

            final var t1 = estimator.getTimeIntervalAsTime();
            assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, t1.getUnit());

            final var t2 = new Time(1.0, TimeUnit.DAY);
            estimator.getTimeIntervalAsTime(t2);
            assertEquals(t1, t2);

            final var nedFrame1 = new NEDFrame(nedPosition, nedC);
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefC1 = ecefFrame1.getCoordinateTransformation();

            if (!ecefPosition1.equals(estimator.getEcefPosition(),
                LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
            final var ecefPosition2 = new ECEFPosition();
            estimator.getEcefPosition(ecefPosition2);
            assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

            assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
            final var ecefFrame2 = new ECEFFrame();
            estimator.getEcefFrame(ecefFrame2);
            assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

            assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
            final var nedFrame2 = new NEDFrame();
            estimator.getNedFrame(nedFrame2);
            assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

            assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
            final var nedPosition2 = new NEDPosition();
            estimator.getNedPosition(nedPosition2);
            assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
            final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
            estimator.getEcefC(ecefC2);
            assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

            assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
            final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
            estimator.getNedC(nedC2);
            assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

            assertEquals(0, estimator.getNumberOfProcessedSamples());
            assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
            assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
            final var elapsedTime1 = estimator.getElapsedTime();
            assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
            final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
            estimator.getElapsedTime(elapsedTime2);
            assertEquals(elapsedTime1, elapsedTime2);
            assertTrue(estimator.isFixKinematicsEnabled());
            assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
            assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                    estimator.getDriftPeriodSeconds(), 0.0);
            final var driftPeriod1 = estimator.getDriftPeriod();
            assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                    driftPeriod1.getValue().doubleValue(), 0.0);
            assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
            final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
            estimator.getDriftPeriod(driftPeriod2);
            assertEquals(driftPeriod1, driftPeriod2);
            assertFalse(estimator.isRunning());
            assertTrue(estimator.isReady());

            assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
            assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
            assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
            assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
            assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
            assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
            final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
            assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
            final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
            estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
            assertEquals(positionNoiseStd1, positionNoiseStd2);
            assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
            final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
            assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
            final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
            assertEquals(velocityNoiseStd1, velocityNoiseStd2);
            assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
            final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
            assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
            final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
            estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
            assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
            assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
            final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
            assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
            final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
            estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
            assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
            final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
            assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
            final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
            assertEquals(velocityUncertainty1, velocityUncertainty2);
            assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
            final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
            assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
            final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
            estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
            assertEquals(attitudeUncertainty1, attitudeUncertainty2);

            final var kinematics1 = estimator.getFixedKinematics();
            assertEquals(new BodyKinematics(), kinematics1);
            final var kinematics2 = new BodyKinematics();
            estimator.getFixedKinematics(kinematics2);
            assertEquals(kinematics1, kinematics2);

            // Force AlgebraException
            final var wrong = Matrix.identity(3, 3);
            wrong.multiplyByScalar(-1.0);
            assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong,
                    bgTriad, mg, timeInterval, this));
            assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad,
                    wrong, timeInterval, this));

            // Force IllegalArgumentException
            final var m1 = new Matrix(1, 3);
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m1,
                    bgTriad, mg, timeInterval, this));
            final var m2 = new Matrix(3, 1);
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                    bgTriad, mg, timeInterval, this));
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                    bgTriad, m1, timeInterval, this));
            assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                    bgTriad, m2, timeInterval, this));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testConstructor41() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong, bgTriad,
                mg, gg, timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad,
                wrong, gg, timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m1,
                bgTriad, mg, gg, timeInterval));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                bgTriad, mg, gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m1, gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m2, gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, mg, m1, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, mg, m2, timeInterval));
    }

    @Test
    void testConstructor42() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad, mg, gg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, wrong, bgTriad,
                mg, gg, timeInterval, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma, bgTriad,
                wrong, gg, timeInterval, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad,
                m1, bgTriad, mg, gg, timeInterval, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, m2,
                bgTriad, mg, gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m1, gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, m2, gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, mg, m1, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, baTriad, ma,
                bgTriad, mg, m2, timeInterval, this));
    }

    @Test
    void testConstructor43() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg,
                timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong,
                timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg,
                timeInterval));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg,
                timeInterval));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg,
                timeInterval));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3,
                timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m4,
                timeInterval));
    }

    @Test
    void testConstructor44() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg, timeInterval, this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax1, baTriad.getMeasurementX());
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay1, baTriad.getMeasurementY());
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg,
                this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong,
                this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg,
                timeInterval, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg,
                timeInterval, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg,
                timeInterval, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3,
                timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m4,
                timeInterval, this));
    }

    @Test
    void testConstructor45() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg, gg, timeInterval);

        // check default values
        assertNull(estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgTriad.getMeasurementX(), bgx1);
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg, gg,
                timeInterval));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong, gg,
                timeInterval));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg,
                gg, timeInterval));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg,
                gg, timeInterval));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg,
                gg, timeInterval));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m4,
                gg, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m3, timeInterval));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m4, timeInterval));
    }

    @Test
    void testConstructor46() throws AlgebraException, InvalidSourceAndDestinationFrameTypeException {
        final var nedPosition = createPosition();
        final var nedC = createOrientation();
        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var ba = generateBa();
        final var baTriad = new AccelerationTriad();
        baTriad.setValueCoordinates(ba);
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var bgTriad = new AngularSpeedTriad();
        bgTriad.setValueCoordinates(bg);
        final var mg = generateMg();
        final var gg = generateGg();

        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;

        final var estimator = new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg, gg, timeInterval,
                this);

        // check default values
        assertSame(this, estimator.getListener());

        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba1, ba2);

        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba3, ba.getBuffer(), 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba3, ba4, 0.0);

        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, triad1);
        final var triad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad2);
        assertEquals(triad1, triad2);

        assertEquals(baTriad.getValueX(), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(baTriad.getValueY(), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baTriad.getValueZ(), estimator.getAccelerationBiasZ(), 0.0);

        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(baTriad.getMeasurementX(), bax1);
        final var bax2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax2);
        assertEquals(bax1, bax2);

        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(baTriad.getMeasurementY(), bay1);
        final var bay2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay2);
        assertEquals(bay1, bay2);

        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz1, baTriad.getMeasurementZ());
        final var baz2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz2);
        assertEquals(baz1, baz2);

        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);

        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma1, ma2);

        var sx = ma.getElementAt(0, 0);
        var sy = ma.getElementAt(1, 1);
        var sz = ma.getElementAt(2, 2);
        var mxy = ma.getElementAt(0, 1);
        var mxz = ma.getElementAt(0, 2);
        var myx = ma.getElementAt(1, 0);
        var myz = ma.getElementAt(1, 2);
        var mzx = ma.getElementAt(2, 0);
        var mzy = ma.getElementAt(2, 1);
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg1, bg2);

        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg3, bg4, 0.0);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, triad3);
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad3, triad4);

        assertEquals(bgTriad.getValueX(), estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgTriad.getValueY(), estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgTriad.getValueZ(), estimator.getAngularSpeedBiasZ(), 0.0);

        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx1, bgTriad.getMeasurementX());
        final var bgx2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx2);
        assertEquals(bgx1, bgx2);

        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy1, bgTriad.getMeasurementY());
        final var bgy2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy2);
        assertEquals(bgy1, bgy2);

        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz1, bgTriad.getMeasurementZ());
        final var bgz2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz2);
        assertEquals(bgz1, bgz2);

        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg1, mg2);

        sx = mg.getElementAt(0, 0);
        sy = mg.getElementAt(1, 1);
        sz = mg.getElementAt(2, 2);
        mxy = mg.getElementAt(0, 1);
        mxz = mg.getElementAt(0, 2);
        myx = mg.getElementAt(1, 0);
        myz = mg.getElementAt(1, 2);
        mzx = mg.getElementAt(2, 0);
        mzy = mg.getElementAt(2, 1);
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);

        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg1, gg2);

        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);

        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(timeInterval, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        final var t2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t2);
        assertEquals(t1, t2);

        final var nedFrame1 = new NEDFrame(nedPosition, nedC);
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(ecefPosition1.equals(estimator.getEcefPosition(), LARGE_ABSOLUTE_ERROR));
        final var ecefPosition2 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition2);
        assertTrue(ecefPosition1.equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));

        assertTrue(ecefFrame1.equals(estimator.getEcefFrame(), LARGE_ABSOLUTE_ERROR));
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertTrue(ecefFrame1.equals(ecefFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedFrame1.equals(estimator.getNedFrame(), LARGE_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, LARGE_ABSOLUTE_ERROR));

        assertTrue(nedPosition.equals(estimator.getNedPosition(), LARGE_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        estimator.getNedPosition(nedPosition2);
        assertTrue(nedPosition.equals(nedPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(ecefC1.equals(estimator.getEcefC(), LARGE_ABSOLUTE_ERROR));
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC2);
        assertTrue(ecefC1.equals(ecefC2, ABSOLUTE_ERROR));

        assertTrue(nedC.equals(estimator.getNedC(), ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC2);
        assertTrue(nedC.equals(nedC2, ABSOLUTE_ERROR));

        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertEquals(0, estimator.getNumberOfProcessedDriftPeriods());
        assertEquals(0.0, estimator.getElapsedTimeSeconds(), 0.0);
        final var elapsedTime1 = estimator.getElapsedTime();
        assertEquals(0.0, elapsedTime1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, elapsedTime1.getUnit());
        final var elapsedTime2 = new Time(1.0, TimeUnit.DAY);
        estimator.getElapsedTime(elapsedTime2);
        assertEquals(elapsedTime1, elapsedTime2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES, estimator.getDriftPeriodSamples());
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                estimator.getDriftPeriodSeconds(), 0.0);
        final var driftPeriod1 = estimator.getDriftPeriod();
        assertEquals(RandomWalkEstimator.DEFAULT_DRIFT_PERIOD_SAMPLES * timeInterval,
                driftPeriod1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, driftPeriod1.getUnit());
        final var driftPeriod2 = new Time(1.0, TimeUnit.DAY);
        estimator.getDriftPeriod(driftPeriod2);
        assertEquals(driftPeriod1, driftPeriod2);
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isReady());

        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(0.0, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());
        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(0.0, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());
        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(0.0, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());
        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        final var positionUncertainty1 = estimator.getPositionUncertaintyAsDistance();
        assertEquals(0.0, positionUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionUncertainty1.getUnit());
        final var positionUncertainty2 = new Distance(1.0, DistanceUnit.MILE);
        estimator.getPositionUncertaintyAsDistance(positionUncertainty2);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        final var velocityUncertainty1 = estimator.getVelocityUncertaintyAsSpeed();
        assertEquals(0.0, velocityUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityUncertainty1.getUnit());
        final var velocityUncertainty2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityUncertaintyAsSpeed(velocityUncertainty2);
        assertEquals(velocityUncertainty1, velocityUncertainty2);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);
        final var attitudeUncertainty1 = estimator.getAttitudeUncertaintyAsAngle();
        assertEquals(0.0, attitudeUncertainty1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeUncertainty1.getUnit());
        final var attitudeUncertainty2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeUncertaintyAsAngle(attitudeUncertainty2);
        assertEquals(attitudeUncertainty1, attitudeUncertainty2);

        final var kinematics1 = estimator.getFixedKinematics();
        assertEquals(new BodyKinematics(), kinematics1);
        final var kinematics2 = new BodyKinematics();
        estimator.getFixedKinematics(kinematics2);
        assertEquals(kinematics1, kinematics2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3);
        wrong.multiplyByScalar(-1.0);
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, wrong, bg, mg, gg,
                timeInterval, this));
        assertThrows(AlgebraException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, wrong, gg,
                timeInterval, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m1, ma, bg, mg,
                gg, timeInterval, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, m2, ma, bg, mg,
                gg, timeInterval, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m3, bg, mg,
                gg, timeInterval, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, m4, bg, mg,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m1, mg,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, m2, mg,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m3,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, m4,
                gg, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m3, timeInterval, this));
        assertThrows(IllegalArgumentException.class, () -> new RandomWalkEstimator(ecefPosition, nedC, ba, ma, bg, mg,
                m4, timeInterval, this));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetAccelerationBias1() throws WrongSizeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var ba1 = estimator.getAccelerationBias();
        assertEquals(new Matrix(3, 1), ba1);

        // set a new value
        final var ba2 = generateBa();
        estimator.setAccelerationBias(ba2);

        // check
        final var ba3 = estimator.getAccelerationBias();
        final var ba4 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba4);
        assertEquals(ba2, ba3);
        assertEquals(ba2, ba4);

        // Force IllegalArgumentException
        final var m = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationBias(m));
    }

    @Test
    void testGetSetAccelerationBiasArray() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var ba1 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba1, 0.0);

        // set a new value
        final var ba2 = generateBa().getBuffer();
        estimator.setAccelerationBias(ba2);

        // check
        final var ba3 = estimator.getAccelerationBiasArray();
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba2, ba3, 0.0);
        assertArrayEquals(ba2, ba4, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.getAccelerationBiasArray(new double[1]));
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationBias(new double[1]));
    }

    @Test
    void testGetSetAccelerationBiasAsTriad() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        final var triad2 = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx, by, bz);
        estimator.setAccelerationBias(triad2);

        // check
        final var triad3 = estimator.getAccelerationBiasAsTriad();
        final var triad4 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(triad4);
        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetAccelerationBiasX() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        estimator.setAccelerationBiasX(bx);

        // check
        assertEquals(bx, estimator.getAccelerationBiasX(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasY() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var by = ba.getElementAtIndex(1);
        estimator.setAccelerationBiasY(by);

        // check
        assertEquals(by, estimator.getAccelerationBiasY(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasZ() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var bz = ba.getElementAtIndex(2);
        estimator.setAccelerationBiasZ(bz);

        // check
        assertEquals(bz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testSetAccelerationBias() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var by = ba.getElementAtIndex(1);
        final var bz = ba.getElementAtIndex(2);
        estimator.setAccelerationBias(bx, by, bz);

        // check
        assertEquals(bx, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(by, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(bz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasXAsAcceleration() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var bx1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bx1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bx1.getUnit());

        // set a new value
        final var ba = generateBa();
        final var bx = ba.getElementAtIndex(0);
        final var bx2 = new Acceleration(bx, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBiasX(bx2);

        // check
        final var bx3 = estimator.getAccelerationBiasXAsAcceleration();
        final var bx4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bx4);
        assertEquals(bx2, bx3);
        assertEquals(bx2, bx4);
    }

    @Test
    void testGetSetAccelerationBiasYAsAcceleration() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var by1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, by1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, by1.getUnit());

        // set a new value
        final var ba = generateBa();
        final var by = ba.getElementAtIndex(1);
        final var by2 = new Acceleration(by, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBiasY(by2);

        // check
        final var by3 = estimator.getAccelerationBiasYAsAcceleration();
        final var by4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(by4);
        assertEquals(by2, by3);
        assertEquals(by2, by4);
    }

    @Test
    void testGetSetAccelerationBiasZAsAcceleration() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var bz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, bz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bz1.getUnit());

        // set a new value
        final var ba = generateBa();
        final var bz = ba.getElementAtIndex(2);
        final var bz2 = new Acceleration(bz, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBiasZ(bz2);

        // check
        final var bz3 = estimator.getAccelerationBiasZAsAcceleration();
        final var bz4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(bz4);
        assertEquals(bz2, bz3);
        assertEquals(bz2, bz4);
    }

    @Test
    void testSetAccelerationBias2() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var bx = new Acceleration(ba.getElementAtIndex(0), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var by = new Acceleration(ba.getElementAtIndex(1), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bz = new Acceleration(ba.getElementAtIndex(2), AccelerationUnit.METERS_PER_SQUARED_SECOND);
        estimator.setAccelerationBias(bx, by, bz);

        // check
        assertEquals(ba.getElementAtIndex(0), estimator.getAccelerationBiasX(), 0.0);
        assertEquals(ba.getElementAtIndex(1), estimator.getAccelerationBiasY(), 0.0);
        assertEquals(ba.getElementAtIndex(2), estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerationCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), ma1);

        // set new values
        final var ma2 = generateMaGeneral();
        estimator.setAccelerationCrossCouplingErrors(ma2);

        // check
        final var ma3 = estimator.getAccelerationCrossCouplingErrors();
        final var ma4 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma4);
        assertEquals(ma2, ma3);
        assertEquals(ma2, ma4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationCrossCouplingErrors(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationCrossCouplingErrors(m2));
    }

    @Test
    void testGetSetAccelerationSx() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        estimator.setAccelerationSx(sx);

        // check
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
    }

    @Test
    void testGetSetAccelerationSy() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var sy = ma.getElementAt(1, 1);
        estimator.setAccelerationSy(sy);

        // check
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
    }

    @Test
    void testGetSetAccelerationSz() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var sz = ma.getElementAt(2, 2);
        estimator.setAccelerationSz(sz);

        // check
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
    }

    @Test
    void testGetSetAccelerationMxy() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var mxy = ma.getElementAt(0, 1);
        estimator.setAccelerationMxy(mxy);

        // check
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
    }

    @Test
    void testGetSetAccelerationMxz() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var mxz = ma.getElementAt(0, 2);
        estimator.setAccelerationMxz(mxz);

        // check
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
    }

    @Test
    void testGetSetAccelerationMyx() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var myx = ma.getElementAt(1, 0);
        estimator.setAccelerationMyx(myx);

        // check
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
    }

    @Test
    void testGetSetAccelerationMyz() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var myz = ma.getElementAt(1, 2);
        estimator.setAccelerationMyz(myz);

        // check
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
    }

    @Test
    void testGetSetAccelerationMzx() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var mzx = ma.getElementAt(2, 0);
        estimator.setAccelerationMzx(mzx);

        // check
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
    }

    @Test
    void testGetSetAccelerationMzy() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        // set a new value
        final var ma = generateMaGeneral();
        final var mzy = ma.getElementAt(2, 1);
        estimator.setAccelerationMzy(mzy);

        // check
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);
    }

    @Test
    void testSetAccelerationScalingFactors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);

        // set new values
        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        estimator.setAccelerationScalingFactors(sx, sy, sz);

        // check
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
    }

    @Test
    void testSetAccelerationCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        // set new values
        final var ma = generateMaGeneral();
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        estimator.setAccelerationCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);
    }

    @Test
    void testSetAccelerationScalingFactorsAndCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);

        // set new values
        final var ma = generateMaGeneral();
        final var sx = ma.getElementAt(0, 0);
        final var sy = ma.getElementAt(1, 1);
        final var sz = ma.getElementAt(2, 2);
        final var mxy = ma.getElementAt(0, 1);
        final var mxz = ma.getElementAt(0, 2);
        final var myx = ma.getElementAt(1, 0);
        final var myz = ma.getElementAt(1, 2);
        final var mzx = ma.getElementAt(2, 0);
        final var mzy = ma.getElementAt(2, 1);

        estimator.setAccelerationScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, estimator.getAccelerationSx(), 0.0);
        assertEquals(sy, estimator.getAccelerationSy(), 0.0);
        assertEquals(sz, estimator.getAccelerationSz(), 0.0);
        assertEquals(mxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(mxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(myx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(myz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(mzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(mzy, estimator.getAccelerationMzy(), 0.0);

        assertEquals(ma, estimator.getAccelerationCrossCouplingErrors());
    }

    @Test
    void testGetSetAngularSpeedBias1() throws WrongSizeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(new Matrix(3, 1), bg1);

        // set a new value
        final var bg2 = generateBg();
        estimator.setAngularSpeedBias(bg2);

        // check
        final var bg3 = estimator.getAngularSpeedBias();
        final var bg4 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg4);

        assertEquals(bg2, bg3);
        assertEquals(bg2, bg4);
    }

    @Test
    void testGetSetAngularSpeedBiasArray() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var bg1 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg1, 0.0);

        // set new values
        final var bg2 = generateBg().getBuffer();
        estimator.setAngularSpeedBias(bg2);

        // check
        final var bg3 = estimator.getAngularSpeedBiasArray();
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);

        assertArrayEquals(bg2, bg3, 0.0);
        assertArrayEquals(bg2, bg4, 0.0);
    }

    @Test
    void testGetSetAngularSpeedBiasAsTriad() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var triad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set new values
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var triad2 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);
        estimator.setAngularSpeedBias(triad2);

        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);

        // check
        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetAngularSpeedBiasX() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);

        // set a new value
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        estimator.setAngularSpeedBiasX(bgx);

        // check
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedBiasY() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);

        // set a new value
        final var bg = generateBg();
        final var bgy = bg.getElementAtIndex(1);
        estimator.setAngularSpeedBiasY(bgy);

        // check
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedBiasZ() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        // set a new value
        final var bg = generateBg();
        final var bgz = bg.getElementAtIndex(2);
        estimator.setAngularSpeedBiasZ(bgz);

        // check
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    void testSetAngularSpeedBias1() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        estimator.setAngularSpeedBias(bgx, bgy, bgz);

        // check
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedBiasXAsAngularSpeed() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var bgx1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgx1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgx1.getUnit());

        // set a new value
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgx2 = new AngularSpeed(bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBiasX(bgx2);

        // check
        final var bgx3 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        final var bgx4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgx4);

        assertEquals(bgx2, bgx3);
        assertEquals(bgx2, bgx4);
    }

    @Test
    void testGetSetAngularSpeedBiasYAsAngularSpeed() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var bgy1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgy1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgy1.getUnit());

        // set a new value
        final var bg = generateBg();
        final var bgy = bg.getElementAtIndex(1);
        final var bgy2 = new AngularSpeed(bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBiasY(bgy2);

        // check
        final var bgy3 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        final var bgy4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgy4);

        assertEquals(bgy2, bgy3);
        assertEquals(bgy2, bgy4);
    }

    @Test
    void testGetSetAngularSpeedBiasZAsAngularSpeed() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var bgz1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgz1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgz1.getUnit());

        // set a new value
        final var bg = generateBg();
        final var bgz = bg.getElementAtIndex(2);
        final var bgz2 = new AngularSpeed(bgz, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBiasZ(bgz2);

        // check
        final var bgz3 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        final var bgz4 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgz4);

        assertEquals(bgz2, bgz3);
        assertEquals(bgz2, bgz4);
    }

    @Test
    void testGetSetAngularSpeedBias2() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        // set a new value
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var bgx1 = new AngularSpeed(bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy1 = new AngularSpeed(bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz1 = new AngularSpeed(bgz, AngularSpeedUnit.RADIANS_PER_SECOND);
        estimator.setAngularSpeedBias(bgx1, bgy1, bgz1);

        // check
        final var bgx2 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        final var bgy2 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        final var bgz2 = estimator.getAngularSpeedBiasZAsAngularSpeed();

        assertEquals(bgx1, bgx2);
        assertEquals(bgy1, bgy2);
        assertEquals(bgz1, bgz2);
    }

    @Test
    void testGetSetAngularSpeedCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(new Matrix(3, 3), mg1);

        // set new values
        final var mg2 = generateMg();
        estimator.setAngularSpeedCrossCouplingErrors(mg2);

        // check
        final var mg3 = estimator.getAngularSpeedCrossCouplingErrors();
        final var mg4 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg4);
        assertEquals(mg2, mg3);
        assertEquals(mg2, mg4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedCrossCouplingErrors(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedCrossCouplingErrors(m2));
    }

    @Test
    void testGetSetAngularSpeedSx() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var sx = mg.getElementAt(0, 0);
        estimator.setAngularSpeedSx(sx);

        // check
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedSy() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var sy = mg.getElementAt(1, 1);
        estimator.setAngularSpeedSy(sy);

        // check
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedSz() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var sz = mg.getElementAt(2, 2);
        estimator.setAngularSpeedSz(sz);

        // check
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMxy() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var mxy = mg.getElementAt(0, 1);
        estimator.setAngularSpeedMxy(mxy);

        // check
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMxz() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var mxz = mg.getElementAt(0, 2);
        estimator.setAngularSpeedMxz(mxz);

        // check
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMyx() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var myx = mg.getElementAt(1, 0);
        estimator.setAngularSpeedMyx(myx);

        // check
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMyz() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var myz = mg.getElementAt(1, 2);
        estimator.setAngularSpeedMyz(myz);

        // check
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMzx() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var mzx = mg.getElementAt(2, 0);
        estimator.setAngularSpeedMzx(mzx);

        // check
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedMzy() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        // set a new value
        final var mg = generateMg();
        final var mzy = mg.getElementAt(2, 1);
        estimator.setAngularSpeedMzy(mzy);

        // check
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);
    }

    @Test
    void testSetAngularSpeedScalingFactors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);

        // set new values
        final var mg = generateMg();
        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        estimator.setAngularSpeedScalingFactors(sx, sy, sz);

        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
    }

    @Test
    void testSetAngularSpeedCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        // set new values
        final var mg = generateMg();
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);
        estimator.setAngularSpeedCrossCouplingErrors(mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);
    }

    @Test
    void testSetAngularSpeedScalingFactorAndCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);

        // set new values
        final var mg = generateMg();
        final var sx = mg.getElementAt(0, 0);
        final var sy = mg.getElementAt(1, 1);
        final var sz = mg.getElementAt(2, 2);
        final var mxy = mg.getElementAt(0, 1);
        final var mxz = mg.getElementAt(0, 2);
        final var myx = mg.getElementAt(1, 0);
        final var myz = mg.getElementAt(1, 2);
        final var mzx = mg.getElementAt(2, 0);
        final var mzy = mg.getElementAt(2, 1);
        estimator.setAngularSpeedScalingFactorsAndCrossCouplingErrors(sx, sy, sz, mxy, mxz, myx, myz, mzx, mzy);

        // check
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(mxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(mxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(myx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(myz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(mzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(mzy, estimator.getAngularSpeedMzy(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedGDependentCrossBias() throws WrongSizeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(new Matrix(3, 3), gg1);

        final var gg2 = generateGg();
        estimator.setAngularSpeedGDependantCrossBias(gg2);

        // check
        final var gg3 = estimator.getAngularSpeedGDependantCrossBias();
        final var gg4 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg4);
        assertEquals(gg2, gg3);
        assertEquals(gg2, gg4);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedGDependantCrossBias(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedGDependantCrossBias(m2));
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertEquals(TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        // set a new value
        final var timeInterval = 2.0 * TIME_INTERVAL_SECONDS;
        estimator.setTimeInterval(timeInterval);

        // check
        assertEquals(timeInterval, estimator.getTimeInterval(), 0.0);
    }

    @Test
    void testGetSetTimeIntervalAsTime() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var t1 = estimator.getTimeIntervalAsTime();
        assertEquals(TIME_INTERVAL_SECONDS, t1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, t1.getUnit());

        // set a new value
        final var t2 = new Time(2.0 * TIME_INTERVAL_SECONDS, TimeUnit.SECOND);
        estimator.setTimeInterval(t2);

        // check
        final var t3 = estimator.getTimeIntervalAsTime();
        final var t4 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(t4);
        assertEquals(t2, t3);
        assertEquals(t2, t4);
    }

    @Test
    void testGetSetEcefPosition() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(estimator.getEcefPosition(), ecefPosition1);

        // set a new value
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);
        estimator.setEcefPosition(ecefPosition2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    void testSetEcefPosition1() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(ecefPosition1, estimator.getEcefPosition());

        //  set a new value
        estimator.setEcefPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        // check
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    void testSetEcefPosition2() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(ecefPosition1, estimator.getEcefPosition());

        //  set a new value
        final var distance = new Distance(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, DistanceUnit.METER);
        estimator.setEcefPosition(distance, distance, distance);

        // check
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    void testSetEcefPosition3() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var ecefPosition1 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84, 0.0, 0.0);
        assertEquals(ecefPosition1, estimator.getEcefPosition());

        //  set a new value
        final var position = new InhomogeneousPoint3D(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);
        estimator.setEcefPosition(position);

        // check
        final var ecefPosition2 = new ECEFPosition(Constants.EARTH_EQUATORIAL_RADIUS_WGS84,
                Constants.EARTH_EQUATORIAL_RADIUS_WGS84, Constants.EARTH_EQUATORIAL_RADIUS_WGS84);

        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefPosition2, ecefPosition4);
    }

    @Test
    void testGetEcefFrame() {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        assertEquals(estimator.getEcefFrame(), ecefFrame1);
        final var ecefFrame2 = new ECEFFrame();
        estimator.getEcefFrame(ecefFrame2);
        assertEquals(ecefFrame1, ecefFrame2);
    }

    @Test
    void testGetNedFrame() {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var nedFrame1 = new NEDFrame();

        assertTrue(estimator.getNedFrame().equals(nedFrame1, ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        estimator.getNedFrame(nedFrame2);
        assertTrue(nedFrame1.equals(nedFrame2, ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetNedPosition() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var nedPosition1 = estimator.getNedPosition();

        assertEquals(0.0, nedPosition1.getLatitude(), ABSOLUTE_ERROR);
        assertEquals(0.0, nedPosition1.getLongitude(), ABSOLUTE_ERROR);
        assertEquals(0.0, nedPosition1.getHeight(), ABSOLUTE_ERROR);

        // set a new value
        final var nedPosition2 = createPosition();
        estimator.setNedPosition(nedPosition2);

        // check
        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    void testSetNedPosition1() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    void testSetNedPosition2() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition2 = new NEDPosition(latitude, longitude, heightDistance);

        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    void testSetNedPosition3() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);

        estimator.setNedPosition(latitude, longitude, height);

        // check
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var nedPosition3 = estimator.getNedPosition();
        final var nedPosition4 = new NEDPosition();
        estimator.getNedPosition(nedPosition4);

        assertTrue(nedPosition2.equals(nedPosition3, ABSOLUTE_ERROR));
        assertEquals(nedPosition3, nedPosition4);
    }

    @Test
    void testGetSetEcefC() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(estimator.getEcefC(), ecefC1);

        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME);
        estimator.setEcefC(ecefC2);

        // check
        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefC2, ecefC3);
        assertEquals(ecefC2, ecefC4);
    }

    @Test
    void testGetSetNedC() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set a new value
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC1 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        estimator.setNedC(nedC1);

        final var nedC2 = estimator.getNedC();
        final var nedC3 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getNedC(nedC3);

        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC3, ABSOLUTE_ERROR));
    }

    @Test
    void testSetNedPositionAndNedOrientation1() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var nedPosition1 = createPosition();
        final var nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(nedPosition1, nedC1);

        // check
        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(nedPosition1, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndNedOrientation2() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);

        final var nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, nedC1);

        // check
        final var nedPosition1 = new NEDPosition(latitude, longitude, height);

        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndNedOrientation3() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, height, nedC1);

        // check
        final var nedPosition1 = new NEDPosition(latitude, longitude, heightDistance);

        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndNedOrientation4() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        assertTrue(estimator.getNedPosition().equals(new NEDPosition(), ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(new CoordinateTransformation(FrameType.BODY_FRAME,
                        FrameType.LOCAL_NAVIGATION_FRAME), ABSOLUTE_ERROR));

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);

        final var nedC1 = createOrientation();

        estimator.setNedPositionAndNedOrientation(latitude, longitude, heightDistance, nedC1);

        // check
        final var nedPosition1 = new NEDPosition(latitude, longitude, heightDistance);

        final var nedPosition2 = estimator.getNedPosition();
        final var nedC2 = estimator.getNedC();

        assertTrue(nedPosition1.equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(nedC1.equals(nedC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndNedOrientation(latitude, longitude, heightDistance,
                        new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation1() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var nedPosition2 = createPosition();
        final var nedC2 = createOrientation();

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setEcefPositionAndEcefOrientation(ecefPosition2, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(ecefPosition1, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation2() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final var nedPosition2 = createPosition();
        final var nedC2 = createOrientation();

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();
        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(x, y, z, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation3() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final var nedPosition2 = createPosition();
        final var nedC2 = createOrientation();

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = new Distance(ecefPosition2.getX(), DistanceUnit.METER);
        final var y = new Distance(ecefPosition2.getY(), DistanceUnit.METER);
        final var z = new Distance(ecefPosition2.getZ(), DistanceUnit.METER);
        estimator.setEcefPositionAndEcefOrientation(x, y, z, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(x, y, z, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndEcefOrientation4() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(new NEDFrame());
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final var nedPosition2 = createPosition();
        final var nedC2 = createOrientation();

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();
        final var point = new InhomogeneousPoint3D(x, y, z);
        estimator.setEcefPositionAndEcefOrientation(point, ecefC2);

        // check
        final var ecefPosition3 = estimator.getEcefPosition();
        final var ecefPosition4 = new ECEFPosition();
        estimator.getEcefPosition(ecefPosition4);

        final var ecefC3 = estimator.getEcefC();
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getEcefC(ecefC4);

        assertEquals(ecefPosition2, ecefPosition3);
        assertEquals(ecefC2, ecefC3);

        assertEquals(ecefPosition2, ecefPosition4);
        assertEquals(ecefC2, ecefC4);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndEcefOrientation(point, new CoordinateTransformation(
                        FrameType.BODY_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation1() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final var nedPosition2 = createPosition();
        final var nedC2 = createOrientation();

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(nedPosition2, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition2);
        assertEquals(estimator.getEcefC(), ecefC2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(nedPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation2() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation3() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var heightDistance = new Distance(height, DistanceUnit.METER);
        final var nedPosition2 = new NEDPosition(latitude, longitude, heightDistance);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition2);
        assertEquals(estimator.getEcefC(), ecefC2);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetNedPositionAndEcefOrientation4() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        // set new values
        final var randomizer = new UniformRandomizer();
        final var latitude = new Angle(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var longitude = new Angle(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES),
                AngleUnit.DEGREES);
        final var height = new Distance(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT), DistanceUnit.METER);
        final var nedPosition2 = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC2 = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        estimator.setNedPositionAndEcefOrientation(latitude, longitude, height, ecefC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertEquals(ecefPosition2, estimator.getEcefPosition());
        assertEquals(ecefC2, estimator.getEcefC());

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setNedPositionAndEcefOrientation(latitude, longitude, height,
                        new CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndNedOrientation1() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // set new values
            final var nedPosition2 = createPosition();
            final var nedC2 = createOrientation();

            final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            final var ecefPosition2 = ecefFrame2.getECEFPosition();
            final var ecefC2 = ecefFrame2.getCoordinateTransformation();

            estimator.setEcefPositionAndNedOrientation(ecefPosition2, nedC2);

            // check
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2, 5.0 * ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2, 5.0 * ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndNedOrientation(ecefPosition1, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testSetEcefPositionAndNedOrientation2() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var estimator = new RandomWalkEstimator();

            // check default values
            final var nedFrame1 = new NEDFrame();
            final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

            final var nedPosition1 = nedFrame1.getPosition();
            final var nedC1 = nedFrame1.getCoordinateTransformation();
            final var ecefPosition1 = ecefFrame1.getECEFPosition();
            final var ecefC1 = ecefFrame1.getCoordinateTransformation();

            assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
            assertEquals(ecefPosition1, estimator.getEcefPosition());
            assertEquals(ecefC1, estimator.getEcefC());

            // set new values
            final var nedPosition2 = createPosition();
            final var nedC2 = createOrientation();

            final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            final var ecefPosition2 = ecefFrame2.getECEFPosition();
            final var ecefC2 = ecefFrame2.getCoordinateTransformation();

            final var x = ecefPosition2.getX();
            final var y = ecefPosition2.getY();
            final var z = ecefPosition2.getZ();

            estimator.setEcefPositionAndNedOrientation(x, y, z, nedC2);

            // check
            if (!estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            if (!estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));
            if (!estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> estimator.setEcefPositionAndNedOrientation(x, y, z, new CoordinateTransformation(
                            FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetEcefPositionAndNedOrientation3() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(ecefPosition1, estimator.getEcefPosition());
        assertEquals(ecefC1, estimator.getEcefC());

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            // set new values
            final var nedPosition2 = createPosition();
            final var nedC2 = createOrientation();

            final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
            final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

            final var ecefPosition2 = ecefFrame2.getECEFPosition();
            final var ecefC2 = ecefFrame2.getCoordinateTransformation();

            final var x = new Distance(ecefPosition2.getX(), DistanceUnit.METER);
            final var y = new Distance(ecefPosition2.getY(), DistanceUnit.METER);
            final var z = new Distance(ecefPosition2.getZ(), DistanceUnit.METER);

            estimator.setEcefPositionAndNedOrientation(x, y, z, nedC2);

            // check
            assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
            assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
            if (!estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR)) {
                continue;
            }
            assertTrue(estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));
            assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

            // Force InvalidSourceAndDestinationFrameTypeException
            assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                    () -> estimator.setEcefPositionAndNedOrientation(x, y, z, new CoordinateTransformation(
                            FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testSetEcefPositionAndNedOrientation4() throws InvalidSourceAndDestinationFrameTypeException, LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default values
        final var nedFrame1 = new NEDFrame();
        final var ecefFrame1 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame1);

        final var nedPosition1 = nedFrame1.getPosition();
        final var nedC1 = nedFrame1.getCoordinateTransformation();
        final var ecefPosition1 = ecefFrame1.getECEFPosition();
        final var ecefC1 = ecefFrame1.getCoordinateTransformation();

        assertTrue(estimator.getNedPosition().equals(nedPosition1, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC1, ABSOLUTE_ERROR));
        assertEquals(estimator.getEcefPosition(), ecefPosition1);
        assertEquals(estimator.getEcefC(), ecefC1);

        // set new values
        final var nedPosition2 = createPosition();
        final var nedC2 = createOrientation();

        final var nedFrame2 = new NEDFrame(nedPosition2, nedC2);
        final var ecefFrame2 = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame2);

        final var ecefPosition2 = ecefFrame2.getECEFPosition();
        final var ecefC2 = ecefFrame2.getCoordinateTransformation();

        final var x = ecefPosition2.getX();
        final var y = ecefPosition2.getY();
        final var z = ecefPosition2.getZ();
        final var point = new InhomogeneousPoint3D(x, y, z);

        estimator.setEcefPositionAndNedOrientation(point, nedC2);

        // check
        assertTrue(estimator.getNedPosition().equals(nedPosition2, ABSOLUTE_ERROR));
        assertTrue(estimator.getNedC().equals(nedC2, ABSOLUTE_ERROR));
        assertTrue(estimator.getEcefPosition().equals(ecefPosition2, LARGE_ABSOLUTE_ERROR));
        assertTrue(estimator.getEcefC().equals(ecefC2, ABSOLUTE_ERROR));

        // Force InvalidSourceAndDestinationFrameTypeException
        assertThrows(InvalidSourceAndDestinationFrameTypeException.class,
                () -> estimator.setEcefPositionAndNedOrientation(point, new CoordinateTransformation(
                        FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)));
    }

    @Test
    void testIsSetFixKinematicsEnabled() throws LockedException {
        final var estimator = new RandomWalkEstimator();

        // check default value
        assertTrue(estimator.isFixKinematicsEnabled());

        // set a new value
        estimator.setFixKinematicsEnabled(false);

        // check
        assertFalse(estimator.isFixKinematicsEnabled());
    }

    @Test
    void testAddBodyKinematicsWithFixEnabledAndReset() throws AlgebraException, LockedException,
            InvalidSourceAndDestinationFrameTypeException, RandomWalkEstimationException, NotReadyException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new RandomWalkEstimator();
        estimator.setNedPositionAndNedOrientation(nedPosition, nedC);
        estimator.setAccelerationBias(ba);
        estimator.setAccelerationCrossCouplingErrors(ma);
        estimator.setAngularSpeedBias(bg);
        estimator.setAngularSpeedCrossCouplingErrors(mg);
        estimator.setAngularSpeedGDependantCrossBias(gg);
        estimator.setTimeInterval(TIME_INTERVAL_SECONDS);
        estimator.setListener(this);

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertTrue(estimator.isReady());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertTrue(estimator.isFixKinematicsEnabled());

        // this is the true body kinematics that should be measured on a perfect
        // sensor if there were no noise or calibration errors at current position
        // and orientations
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefC,
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        final var measuredKinematics = new BodyKinematics();
        final var random = new Random();
        for (var i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            estimator.addBodyKinematics(measuredKinematics);

            assertEquals(estimator.getNumberOfProcessedSamples(), i + 1);
            assertFalse(estimator.isRunning());
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var accelerometerBiasPsd = estimator.getAccelerometerBiasPSD();
        assertTrue(accelerometerBiasPsd > 0.0);

        final var gyroBiasPsd = estimator.getGyroBiasPSD();
        assertTrue(gyroBiasPsd > 0.0);

        final var positionNoiseVariance = estimator.getPositionNoiseVariance();
        assertTrue(positionNoiseVariance > 0.0);

        final var velocityNoiseVariance = estimator.getVelocityNoiseVariance();
        assertTrue(velocityNoiseVariance > 0.0);

        final var attitudeNoiseVariance = estimator.getAttitudeNoiseVariance();
        assertTrue(attitudeNoiseVariance > 0.0);

        final var positionNoiseStandardDeviation = estimator.getPositionNoiseStandardDeviation();
        assertEquals(Math.sqrt(positionNoiseVariance), positionNoiseStandardDeviation, 0.0);

        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(positionNoiseStandardDeviation, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());

        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);

        final var velocityNoiseStandardDeviation = estimator.getVelocityNoiseStandardDeviation();
        assertEquals(Math.sqrt(velocityNoiseVariance), velocityNoiseStandardDeviation, 0.0);

        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(velocityNoiseStandardDeviation, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());

        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);

        final var attitudeNoiseStandardDeviation = estimator.getAttitudeNoiseStandardDeviation();
        assertEquals(Math.sqrt(attitudeNoiseVariance), attitudeNoiseStandardDeviation, 0.0);

        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(attitudeNoiseStandardDeviation, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());

        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);

        // check fixed kinematics
        final var fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var fixedKinematics1 = fixer.fixAndReturnNew(measuredKinematics);
        final var fixedKinematics2 = estimator.getFixedKinematics();
        final var fixedKinematics3 = new BodyKinematics();
        estimator.getFixedKinematics(fixedKinematics3);

        assertEquals(fixedKinematics1, fixedKinematics2);
        assertEquals(fixedKinematics1, fixedKinematics3);

        assertTrue(estimator.reset());
        assertEquals(1, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);

        assertFalse(estimator.reset());
        assertEquals(1, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
    }

    @Test
    void testAddBodyKinematicsWithFixDisabledAndReset() throws AlgebraException, LockedException,
            InvalidSourceAndDestinationFrameTypeException, RandomWalkEstimationException, NotReadyException {
        final var ba = generateBa();
        final var bg = generateBg();
        final var ma = generateMaCommonAxis();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        final var nedPosition = new NEDPosition(latitude, longitude, height);

        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var nedC = new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);

        final var nedFrame = new NEDFrame(nedPosition, nedC);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var estimator = new RandomWalkEstimator();
        estimator.setNedPositionAndNedOrientation(nedPosition, nedC);
        estimator.setAccelerationBias(ba);
        estimator.setAccelerationCrossCouplingErrors(ma);
        estimator.setAngularSpeedBias(bg);
        estimator.setAngularSpeedCrossCouplingErrors(mg);
        estimator.setAngularSpeedGDependantCrossBias(gg);
        estimator.setTimeInterval(TIME_INTERVAL_SECONDS);
        estimator.setListener(this);
        estimator.setFixKinematicsEnabled(false);

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertTrue(estimator.isReady());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertFalse(estimator.isFixKinematicsEnabled());

        // this is the true body kinematics that should be measured on a perfect
        // sensor if there were no noise or calibration errors at current position
        // and orientations
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefC,
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        BodyKinematicsFixer fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var measuredKinematics = new BodyKinematics();
        // fixed kinematics where bias and cross-couplings are compensated but
        // that still contains noise
        final var fixedKinematics1 = new BodyKinematics();
        final var random = new Random();
        for (var i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);
            fixer.fix(measuredKinematics, fixedKinematics1);
            estimator.addBodyKinematics(fixedKinematics1);

            assertEquals(i + 1, estimator.getNumberOfProcessedSamples());
            assertFalse(estimator.isRunning());
        }

        assertEquals(N_SAMPLES, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var accelerometerBiasPsd = estimator.getAccelerometerBiasPSD();
        assertTrue(accelerometerBiasPsd > 0.0);

        final var gyroBiasPsd = estimator.getGyroBiasPSD();
        assertTrue(gyroBiasPsd > 0.0);

        final var positionNoiseVariance = estimator.getPositionNoiseVariance();
        assertTrue(positionNoiseVariance > 0.0);

        final var velocityNoiseVariance = estimator.getVelocityNoiseVariance();
        assertTrue(velocityNoiseVariance > 0.0);

        final var attitudeNoiseVariance = estimator.getAttitudeNoiseVariance();
        assertTrue(attitudeNoiseVariance > 0.0);

        final var positionNoiseStandardDeviation = estimator.getPositionNoiseStandardDeviation();
        assertEquals(Math.sqrt(positionNoiseVariance), positionNoiseStandardDeviation, 0.0);

        final var positionNoiseStd1 = estimator.getPositionNoiseStandardDeviationAsDistance();
        assertEquals(positionNoiseStandardDeviation, positionNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, positionNoiseStd1.getUnit());

        final var positionNoiseStd2 = new Distance(1.0, DistanceUnit.INCH);
        estimator.getPositionNoiseStandardDeviationAsDistance(positionNoiseStd2);
        assertEquals(positionNoiseStd1, positionNoiseStd2);

        final var velocityNoiseStandardDeviation = estimator.getVelocityNoiseStandardDeviation();
        assertEquals(Math.sqrt(velocityNoiseVariance), velocityNoiseStandardDeviation, 0.0);

        final var velocityNoiseStd1 = estimator.getVelocityNoiseStandardDeviationAsSpeed();
        assertEquals(velocityNoiseStandardDeviation, velocityNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, velocityNoiseStd1.getUnit());

        final var velocityNoiseStd2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        estimator.getVelocityNoiseStandardDeviationAsSpeed(velocityNoiseStd2);
        assertEquals(velocityNoiseStd1, velocityNoiseStd2);

        final var attitudeNoiseStandardDeviation = estimator.getAttitudeNoiseStandardDeviation();
        assertEquals(Math.sqrt(attitudeNoiseVariance), attitudeNoiseStandardDeviation, 0.0);

        final var attitudeNoiseStd1 = estimator.getAttitudeNoiseStandardDeviationAsAngle();
        assertEquals(attitudeNoiseStandardDeviation, attitudeNoiseStd1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, attitudeNoiseStd1.getUnit());

        final var attitudeNoiseStd2 = new Angle(1.0, AngleUnit.DEGREES);
        estimator.getAttitudeNoiseStandardDeviationAsAngle(attitudeNoiseStd2);
        assertEquals(attitudeNoiseStd1, attitudeNoiseStd2);

        // check fixed kinematics
        final var fixedKinematics2 = estimator.getFixedKinematics();
        final var fixedKinematics3 = new BodyKinematics();
        estimator.getFixedKinematics(fixedKinematics3);

        assertEquals(fixedKinematics1, fixedKinematics2);
        assertEquals(fixedKinematics1, fixedKinematics3);

        assertTrue(estimator.reset());
        assertEquals(1, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertFalse(estimator.isRunning());
        assertEquals(0.0, estimator.getAccelerometerBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getGyroBiasPSD(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseVariance(), 0.0);
        assertEquals(0.0, estimator.getPositionNoiseStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getVelocityNoiseStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getAttitudeNoiseStandardDeviation(), 0.0);
        assertEquals(0.0, estimator.getPositionUncertainty(), 0.0);
        assertEquals(0.0, estimator.getVelocityUncertainty(), 0.0);
        assertEquals(0.0, estimator.getAttitudeUncertainty(), 0.0);

        assertFalse(estimator.reset());
        assertEquals(1, reset);
        assertEquals(0, estimator.getNumberOfProcessedSamples());
    }

    @Override
    public void onStart(final RandomWalkEstimator estimator) {
        checkLocked(estimator);
        start++;
    }

    @Override
    public void onBodyKinematicsAdded(
            final RandomWalkEstimator estimator, final BodyKinematics measuredKinematics,
            final BodyKinematics fixedKinematics) {
        if (bodyKinematicsAdded == 0) {
            checkLocked(estimator);
        }
        bodyKinematicsAdded++;
    }

    @Override
    public void onReset(final RandomWalkEstimator estimator) {
        checkLocked(estimator);
        reset++;
    }

    private static void checkLocked(RandomWalkEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBias((Matrix) null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBias((double[]) null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBias((AccelerationTriad) null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBiasX(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBiasY(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBiasZ(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBias(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBiasX(null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBiasY(null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBiasZ(null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationBias(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationCrossCouplingErrors(null));
        assertThrows(LockedException.class, () -> estimator.setAccelerationSx(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationSy(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationMxy(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationMxz(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationMyx(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationMyz(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationMzx(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationMzy(0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationScalingFactors(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAccelerationScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBias((Matrix) null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBias((double[]) null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBias((AngularSpeedTriad) null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBiasX(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBiasY(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBiasZ(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBias(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBiasX(null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBiasY(null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBiasZ(null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedBias(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedCrossCouplingErrors(null));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedSx(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedSy(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedSz(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedMxy(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedMxz(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedMyx(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedMyz(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedMzx(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedMzy(0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedScalingFactors(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedScalingFactorsAndCrossCouplingErrors(
                0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setAngularSpeedGDependantCrossBias(null));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.01));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(null));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition((ECEFPosition) null));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPosition((Point3D) null));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(null));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(0.0, 0.0, 0.0));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(null, null, 0.0));
        assertThrows(LockedException.class, () -> estimator.setNedPosition(null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefC(null));
        assertThrows(LockedException.class, () -> estimator.setNedC(null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                null, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(null, null,
                0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndNedOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation((ECEFPosition) null,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(0.0, 0.0, 0.0,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndEcefOrientation((Point3D) null,
                null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(null,
                null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(
                null, null, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setNedPositionAndEcefOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation((ECEFPosition) null,
                null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(
                0.0, 0.0, 0.0, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation(
                null, null, null, null));
        assertThrows(LockedException.class, () -> estimator.setEcefPositionAndNedOrientation((Point3D) null,
                null));
        assertThrows(LockedException.class, () -> estimator.setFixKinematicsEnabled(true));
        assertThrows(LockedException.class, () -> estimator.setDriftPeriodSamples(1));
        assertThrows(LockedException.class, () -> estimator.addBodyKinematics(null));
        assertThrows(LockedException.class, estimator::reset);
    }

    private void reset() {
        start = 0;
        bodyKinematicsAdded = 0;
        reset = 0;
    }

    private static Matrix generateBa() {
        return Matrix.newFromArray(new double[]{
                900 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                -1300 * MICRO_G_TO_METERS_PER_SECOND_SQUARED,
                800 * MICRO_G_TO_METERS_PER_SECOND_SQUARED});
    }

    private static Matrix generateBg() {
        return Matrix.newFromArray(new double[]{
                -9 * DEG_TO_RAD / 3600.0,
                13 * DEG_TO_RAD / 3600.0,
                -8 * DEG_TO_RAD / 3600.0});
    }

    private static Matrix generateMaGeneral() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                -150e-6, -600e-6, 250e-6,
                -250e-6, 100e-6, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMaCommonAxis() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);

        return result;
    }

    private static Matrix generateMg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                400e-6, -300e-6, 250e-6,
                0.0, -300e-6, -150e-6,
                0.0, 0.0, -350e-6
        }, false);

        return result;
    }

    private static Matrix generateGg() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        final var tmp = DEG_TO_RAD / (3600 * 9.80665);
        result.fromArray(new double[]{
                0.9 * tmp, -1.1 * tmp, -0.6 * tmp,
                -0.5 * tmp, 1.9 * tmp, -1.6 * tmp,
                0.3 * tmp, 1.1 * tmp, -1.3 * tmp
        }, false);

        return result;
    }

    private static double getAccelNoiseRootPSD() {
        return 100.0 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
    }

    private static double getGyroNoiseRootPSD() {
        return 0.01 * DEG_TO_RAD / 60.0;
    }

    private static NEDPosition createPosition() {
        final var randomizer = new UniformRandomizer();
        final var latitude = Math.toRadians(randomizer.nextDouble(MIN_LATITUDE_DEGREES, MAX_LATITUDE_DEGREES));
        final var longitude = Math.toRadians(randomizer.nextDouble(MIN_LONGITUDE_DEGREES, MAX_LONGITUDE_DEGREES));
        final var height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT);
        return new NEDPosition(latitude, longitude, height);
    }

    private static CoordinateTransformation createOrientation() {
        final var randomizer = new UniformRandomizer();
        final var roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        final var yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES));
        return new CoordinateTransformation(roll, pitch, yaw, FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME);
    }
}
