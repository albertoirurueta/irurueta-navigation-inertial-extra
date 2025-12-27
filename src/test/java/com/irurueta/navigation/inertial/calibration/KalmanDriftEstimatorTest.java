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
import com.irurueta.geometry.InvalidRotationMatrixException;
import com.irurueta.geometry.Quaternion;
import com.irurueta.geometry.RotationException;
import com.irurueta.navigation.LockedException;
import com.irurueta.navigation.NotReadyException;
import com.irurueta.navigation.frames.*;
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter;
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter;
import com.irurueta.navigation.inertial.*;
import com.irurueta.navigation.inertial.calibration.accelerometer.KnownGravityNormAccelerometerCalibrator;
import com.irurueta.navigation.inertial.calibration.generators.AccelerometerAndGyroscopeMeasurementsGenerator;
import com.irurueta.navigation.inertial.calibration.gyroscope.EasyGyroscopeCalibrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionIntegrator;
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType;
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer;
import com.irurueta.navigation.inertial.calibration.intervals.thresholdfactor.IntervalDetectorThresholdFactorOptimizerException;
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator;
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator;
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator;
import com.irurueta.navigation.inertial.navigators.InertialNavigatorException;
import com.irurueta.statistics.UniformRandomizer;
import com.irurueta.units.*;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.logging.Level;
import java.util.logging.Logger;

import static org.junit.jupiter.api.Assertions.*;

class KalmanDriftEstimatorTest implements DriftEstimatorListener {

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

    // the number of samples is equivalent to 30 seconds for the default time interval
    private static final int N_SAMPLES = 1500;

    private static final double FRAME_ABSOLUTE_ERROR = 1e-8;

    private static final double ABSOLUTE_ERROR = 1e-12;

    private static final double MIN_DELTA_POS_METERS = -1e-3;
    private static final double MAX_DELTA_POS_METERS = 1e-3;
    private static final double MIN_DELTA_ANGLE_DEGREES = -2.0;
    private static final double MAX_DELTA_ANGLE_DEGREES = 2.0;

    private static final int TIMES = 100;

    private static final Logger LOGGER = Logger.getLogger(KalmanDriftEstimatorTest.class.getName());

    private int start;
    private int bodyKinematicsAdded;
    private int reset;

    private final List<TimedBodyKinematics> timedBodyKinematics = new ArrayList<>();

    private final AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource dataSource =
            new AccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizerDataSource() {

                @Override
                public int count() {
                    return timedBodyKinematics.size();
                }

                @Override
                public TimedBodyKinematics getAt(final int index) {
                    return timedBodyKinematics.get(index);
                }
            };

    @Test
    void testConstructor1() throws WrongSizeException {
        final var estimator = new KalmanDriftEstimator();

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertNull(estimator.getKalmanConfig());
        assertNull(estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor2() throws WrongSizeException {
        final var estimator = new KalmanDriftEstimator(this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertNull(estimator.getKalmanConfig());
        assertNull(estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor3() throws WrongSizeException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();
        final var estimator = new KalmanDriftEstimator(kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor4() throws WrongSizeException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var estimator = new KalmanDriftEstimator(kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor5() throws WrongSizeException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var estimator = new KalmanDriftEstimator(ecefFrame, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertSame(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor6() throws WrongSizeException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var estimator = new KalmanDriftEstimator(ecefFrame, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertSame(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor7() throws WrongSizeException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var estimator = new KalmanDriftEstimator(nedFrame, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor8() throws WrongSizeException {
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var estimator = new KalmanDriftEstimator(nedFrame, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(new Matrix(3, 1), ba1);
        assertArrayEquals(new double[3], estimator.getAccelerationBiasArray(), 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(new double[3], ba2, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, baTriad1.getValueX(), 0.0);
        assertEquals(0.0, baTriad1.getValueY(), 0.0);
        assertEquals(0.0, baTriad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baTriad1.getUnit());
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad1, baTriad2);
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());
        final var ma = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma);
        assertEquals(new Matrix(3, 3), ma);
        assertEquals(0.0, estimator.getAccelerationSx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationSz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxy(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMxz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMyz(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzx(), 0.0);
        assertEquals(0.0, estimator.getAccelerationMzy(), 0.0);
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());
        final var bg1 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg1);
        assertEquals(new Matrix(3, 1), bg1);
        assertArrayEquals(new double[3], estimator.getAngularSpeedBiasArray(), 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(new double[3], bg2, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, bgTriad1.getValueX(), 0.0);
        assertEquals(0.0, bgTriad1.getValueY(), 0.0);
        assertEquals(0.0, bgTriad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgTriad1.getUnit());
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad1, bgTriad2);
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(0.0, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(0.0, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(0.0, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());
        final var mg = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg);
        assertEquals(new Matrix(3, 3), mg);
        assertEquals(0.0, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));
    }

    @Test
    void testConstructor9() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg, kalmanConfig,
                initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong, kalmanConfig,
                initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m1, bgTriad, mg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m2, bgTriad, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m1,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m2,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor10() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg, kalmanConfig,
                initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong, kalmanConfig,
                initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m1, bgTriad, mg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m2, bgTriad, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m1,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m2,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor11() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong, gg,
                kalmanConfig, initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m1, bgTriad, mg, gg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m2, bgTriad, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m1, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m2, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, m1,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, m2,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor12() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, wrong, bgTriad, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, wrong, gg,
                kalmanConfig, initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m1, bgTriad, mg, gg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, m2, bgTriad, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m1, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, m2, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, m1,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(baTriad, ma, bgTriad, mg, m2,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor13() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ba, ma, bg, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, wrong, bg, mg, kalmanConfig,
                initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, ma, bg, wrong, kalmanConfig,
                initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m1, ma, bg, mg, kalmanConfig,
                initConfig));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m2, ma, bg, mg, kalmanConfig,
                initConfig));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m3, bg, mg, kalmanConfig,
                initConfig));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m4, bg, mg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m1, mg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m2, mg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m3, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m4, kalmanConfig,
                initConfig));
    }

    @Test
    void testConstructor14() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ba, ma, bg, mg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, wrong, bg, mg, kalmanConfig, initConfig,
                this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, ma, bg, wrong, kalmanConfig, initConfig,
                this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m1, ma, bg, mg, kalmanConfig,
                initConfig, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m2, ma, bg, mg, kalmanConfig,
                initConfig, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m3, bg, mg, kalmanConfig,
                initConfig, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m4, bg, mg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m1, mg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m2, mg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m3, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m4, kalmanConfig,
                initConfig, this));
    }

    @Test
    void testConstructor15() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ba, ma, bg, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, wrong, bg, mg, gg, kalmanConfig,
                initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, ma, bg, wrong, gg, kalmanConfig,
                initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m1, ma, bg, mg, gg, kalmanConfig,
                initConfig));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m2, ma, bg, mg, gg, kalmanConfig,
                initConfig));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m3, bg, mg, gg, kalmanConfig,
                initConfig));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m4, bg, mg, gg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m1, mg, gg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m2, mg, gg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m3, gg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m4, gg, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, mg, m3, kalmanConfig,
                initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, mg, m4, kalmanConfig,
                initConfig));
    }

    @Test
    void testConstructor16() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ba, ma, bg, mg, gg, kalmanConfig, initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg.getBuffer(), bg4, 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertFalse(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, wrong, bg, mg, gg, kalmanConfig,
                initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ba, ma, bg, wrong, gg, kalmanConfig,
                initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m1, ma, bg, mg, gg, kalmanConfig,
                initConfig, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(m2, ma, bg, mg, gg, kalmanConfig,
                initConfig, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m3, bg, mg, gg, kalmanConfig,
                initConfig, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, m4, bg, mg, gg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m1, mg, gg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, m2, mg, gg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m3, gg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, m4, gg, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, mg, m3, kalmanConfig,
                initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ba, ma, bg, mg, m4, kalmanConfig,
                initConfig, this));
    }

    @Test
    void testConstructor17() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefVelocity1, ecefFrame.getECEFVelocity());
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, wrong, bgTriad, mg,
                kalmanConfig, initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, wrong,
                kalmanConfig, initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m1, bgTriad, mg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m2, bgTriad, mg,
                kalmanConfig, initConfig));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m3,
                kalmanConfig, initConfig));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m4,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor18() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, wrong, bgTriad, mg,
                kalmanConfig, initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, wrong,
                kalmanConfig, initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m1, bgTriad, mg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m2, bgTriad, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m1,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m2,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor19() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg, gg, kalmanConfig,
                initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba.getBuffer(), ba2, 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, wrong, bgTriad, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, wrong, gg,
                kalmanConfig, initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m1, bgTriad, mg,
                gg, kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m2, bgTriad, mg,
                gg, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m1,
                gg, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m2,
                gg, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg,
                m1, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg,
                m2, kalmanConfig, initConfig));
    }

    @Test
    void testConstructor20() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg, gg, kalmanConfig,
                initConfig, this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, wrong, bgTriad, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, wrong, gg,
                kalmanConfig, initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m1, bgTriad, mg,
                gg, kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, m2, bgTriad, mg,
                gg, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m1,
                gg, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, m2,
                gg, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg,
                m1, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, baTriad, ma, bgTriad, mg,
                m2, kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor21() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg.getBuffer(), bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg, kalmanConfig,
                initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong, kalmanConfig,
                initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m1, ma, bg, mg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m2, ma, bg, mg,
                kalmanConfig, initConfig));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m3, bg, mg,
                kalmanConfig, initConfig));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m4, bg, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m1, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m2, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m3,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m4,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor22() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg, kalmanConfig,
                initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong, kalmanConfig,
                initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m1, ma, bg, mg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m2, ma, bg, mg,
                kalmanConfig, initConfig, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m3, bg, mg,
                kalmanConfig, initConfig, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m4, bg, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m1, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m2, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m3,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m4,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor23() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefVelocity1, ecefFrame.getECEFVelocity());
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong, gg,
                kalmanConfig, initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m1, ma, bg, mg, gg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m2, ma, bg, mg, gg,
                kalmanConfig, initConfig));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m3, bg, mg, gg,
                kalmanConfig, initConfig));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m4, bg, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m1, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m2, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m3, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m4, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, m3,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, m4,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor24() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, wrong, bg, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, wrong, gg,
                kalmanConfig, initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m1, ma, bg, mg, gg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, m2, ma, bg, mg, gg,
                kalmanConfig, initConfig, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m3, bg, mg, gg,
                kalmanConfig, initConfig, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, m4, bg, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m1, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, m2, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m3, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, m4, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, m3,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(ecefFrame, ba, ma, bg, mg, m4,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor25() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        KalmanDriftEstimator estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg, kalmanConfig,
                initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, wrong, bgTriad, mg,
                kalmanConfig, initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, wrong,
                kalmanConfig, initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m1, bgTriad, mg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m2, bgTriad, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m1,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m2,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor26() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefVelocity1, ecefFrame.getECEFVelocity());
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, wrong, bgTriad, mg,
                kalmanConfig, initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, wrong,
                kalmanConfig, initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m1, bgTriad, mg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m2, bgTriad, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m1,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m2,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor27() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg, gg, kalmanConfig,
                initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, wrong, bgTriad, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, wrong, gg,
                kalmanConfig, initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m1, bgTriad, mg,
                gg, kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m2, bgTriad, mg,
                gg, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m1,
                gg, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m2,
                gg, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg,
                m1, kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg,
                m2, kalmanConfig, initConfig));
    }

    @Test
    void testConstructor28() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg, gg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, wrong, bgTriad, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, wrong, gg,
                kalmanConfig, initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m1, bgTriad, mg,
                gg, kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, m2, bgTriad, mg,
                gg, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m1,
                gg, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, m2,
                gg, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg,
                m1, kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, baTriad, ma, bgTriad, mg,
                m2, kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor29() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefFrame.getCoordinateTransformation(), ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg, kalmanConfig,
                initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong, kalmanConfig,
                initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m1, ma, bg, mg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m2, ma, bg, mg,
                kalmanConfig, initConfig));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m3, bg, mg,
                kalmanConfig, initConfig));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m4, bg, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m1, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m2, mg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m3,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m4,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor30() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefVelocity1, ecefFrame.getECEFVelocity());
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());
        final var gg = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg);
        assertEquals(new Matrix(3, 3), gg);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg, kalmanConfig,
                initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong, kalmanConfig,
                initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m1, ma, bg, mg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m2, ma, bg, mg,
                kalmanConfig, initConfig, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m3, bg, mg,
                kalmanConfig, initConfig, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m4, bg, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m1, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m2, mg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m3,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m4,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testConstructor31() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig);

        // check default values
        assertNull(estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefFrame.getECEFVelocity(), ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg, gg, 
                kalmanConfig, initConfig));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong, gg,
                kalmanConfig, initConfig));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m1, ma, bg, mg, gg,
                kalmanConfig, initConfig));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m2, ma, bg, mg, gg,
                kalmanConfig, initConfig));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m3, bg, mg, gg,
                kalmanConfig, initConfig));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m4, bg, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m1, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m2, mg, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m3, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m4, gg,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, m3,
                kalmanConfig, initConfig));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, m4,
                kalmanConfig, initConfig));
    }

    @Test
    void testConstructor32() throws AlgebraException {
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();

        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ba = generateBa();
        final var ma = generateMaGeneral();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var asx = ma.getElementAt(0, 0);
        final var asy = ma.getElementAt(1, 1);
        final var asz = ma.getElementAt(2, 2);
        final var amxy = ma.getElementAt(0, 1);
        final var amxz = ma.getElementAt(0, 2);
        final var amyx = ma.getElementAt(1, 0);
        final var amyz = ma.getElementAt(1, 2);
        final var amzx = ma.getElementAt(2, 0);
        final var amzy = ma.getElementAt(2, 1);

        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var gsx = mg.getElementAt(0, 0);
        final var gsy = mg.getElementAt(1, 1);
        final var gsz = mg.getElementAt(2, 2);
        final var gmxy = mg.getElementAt(0, 1);
        final var gmxz = mg.getElementAt(0, 2);
        final var gmyx = mg.getElementAt(1, 0);
        final var gmyz = mg.getElementAt(1, 2);
        final var gmzx = mg.getElementAt(2, 0);
        final var gmzy = mg.getElementAt(2, 1);

        final var baTriad = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);

        final var bgTriad = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);

        final var estimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, gg, kalmanConfig, initConfig,
                this);

        // check default values
        assertSame(this, estimator.getListener());
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefFrame.getECEFPosition(), ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        assertEquals(ecefPosition1, ecefPosition2);
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefVelocity1, ecefFrame.getECEFVelocity());
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity1, ecefVelocity2);
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC1, ecefFrame.getCoordinateTransformation());
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceEcefCoordinateTransformation(ecefC2));
        assertEquals(ecefC1, ecefC2);
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition1.equals(nedFrame.getPosition(), FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity1.equals(nedFrame.getVelocity(), FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC1.equals(nedFrame.getCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ba, estimator.getAccelerationBias());
        final var ba1 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba1);
        assertEquals(ba, ba1);
        final var ba2 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba2, ba.getBuffer(), 0.0);
        final var ba3 = new double[3];
        estimator.getAccelerationBiasArray(ba3);
        assertArrayEquals(ba2, ba3, 0.0);
        final var baTriad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(baTriad, baTriad1);
        final var baTriad2 = new AccelerationTriad();
        estimator.getAccelerationBiasAsTriad(baTriad2);
        assertEquals(baTriad, baTriad2);
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
        final var baX1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(bax, baX1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baX1.getUnit());
        final var baX2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(baX2);
        assertEquals(baX1, baX2);
        final var baY1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(bay, baY1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baY1.getUnit());
        final var baY2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(baY2);
        assertEquals(baY1, baY2);
        final var baZ1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(baz, baZ1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baZ1.getUnit());
        final var baZ2 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baZ2);
        assertEquals(baZ1, baZ2);
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        assertEquals(ma, ma1);
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma2);
        assertEquals(asx, estimator.getAccelerationSx(), 0.0);
        assertEquals(asy, estimator.getAccelerationSy(), 0.0);
        assertEquals(asz, estimator.getAccelerationSz(), 0.0);
        assertEquals(amxy, estimator.getAccelerationMxy(), 0.0);
        assertEquals(amxz, estimator.getAccelerationMxz(), 0.0);
        assertEquals(amyx, estimator.getAccelerationMyx(), 0.0);
        assertEquals(amyz, estimator.getAccelerationMyz(), 0.0);
        assertEquals(amzx, estimator.getAccelerationMzx(), 0.0);
        assertEquals(amzy, estimator.getAccelerationMzy(), 0.0);
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg3, bg.getBuffer(), 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg4, bg.getBuffer(), 0.0);
        final var bgTriad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(bgTriad, bgTriad1);
        final var bgTriad2 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(bgTriad2);
        assertEquals(bgTriad, bgTriad2);
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
        final var bgX1 = estimator.getAngularSpeedBiasXAsAngularSpeed();
        assertEquals(bgx, bgX1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgX1.getUnit());
        final var bgX2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasXAsAngularSpeed(bgX2);
        assertEquals(bgX1, bgX2);
        final var bgY1 = estimator.getAngularSpeedBiasYAsAngularSpeed();
        assertEquals(bgy, bgY1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgY1.getUnit());
        final var bgY2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasYAsAngularSpeed(bgY2);
        assertEquals(bgY1, bgY2);
        final var bgZ1 = estimator.getAngularSpeedBiasZAsAngularSpeed();
        assertEquals(bgz, bgZ1.getValue().doubleValue(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, bgZ1.getUnit());
        final var bgZ2 = new AngularSpeed(1.0, AngularSpeedUnit.DEGREES_PER_SECOND);
        estimator.getAngularSpeedBiasZAsAngularSpeed(bgZ2);
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        assertEquals(mg, mg1);
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg2);
        assertEquals(gsx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(gsy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(gsz, estimator.getAngularSpeedSz(), 0.0);
        assertEquals(gmxy, estimator.getAngularSpeedMxy(), 0.0);
        assertEquals(gmxz, estimator.getAngularSpeedMxz(), 0.0);
        assertEquals(gmyx, estimator.getAngularSpeedMyx(), 0.0);
        assertEquals(gmyz, estimator.getAngularSpeedMyz(), 0.0);
        assertEquals(gmzx, estimator.getAngularSpeedMzx(), 0.0);
        assertEquals(gmzy, estimator.getAngularSpeedMzy(), 0.0);
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        assertEquals(gg, gg1);
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg2);
        assertTrue(estimator.isFixKinematicsEnabled());
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());
        final var timeInterval2 = new Time(1.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval2);
        assertEquals(timeInterval1, timeInterval2);
        assertTrue(estimator.isReady());
        assertFalse(estimator.isRunning());
        assertEquals(0, estimator.getNumberOfProcessedSamples());
        assertNull(estimator.getCurrentPositionDrift());
        assertFalse(estimator.getCurrentPositionDrift(null));
        assertNull(estimator.getCurrentVelocityDrift());
        assertFalse(estimator.getCurrentVelocityDrift(null));
        assertNull(estimator.getCurrentOrientationDrift());
        assertFalse(estimator.getCurrentOrientationDrift(null));
        assertNull(estimator.getCurrentPositionDriftNormMeters());
        assertNull(estimator.getCurrentPositionDriftNorm());
        assertFalse(estimator.getCurrentPositionDriftNorm(null));
        assertNull(estimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(estimator.getCurrentVelocityDriftNorm());
        assertFalse(estimator.getCurrentVelocityDriftNorm(null));
        assertNull(estimator.getCurrentOrientationDriftRadians());
        assertNull(estimator.getCurrentOrientationDriftAngle());
        assertFalse(estimator.getCurrentOrientationDriftAngle(null));
        assertNull(estimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(estimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(estimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(estimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
        assertSame(kalmanConfig, estimator.getKalmanConfig());
        assertSame(initConfig, estimator.getInitConfig());
        assertNull(estimator.getState());
        assertFalse(estimator.getState(null));

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, wrong, bg, mg, gg, 
                kalmanConfig, initConfig, this));
        assertThrows(AlgebraException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, wrong, gg,
                kalmanConfig, initConfig, this));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m1, ma, bg, mg, gg,
                kalmanConfig, initConfig, this));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, m2, ma, bg, mg, gg,
                kalmanConfig, initConfig, this));
        final var m3 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m3, bg, mg, gg,
                kalmanConfig, initConfig, this));
        final var m4 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, m4, bg, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m1, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, m2, mg, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m3, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, m4, gg,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, m3,
                kalmanConfig, initConfig, this));
        assertThrows(IllegalArgumentException.class, () -> new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, m4,
                kalmanConfig, initConfig, this));
    }

    @Test
    void testGetSetListener() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getListener());

        // set a new value
        estimator.setListener(this);

        // check
        assertSame(this, estimator.getListener());
    }

    @Test
    void testGetSetReferenceFrame() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceFrame());

        // set a new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        estimator.setReferenceFrame(ecefFrame);

        // check
        assertSame(ecefFrame, estimator.getReferenceFrame());
        assertTrue(nedFrame.equals(estimator.getReferenceNedFrame(), FRAME_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetReferenceNedFrame() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedFrame());

        // set a new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        estimator.setReferenceNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame1 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame1, FRAME_ABSOLUTE_ERROR));
        final var nedFrame2 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame2));
        assertEquals(nedFrame1, nedFrame2);

        // set again
        estimator.setReferenceNedFrame(nedFrame);

        // check
        assertEquals(ecefFrame, estimator.getReferenceFrame());
        final var nedFrame3 = estimator.getReferenceNedFrame();
        assertTrue(nedFrame.equals(nedFrame3, FRAME_ABSOLUTE_ERROR));
        final var nedFrame4 = new NEDFrame();
        assertTrue(estimator.getReferenceNedFrame(nedFrame4));
        assertEquals(nedFrame3, nedFrame4);

        // set to null
        estimator.setReferenceNedFrame(null);

        // check
        assertNull(estimator.getReferenceFrame());
        assertNull(estimator.getReferenceNedFrame());
        assertFalse(estimator.getReferenceNedFrame(null));
    }

    @Test
    void testGetSetReferenceEcefPosition() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceEcefPosition());
        assertFalse(estimator.getReferenceEcefPosition(null));

        // set a new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefPosition = ecefFrame.getECEFPosition();
        final var nedPosition = nedFrame.getPosition();

        estimator.setReferenceEcefPosition(ecefPosition);

        // check
        final var ecefPosition1 = estimator.getReferenceEcefPosition();
        assertEquals(ecefPosition, ecefPosition1);
        final var ecefPosition2 = new ECEFPosition();
        assertTrue(estimator.getReferenceEcefPosition(ecefPosition2));
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition.equals(nedPosition1, FRAME_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetReferenceEcefVelocity() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceEcefVelocity());
        assertFalse(estimator.getReferenceEcefVelocity(null));

        // set a random position to avoid singularity at Earth's center
        final var nedFrame = new NEDFrame();
        nedFrame.setVelocityCoordinates(1.0, 2.0, 3.0);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefPosition = ecefFrame.getECEFPosition();
        estimator.setReferenceEcefPosition(ecefPosition);

        // set a new velocity value
        final var ecefVelocity = ecefFrame.getECEFVelocity();
        final var nedVelocity = nedFrame.getVelocity();

        estimator.setReferenceEcefVelocity(ecefVelocity);

        // check
        final var ecefVelocity1 = estimator.getReferenceEcefVelocity();
        assertEquals(ecefVelocity, ecefVelocity1);
        final var ecefVelocity2 = new ECEFVelocity();
        assertTrue(estimator.getReferenceEcefVelocity(ecefVelocity2));
        assertEquals(ecefVelocity, ecefVelocity2);
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity.equals(nedVelocity1, FRAME_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetReferenceEcefCoordinateTransformation() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceEcefCoordinateTransformation());
        assertFalse(estimator.getReferenceEcefCoordinateTransformation(null));

        // set a new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var nedC = nedFrame.getCoordinateTransformation();

        estimator.setReferenceEcefCoordinateTransformation(ecefC);

        // check
        final var ecefC1 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC, ecefC1);
        final var ecefC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getReferenceEcefCoordinateTransformation(ecefC2);
        assertEquals(ecefC, ecefC2);
        assertTrue(nedC.equals(estimator.getReferenceNedCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));

        // set again
        estimator.setReferenceEcefCoordinateTransformation(ecefC);

        // check
        final var ecefC3 = estimator.getReferenceEcefCoordinateTransformation();
        assertEquals(ecefC, ecefC3);
        final var ecefC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        estimator.getReferenceEcefCoordinateTransformation(ecefC4);
        assertEquals(ecefC, ecefC4);
        assertTrue(nedC.equals(estimator.getReferenceNedCoordinateTransformation(), FRAME_ABSOLUTE_ERROR));
    }

    @Test
    void testGetSetReferenceNedPosition() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedPosition());
        assertFalse(estimator.getReferenceNedPosition(null));

        // set a new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var nedPosition = nedFrame.getPosition();
        final var ecefPosition = ecefFrame.getECEFPosition();

        estimator.setReferenceNedPosition(nedPosition);

        // check
        final var nedPosition1 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition.equals(nedPosition1, FRAME_ABSOLUTE_ERROR));
        final var nedPosition2 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition2));
        assertEquals(nedPosition1, nedPosition2);
        assertEquals(ecefPosition, estimator.getReferenceEcefPosition());

        // set again
        estimator.setReferenceNedPosition(nedPosition);

        // check
        final var nedPosition3 = estimator.getReferenceNedPosition();
        assertTrue(nedPosition.equals(nedPosition3, FRAME_ABSOLUTE_ERROR));
        final var nedPosition4 = new NEDPosition();
        assertTrue(estimator.getReferenceNedPosition(nedPosition4));
        assertEquals(nedPosition3, nedPosition4);
        assertEquals(ecefPosition, estimator.getReferenceEcefPosition());
    }

    @Test
    void testGetSetReferenceNedVelocity() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedVelocity());
        assertFalse(estimator.getReferenceNedVelocity(null));

        // set a new value
        final var nedFrame = new NEDFrame();
        nedFrame.setVelocityCoordinates(1.0, 2.0, 3.0);
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var nedVelocity = nedFrame.getVelocity();
        final var ecefVelocity = ecefFrame.getECEFVelocity();

        estimator.setReferenceNedVelocity(nedVelocity);

        // check
        final var nedVelocity1 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity.equals(nedVelocity1, FRAME_ABSOLUTE_ERROR));
        final var nedVelocity2 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity2));
        assertEquals(nedVelocity1, nedVelocity2);
        assertEquals(ecefVelocity, estimator.getReferenceEcefVelocity());

        // set again
        estimator.setReferenceNedVelocity(nedVelocity);

        // check
        final var nedVelocity3 = estimator.getReferenceNedVelocity();
        assertTrue(nedVelocity.equals(nedVelocity3, FRAME_ABSOLUTE_ERROR));
        final var nedVelocity4 = new NEDVelocity();
        assertTrue(estimator.getReferenceNedVelocity(nedVelocity4));
        assertEquals(nedVelocity3, nedVelocity4);
        assertEquals(ecefVelocity, estimator.getReferenceEcefVelocity());
    }

    @Test
    void testGetSetReferenceNedCoordinateTransformation() throws InvalidSourceAndDestinationFrameTypeException,
            LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getReferenceNedCoordinateTransformation());
        assertFalse(estimator.getReferenceNedCoordinateTransformation(null));

        // set a new value
        final var nedFrame = new NEDFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);
        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var nedC = nedFrame.getCoordinateTransformation();

        estimator.setReferenceNedCoordinateTransformation(nedC);

        // check
        final var nedC1 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC.equals(nedC1, FRAME_ABSOLUTE_ERROR));
        final var nedC2 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC2));
        assertEquals(nedC1, nedC2);
        assertEquals(ecefC, estimator.getReferenceEcefCoordinateTransformation());

        // set again
        estimator.setReferenceNedCoordinateTransformation(nedC);

        // check
        final var nedC3 = estimator.getReferenceNedCoordinateTransformation();
        assertTrue(nedC.equals(nedC3, FRAME_ABSOLUTE_ERROR));
        final var nedC4 = new CoordinateTransformation(FrameType.BODY_FRAME, FrameType.BODY_FRAME);
        assertTrue(estimator.getReferenceNedCoordinateTransformation(nedC4));
        assertEquals(nedC3, nedC4);
        assertEquals(ecefC, estimator.getReferenceEcefCoordinateTransformation());
    }

    @Test
    void testGetSetAccelerationBias() throws WrongSizeException, LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 1), estimator.getAccelerationBias());

        // set a new value
        final var ba = generateBa();
        estimator.setAccelerationBias(ba);

        // check
        final var ba1 = estimator.getAccelerationBias();
        assertEquals(ba, ba1);
        final var ba2 = new Matrix(3, 1);
        estimator.getAccelerationBias(ba2);
        assertEquals(ba, ba2);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationBias(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationBias(m2));
    }

    @Test
    void testGetSetAccelerationBiasArray() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var ba1 = estimator.getAccelerationBiasArray();
        assertArrayEquals(new double[3], ba1, 0.0);
        final var ba2 = new double[3];
        estimator.getAccelerationBiasArray(ba2);
        assertArrayEquals(ba1, ba2, 0.0);

        // set a new value
        final var ba = generateBa().getBuffer();
        estimator.setAccelerationBias(ba);

        // check
        final var ba3 = estimator.getAccelerationBiasArray();
        assertArrayEquals(ba, ba3, 0.0);
        final var ba4 = new double[3];
        estimator.getAccelerationBiasArray(ba4);
        assertArrayEquals(ba, ba4, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationBias(new double[1]));
    }

    @Test
    void testGetSetAccelerationBiasAsTriad() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var triad1 = estimator.getAccelerationBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.getUnit());

        // set a new value
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);
        final var triad2 = new AccelerationTriad(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax, bay, baz);
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
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        estimator.setAccelerationBiasX(bax);

        // check
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasY() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var bay = ba.getElementAtIndex(1);
        estimator.setAccelerationBiasY(bay);

        // check
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasZ() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set a new value
        final var ba = generateBa();
        final var baz = ba.getElementAtIndex(2);
        estimator.setAccelerationBiasZ(baz);

        // check
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testSetAccelerationBias1() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new values
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        estimator.setAccelerationBias(bax, bay, baz);

        // check
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerationBiasXAsAcceleration() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var bax1 = estimator.getAccelerationBiasXAsAcceleration();
        assertEquals(0.0, bax1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bax1.getUnit());

        // set a new value
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bax2 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        estimator.setAccelerationBiasX(bax2);

        // check
        final var bax3 = estimator.getAccelerationBiasXAsAcceleration();
        final var bax4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasXAsAcceleration(bax4);
        assertEquals(bax2, bax3);
        assertEquals(bax2, bax4);
    }

    @Test
    void testGetSetAccelerationBiasYAsAcceleration() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var bay1 = estimator.getAccelerationBiasYAsAcceleration();
        assertEquals(0.0, bay1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, bay1.getUnit());

        // set a new value
        final var ba = generateBa();
        final var bay = ba.getElementAtIndex(1);
        final var bay2 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        estimator.setAccelerationBiasY(bay2);

        // check
        final var bay3 = estimator.getAccelerationBiasYAsAcceleration();
        final var bay4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasYAsAcceleration(bay4);
        assertEquals(bay2, bay3);
        assertEquals(bay2, bay4);
    }

    @Test
    void testGetSetAccelerationBiasZAsAcceleration() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var baz1 = estimator.getAccelerationBiasZAsAcceleration();
        assertEquals(0.0, baz1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, baz1.getUnit());

        // set a new value
        final var ba = generateBa();
        final var baz = ba.getElementAtIndex(2);
        final var baz2 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        estimator.setAccelerationBiasZ(baz2);

        // check
        final var baz3 = estimator.getAccelerationBiasZAsAcceleration();
        final var baz4 = new Acceleration(1.0, AccelerationUnit.FEET_PER_SQUARED_SECOND);
        estimator.getAccelerationBiasZAsAcceleration(baz4);
        assertEquals(baz2, baz3);
        assertEquals(baz2, baz4);
    }

    @Test
    void testSetAccelerationBias2() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default values
        assertEquals(0.0, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(0.0, estimator.getAccelerationBiasZ(), 0.0);

        // set new values
        final var ba = generateBa();
        final var bax = ba.getElementAtIndex(0);
        final var bay = ba.getElementAtIndex(1);
        final var baz = ba.getElementAtIndex(2);

        final var bax1 = new Acceleration(bax, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var bay1 = new Acceleration(bay, AccelerationUnit.METERS_PER_SQUARED_SECOND);
        final var baz1 = new Acceleration(baz, AccelerationUnit.METERS_PER_SQUARED_SECOND);

        estimator.setAccelerationBias(bax1, bay1, baz1);

        // check
        assertEquals(bax, estimator.getAccelerationBiasX(), 0.0);
        assertEquals(bay, estimator.getAccelerationBiasY(), 0.0);
        assertEquals(baz, estimator.getAccelerationBiasZ(), 0.0);
    }

    @Test
    void testGetSetAccelerationCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 3), estimator.getAccelerationCrossCouplingErrors());

        // set a new value
        final var ma = generateMaGeneral();
        estimator.setAccelerationCrossCouplingErrors(ma);

        // check
        final var ma1 = estimator.getAccelerationCrossCouplingErrors();
        final var ma2 = new Matrix(3, 3);
        estimator.getAccelerationCrossCouplingErrors(ma2);
        assertEquals(ma, ma1);
        assertEquals(ma, ma2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> estimator.setAccelerationCrossCouplingErrors(wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationCrossCouplingErrors(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAccelerationCrossCouplingErrors(m2));
    }

    @Test
    void testGetSetAccelerationSx() throws AlgebraException, LockedException {
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

        // check default values
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
        final var estimator = new KalmanDriftEstimator();

        // check default values
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
        final var estimator = new KalmanDriftEstimator();

        // check default values
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
    }

    @Test
    void testGetSetAngularSpeedBias() throws WrongSizeException, LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 1), estimator.getAngularSpeedBias());

        // set a new value
        final var bg = generateBg();
        estimator.setAngularSpeedBias(bg);

        // check
        final var bg1 = estimator.getAngularSpeedBias();
        assertEquals(bg, bg1);
        final var bg2 = new Matrix(3, 1);
        estimator.getAngularSpeedBias(bg2);
        assertEquals(bg, bg2);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedBias(m1));
        final var m2 = new Matrix(3, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedBias(m2));
    }

    @Test
    void testGetSetAngularSpeedBiasArray() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var bg1 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(new double[3], bg1, 0.0);
        final var bg2 = new double[3];
        estimator.getAngularSpeedBiasArray(bg2);
        assertArrayEquals(bg1, bg2, 0.0);

        // set a new value
        final var bg = generateBg().getBuffer();
        estimator.setAngularSpeedBias(bg);

        // check
        final var bg3 = estimator.getAngularSpeedBiasArray();
        assertArrayEquals(bg, bg3, 0.0);
        final var bg4 = new double[3];
        estimator.getAngularSpeedBiasArray(bg4);
        assertArrayEquals(bg, bg4, 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedBias(new double[1]));
    }

    @Test
    void testGetSetAngularSpeedBiasAsTriad() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var triad1 = estimator.getAngularSpeedBiasAsTriad();
        assertEquals(0.0, triad1.getValueX(), 0.0);
        assertEquals(0.0, triad1.getValueY(), 0.0);
        assertEquals(0.0, triad1.getValueZ(), 0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.getUnit());

        // set a new value
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);
        final var triad2 = new AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, bgx, bgy, bgz);
        estimator.setAngularSpeedBias(triad2);

        // check
        final var triad3 = estimator.getAngularSpeedBiasAsTriad();
        final var triad4 = new AngularSpeedTriad();
        estimator.getAngularSpeedBiasAsTriad(triad4);
        assertEquals(triad2, triad3);
        assertEquals(triad2, triad4);
    }

    @Test
    void testGetSetAngularSpeedBiasX() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

        // check default values
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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
    void testSetAngularSpeedBias2() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(0.0, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(0.0, estimator.getAngularSpeedBiasZ(), 0.0);

        // set new values
        final var bg = generateBg();
        final var bgx = bg.getElementAtIndex(0);
        final var bgy = bg.getElementAtIndex(1);
        final var bgz = bg.getElementAtIndex(2);

        final var bgx1 = new AngularSpeed(bgx, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgy1 = new AngularSpeed(bgy, AngularSpeedUnit.RADIANS_PER_SECOND);
        final var bgz1 = new AngularSpeed(bgz, AngularSpeedUnit.RADIANS_PER_SECOND);

        estimator.setAngularSpeedBias(bgx1, bgy1, bgz1);

        // check
        assertEquals(bgx, estimator.getAngularSpeedBiasX(), 0.0);
        assertEquals(bgy, estimator.getAngularSpeedBiasY(), 0.0);
        assertEquals(bgz, estimator.getAngularSpeedBiasZ(), 0.0);
    }

    @Test
    void testGetSetAngularSpeedCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedCrossCouplingErrors());

        // set a new value
        final var mg = generateMg();
        estimator.setAngularSpeedCrossCouplingErrors(mg);

        // check
        final var mg1 = estimator.getAngularSpeedCrossCouplingErrors();
        final var mg2 = new Matrix(3, 3);
        estimator.getAngularSpeedCrossCouplingErrors(mg2);
        assertEquals(mg, mg1);
        assertEquals(mg, mg2);

        // Force AlgebraException
        final var wrong = Matrix.identity(3, 3).multiplyByScalarAndReturnNew(-1.0);
        assertThrows(AlgebraException.class, () -> estimator.setAngularSpeedCrossCouplingErrors(wrong));

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedCrossCouplingErrors(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedCrossCouplingErrors(m2));
    }

    @Test
    void testGetSetAngularSpeedSx() throws AlgebraException, LockedException {
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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

        // check
        assertEquals(sx, estimator.getAngularSpeedSx(), 0.0);
        assertEquals(sy, estimator.getAngularSpeedSy(), 0.0);
        assertEquals(sz, estimator.getAngularSpeedSz(), 0.0);
    }

    @Test
    void testSetAngularSpeedCrossCouplingErrors() throws AlgebraException, LockedException {
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

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
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(new Matrix(3, 3), estimator.getAngularSpeedGDependantCrossBias());

        // set a new value
        final var gg = generateGg();
        estimator.setAngularSpeedGDependantCrossBias(gg);

        // check
        final var gg1 = estimator.getAngularSpeedGDependantCrossBias();
        final var gg2 = new Matrix(3, 3);
        estimator.getAngularSpeedGDependantCrossBias(gg2);
        assertEquals(gg, gg1);
        assertEquals(gg, gg2);

        // Force IllegalArgumentException
        final var m1 = new Matrix(1, 3);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedGDependantCrossBias(m1));
        final var m2 = new Matrix(3, 1);
        assertThrows(IllegalArgumentException.class, () -> estimator.setAngularSpeedGDependantCrossBias(m2));
    }

    @Test
    void testIsSetFixKinematicsEnabled() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        assertTrue(estimator.isFixKinematicsEnabled());

        // set a new value
        estimator.setFixKinematicsEnabled(false);

        // check
        assertFalse(estimator.isFixKinematicsEnabled());
    }

    @Test
    void testGetSetTimeInterval() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, estimator.getTimeInterval(), 0.0);

        // set a new value
        estimator.setTimeInterval(1.0);

        // check
        assertEquals(1.0, estimator.getTimeInterval(), 0.0);

        // Force IllegalArgumentException
        assertThrows(IllegalArgumentException.class, () -> estimator.setTimeInterval(-1.0));
    }

    @Test
    void testGetSetTimeIntervalAsTime() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        final var timeInterval1 = estimator.getTimeIntervalAsTime();
        assertEquals(DriftEstimator.DEFAULT_TIME_INTERVAL_SECONDS, timeInterval1.getValue().doubleValue(), 0.0);
        assertEquals(TimeUnit.SECOND, timeInterval1.getUnit());

        // set a new value
        final var timeInterval2 = new Time(1.0, TimeUnit.SECOND);
        estimator.setTimeInterval(timeInterval2);

        // check
        final var timeInterval3 = estimator.getTimeIntervalAsTime();
        final var timeInterval4 = new Time(3.0, TimeUnit.DAY);
        estimator.getTimeIntervalAsTime(timeInterval4);
        assertEquals(timeInterval2, timeInterval3);
        assertEquals(timeInterval2, timeInterval4);
    }

    @Test
    void testGetSetKalmanConfig() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getKalmanConfig());

        // set a new value
        final var kalmanConfig = new INSLooselyCoupledKalmanConfig();
        estimator.setKalmanConfig(kalmanConfig);

        // check
        assertSame(kalmanConfig, estimator.getKalmanConfig());
    }

    @Test
    void testGetSetInitConfig() throws LockedException {
        final var estimator = new KalmanDriftEstimator();

        // check default value
        assertNull(estimator.getInitConfig());

        // set a new value
        final var initConfig = new INSLooselyCoupledKalmanInitializerConfig();
        estimator.setInitConfig(initConfig);

        // check
        assertSame(initConfig, estimator.getInitConfig());
    }

    @Disabled("usually fails")
    @Test
    void testAddBodyKinematicsAndResetExactCalibrationNoNoiseAndKinematicsFixed() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InertialNavigatorException,
            DriftEstimationException, NotReadyException, InvalidRotationMatrixException, RotationException,
            INSException {

        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();

        final var ba = generateBa();
        final var ma = generateMaCommonAxis();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel, 
                gyroQuantLevel);

        final var nedFrame = generateFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var navigationFrame = new ECEFFrame(ecefFrame);
        final var fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var kalmanDriftEstimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, gg, kalmanConfig, 
                initConfig, this);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var driftEstimator = new DriftEstimator(nedFrame, ba, ma, bg, mg, gg);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var kalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, 
                new ECEFFrame(ecefFrame));

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefC,
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);
        final var fixedKinematics = new BodyKinematics();

        final var measuredKinematics = new BodyKinematics();
        final var random = new Random();
        for (var i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(i + 1, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());

            fixer.fix(measuredKinematics, fixedKinematics);
            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, navigationFrame, fixedKinematics, 
                    navigationFrame);

            kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(N_SAMPLES, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var kalmanState = kalmanEstimator.getState();

        final var navigationPositionDrift = positionDrift(ecefFrame, navigationFrame);
        final var navigationVelocityDrift = velocityDrift(ecefFrame, navigationFrame);
        final var navigationOrientationDrift = orientationDrift(ecefFrame, navigationFrame);

        final var kalmanPositionDrift = positionDrift(ecefFrame, kalmanState);
        final var kalmanVelocityDrift = velocityDrift(ecefFrame, kalmanState);
        final var kalmanOrientationDrift = orientationDrift(ecefFrame, kalmanState);

        final var currentPositionDrift1 = kalmanDriftEstimator.getCurrentPositionDrift();
        final var currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final var currentVelocityDrift1 = kalmanDriftEstimator.getCurrentVelocityDrift();
        final var currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final var currentOrientationDrift1 = kalmanDriftEstimator.getCurrentOrientationDrift();
        final var currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final var distanceFormatter = new DistanceFormatter();
        final var speedFormatter = new SpeedFormatter();
        final var angleFormatter = new AngleFormatter();
        final var accelerationFormatter = new AccelerationFormatter();
        final var angularSpeedFormatter = new AngularSpeedFormatter();

        final var currentPositionDriftNorm = kalmanDriftEstimator.getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current position drift: %s", distanceFormatter.format(
                currentPositionDriftNorm, DistanceUnit.METER)));
        assertTrue(currentPositionDriftNorm < 8.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm <= navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift, ABSOLUTE_ERROR);

        final var currentPositionDriftNorm1 = kalmanDriftEstimator.getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm, currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final var currentPositionDriftNorm2 = new Distance(1.0, DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final var currentVelocityDriftNorm = kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current velocity drift: %s", speedFormatter.format(
                currentVelocityDriftNorm, SpeedUnit.METERS_PER_SECOND)));
        assertTrue(currentVelocityDriftNorm < 0.7);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift, ABSOLUTE_ERROR);

        final var currentVelocityDriftNorm1 = kalmanDriftEstimator.getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm, currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentVelocityDriftNorm1.getUnit());
        final var currentVelocityDriftNorm2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final var currentOrientationDriftNorm = kalmanDriftEstimator.getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm, currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current orientation drift: %s", angleFormatter.format(
                AngleConverter.convert(currentOrientationDriftNorm, AngleUnit.RADIANS, AngleUnit.DEGREES),
                AngleUnit.DEGREES)));
        assertTrue(currentOrientationDriftNorm < 0.006);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm <= navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift, ABSOLUTE_ERROR);

        final var currentOrientationDriftNorm1 = kalmanDriftEstimator.getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm, currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final var currentOrientationDriftNorm2 = new Angle(1.0, AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final var currentPositionDriftPerTimeUnit = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current position drift per time unit: %s", speedFormatter.format(
                currentPositionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)));
        assertTrue(currentPositionDriftPerTimeUnit < 0.3);
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentPositionDriftPerTimeUnit1 = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftPerTimeUnit1.getValue().doubleValue(), 
                0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentPositionDriftPerTimeUnit1.getUnit());
        final var currentPositionDriftPerTimeUnit2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1, currentPositionDriftPerTimeUnit2);

        final var currentVelocityDriftPerTimeUnit = kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current velocity drift per time unit: %s", accelerationFormatter.format(
                currentVelocityDriftPerTimeUnit, AccelerationUnit.METERS_PER_SQUARED_SECOND)));
        assertTrue(currentVelocityDriftPerTimeUnit < 0.03);
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentVelocityDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftPerTimeUnit1.getValue().doubleValue(), 
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, currentVelocityDriftPerTimeUnit1.getUnit());
        final var currentVelocityDriftPerTimeUnit2 = new Acceleration(1.0, 
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1, currentVelocityDriftPerTimeUnit2);

        final var currentOrientationDriftPerTimeUnit = kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current orientation drift per time unit: %s",
                angularSpeedFormatter.format(AngularSpeedConverter.convert(currentOrientationDriftPerTimeUnit,
                                AngularSpeedUnit.RADIANS_PER_SECOND, AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND)));
        assertTrue(currentOrientationDriftPerTimeUnit < 2e-4);
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentOrientationDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, currentOrientationDriftPerTimeUnit1.getUnit());
        final var currentOrientationDriftPerTimeUnit2 = new AngularSpeed(1.0, 
                AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1, currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(1, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
    }

    @Test
    void testAddBodyKinematicsAndResetExactCalibrationNoNoiseAndKinematicsNotFixed() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InertialNavigatorException, 
            DriftEstimationException, NotReadyException, InvalidRotationMatrixException, RotationException,
            INSException {

        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();

        final var ba = generateBa();
        final var ma = generateMaCommonAxis();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = 0.0;
        final var gyroNoiseRootPSD = 0.0;
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var nedFrame = generateFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var navigationFrame = new ECEFFrame(ecefFrame);

        final var kalmanDriftEstimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, gg, kalmanConfig, 
                initConfig, this);
        kalmanDriftEstimator.setFixKinematicsEnabled(false);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var driftEstimator = new DriftEstimator(nedFrame, ba, ma, bg, mg, gg);
        driftEstimator.setFixKinematicsEnabled(false);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var kalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig,
                new ECEFFrame(ecefFrame));

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertFalse(kalmanDriftEstimator.isFixKinematicsEnabled());

        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefC,
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        final var measuredKinematics = new BodyKinematics();
        final var random = new Random();
        for (var i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(i + 1, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());

            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, navigationFrame, measuredKinematics, 
                    navigationFrame);

            kalmanEstimator.update(measuredKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(N_SAMPLES, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var kalmanState = kalmanEstimator.getState();

        final var navigationPositionDrift = positionDrift(ecefFrame, navigationFrame);
        final var navigationVelocityDrift = velocityDrift(ecefFrame, navigationFrame);
        final var navigationOrientationDrift = orientationDrift(ecefFrame, navigationFrame);

        final var kalmanPositionDrift = positionDrift(ecefFrame, kalmanState);
        final var kalmanVelocityDrift = velocityDrift(ecefFrame, kalmanState);
        final var kalmanOrientationDrift = orientationDrift(ecefFrame, kalmanState);

        final var currentPositionDrift1 = kalmanDriftEstimator.getCurrentPositionDrift();
        final var currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final var currentVelocityDrift1 = kalmanDriftEstimator.getCurrentVelocityDrift();
        final var currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final var currentOrientationDrift1 = kalmanDriftEstimator.getCurrentOrientationDrift();
        final var currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final var distanceFormatter = new DistanceFormatter();
        final var speedFormatter = new SpeedFormatter();
        final var angleFormatter = new AngleFormatter();
        final var accelerationFormatter = new AccelerationFormatter();
        final var angularSpeedFormatter = new AngularSpeedFormatter();

        final var currentPositionDriftNorm = kalmanDriftEstimator.getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current position drift: %s", distanceFormatter.format(
                currentPositionDriftNorm, DistanceUnit.METER)));
        assertTrue(currentPositionDriftNorm < 17.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm < navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift, ABSOLUTE_ERROR);

        final var currentPositionDriftNorm1 = kalmanDriftEstimator.getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm, currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final var currentPositionDriftNorm2 = new Distance(1.0, DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final var currentVelocityDriftNorm = kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current velocity drift: %s", speedFormatter.format(
                currentVelocityDriftNorm, SpeedUnit.METERS_PER_SECOND)));
        assertTrue(currentVelocityDriftNorm < 1.3);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift, ABSOLUTE_ERROR);

        final var currentVelocityDriftNorm1 = kalmanDriftEstimator.getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm, currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentVelocityDriftNorm1.getUnit());
        final var currentVelocityDriftNorm2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final var currentOrientationDriftNorm = kalmanDriftEstimator.getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm, currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current orientation drift: %s",
                angleFormatter.format(AngleConverter.convert(
                        currentOrientationDriftNorm, AngleUnit.RADIANS,
                        AngleUnit.DEGREES), AngleUnit.DEGREES)));
        assertTrue(currentOrientationDriftNorm < 0.008);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift, ABSOLUTE_ERROR);

        final var currentOrientationDriftNorm1 = kalmanDriftEstimator.getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm, currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final var currentOrientationDriftNorm2 = new Angle(1.0, AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final var currentPositionDriftPerTimeUnit = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current position drift per time unit: %s", speedFormatter.format(
                currentPositionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)));
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentPositionDriftPerTimeUnit1 = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftPerTimeUnit1.getValue().doubleValue(), 
                0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentPositionDriftPerTimeUnit1.getUnit());
        final var currentPositionDriftPerTimeUnit2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1, currentPositionDriftPerTimeUnit2);

        final var currentVelocityDriftPerTimeUnit = kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current velocity drift per time unit: %s", accelerationFormatter.format(
                currentVelocityDriftPerTimeUnit, AccelerationUnit.METERS_PER_SQUARED_SECOND)));
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentVelocityDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftPerTimeUnit1.getValue().doubleValue(), 
                0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, currentVelocityDriftPerTimeUnit1.getUnit());
        final var currentVelocityDriftPerTimeUnit2 = new Acceleration(1.0, 
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1, currentVelocityDriftPerTimeUnit2);

        final var currentOrientationDriftPerTimeUnit = kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current orientation drift per time unit: %s",
                angularSpeedFormatter.format(AngularSpeedConverter.convert(currentOrientationDriftPerTimeUnit,
                                AngularSpeedUnit.RADIANS_PER_SECOND, AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND)));
        assertEquals(currentOrientationDriftPerTimeUnit,
                currentOrientationDriftNorm / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentOrientationDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, currentOrientationDriftPerTimeUnit1.getUnit());
        final var currentOrientationDriftPerTimeUnit2 = new AngularSpeed(1.0, 
                AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1, currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(1, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
    }
    
    @Disabled("usually fails")
    @Test
    void testAddBodyKinematicsAndResetExactCalibrationWithNoiseAndKinematicsFixed() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InertialNavigatorException,
            DriftEstimationException, NotReadyException, InvalidRotationMatrixException, RotationException,
            INSException {

        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();

        final var ba = generateBa();
        final var ma = generateMaCommonAxis();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var nedFrame = generateFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var navigationFrame = new ECEFFrame(ecefFrame);
        final var fixer = new BodyKinematicsFixer();
        fixer.setAccelerationBias(ba);
        fixer.setAccelerationCrossCouplingErrors(ma);
        fixer.setAngularSpeedBias(bg);
        fixer.setAngularSpeedCrossCouplingErrors(mg);
        fixer.setAngularSpeedGDependantCrossBias(gg);

        final var kalmanDriftEstimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, gg, kalmanConfig, 
                initConfig, this);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var driftEstimator = new DriftEstimator(nedFrame, ba, ma, bg, mg, gg);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var kalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, 
                new ECEFFrame(ecefFrame));

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefC,
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);
        final var fixedKinematics = new BodyKinematics();

        final var measuredKinematics = new BodyKinematics();
        final var random = new Random();
        for (var i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(i + 1, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());

            fixer.fix(measuredKinematics, fixedKinematics);
            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, navigationFrame, fixedKinematics, 
                    navigationFrame);

            kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(N_SAMPLES, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var kalmanState = kalmanEstimator.getState();

        final var navigationPositionDrift = positionDrift(ecefFrame, navigationFrame);
        final var navigationVelocityDrift = velocityDrift(ecefFrame, navigationFrame);
        final var navigationOrientationDrift = orientationDrift(ecefFrame, navigationFrame);

        final var kalmanPositionDrift = positionDrift(ecefFrame, kalmanState);
        final var kalmanVelocityDrift = velocityDrift(ecefFrame, kalmanState);
        final var kalmanOrientationDrift = orientationDrift(ecefFrame, kalmanState);

        final var currentPositionDrift1 = kalmanDriftEstimator.getCurrentPositionDrift();
        final var currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final var currentVelocityDrift1 = kalmanDriftEstimator.getCurrentVelocityDrift();
        final var currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final var currentOrientationDrift1 = kalmanDriftEstimator.getCurrentOrientationDrift();
        final var currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final var distanceFormatter = new DistanceFormatter();
        final var speedFormatter = new SpeedFormatter();
        final var angleFormatter = new AngleFormatter();
        final var accelerationFormatter = new AccelerationFormatter();
        final var angularSpeedFormatter = new AngularSpeedFormatter();

        final var currentPositionDriftNorm = kalmanDriftEstimator.getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current position drift: %s", distanceFormatter.format(
                currentPositionDriftNorm, DistanceUnit.METER)));
        assertTrue(currentPositionDriftNorm < 8.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm < navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift, ABSOLUTE_ERROR);

        final var currentPositionDriftNorm1 = kalmanDriftEstimator.getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm, currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final var currentPositionDriftNorm2 = new Distance(1.0, DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final var currentVelocityDriftNorm = kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current velocity drift: %s", speedFormatter.format(
                currentVelocityDriftNorm, SpeedUnit.METERS_PER_SECOND)));
        assertTrue(currentVelocityDriftNorm < 0.7);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift, ABSOLUTE_ERROR);

        final var currentVelocityDriftNorm1 = kalmanDriftEstimator.getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm, currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentVelocityDriftNorm1.getUnit());
        final var currentVelocityDriftNorm2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final var currentOrientationDriftNorm = kalmanDriftEstimator.getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm, currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current orientation drift: %s", angleFormatter.format(
                AngleConverter.convert(currentOrientationDriftNorm, AngleUnit.RADIANS, AngleUnit.DEGREES),
                AngleUnit.DEGREES)));
        assertTrue(currentOrientationDriftNorm < 0.006);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm <= navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift, ABSOLUTE_ERROR);

        final var currentOrientationDriftNorm1 = kalmanDriftEstimator.getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm, currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final var currentOrientationDriftNorm2 = new Angle(1.0, AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final var currentPositionDriftPerTimeUnit = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current position drift per time unit: %s", speedFormatter.format(
                currentPositionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)));
        assertTrue(currentPositionDriftPerTimeUnit < 0.3);
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentPositionDriftPerTimeUnit1 = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftPerTimeUnit1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentPositionDriftPerTimeUnit1.getUnit());
        final var currentPositionDriftPerTimeUnit2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1, currentPositionDriftPerTimeUnit2);

        final var currentVelocityDriftPerTimeUnit = kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current velocity drift per time unit: %s", accelerationFormatter.format(
                currentVelocityDriftPerTimeUnit, AccelerationUnit.METERS_PER_SQUARED_SECOND)));
        assertTrue(currentVelocityDriftPerTimeUnit < 0.03);
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentVelocityDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftPerTimeUnit1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, currentVelocityDriftPerTimeUnit1.getUnit());
        final var currentVelocityDriftPerTimeUnit2 = new Acceleration(1.0, 
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1, currentVelocityDriftPerTimeUnit2);

        final var currentOrientationDriftPerTimeUnit = kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current orientation drift per time unit: %s",
                angularSpeedFormatter.format(AngularSpeedConverter.convert(currentOrientationDriftPerTimeUnit,
                                AngularSpeedUnit.RADIANS_PER_SECOND, AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND)));
        assertTrue(currentOrientationDriftPerTimeUnit < 2e-4);
        assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftNorm 
                / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentOrientationDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, currentOrientationDriftPerTimeUnit1.getUnit());
        final var currentOrientationDriftPerTimeUnit2 = new AngularSpeed(1.0, 
                AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1, currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(1, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
    }

    @Test
    void testAddBodyKinematicsAndResetExactCalibrationWithNoiseAndKinematicsNotFixed() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InertialNavigatorException,
            DriftEstimationException, NotReadyException, InvalidRotationMatrixException, RotationException,
            INSException {

        final var kalmanConfig = generateKalmanConfig();
        final var initConfig = generateInitConfig();

        final var ba = generateBa();
        final var ma = generateMaCommonAxis();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();
        final var accelNoiseRootPSD = getAccelNoiseRootPSD();
        final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var nedFrame = generateFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var ecefC = ecefFrame.getCoordinateTransformation();
        final var ecefPosition = ecefFrame.getECEFPosition();

        final var navigationFrame = new ECEFFrame(ecefFrame);

        final var kalmanDriftEstimator = new KalmanDriftEstimator(nedFrame, ba, ma, bg, mg, gg, kalmanConfig, 
                initConfig, this);
        kalmanDriftEstimator.setFixKinematicsEnabled(false);
        kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var driftEstimator = new DriftEstimator(nedFrame, ba, ma, bg, mg, gg);
        driftEstimator.setFixKinematicsEnabled(false);
        driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

        final var kalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig, 
                new ECEFFrame(ecefFrame));

        reset();
        assertEquals(0, start);
        assertEquals(0, bodyKinematicsAdded);
        assertEquals(0, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertFalse(kalmanDriftEstimator.isFixKinematicsEnabled());

        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, ecefC,
                ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

        final var measuredKinematics = new BodyKinematics();
        final var random = new Random();
        for (var i = 0; i < N_SAMPLES; i++) {
            BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, measuredKinematics);

            kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
            driftEstimator.addBodyKinematics(measuredKinematics);

            assertEquals(i + 1, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());

            ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, navigationFrame, measuredKinematics, 
                    navigationFrame);

            kalmanEstimator.update(measuredKinematics, i * TIME_INTERVAL_SECONDS);
        }

        assertEquals(N_SAMPLES, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertFalse(kalmanDriftEstimator.isRunning());
        assertEquals(1, start);
        assertEquals(N_SAMPLES, bodyKinematicsAdded);
        assertEquals(0, reset);

        final var kalmanState = kalmanEstimator.getState();

        final var navigationPositionDrift = positionDrift(ecefFrame, navigationFrame);
        final var navigationVelocityDrift = velocityDrift(ecefFrame, navigationFrame);
        final var navigationOrientationDrift = orientationDrift(ecefFrame, navigationFrame);

        final var kalmanPositionDrift = positionDrift(ecefFrame, kalmanState);
        final var kalmanVelocityDrift = velocityDrift(ecefFrame, kalmanState);
        final var kalmanOrientationDrift = orientationDrift(ecefFrame, kalmanState);

        final var currentPositionDrift1 = kalmanDriftEstimator.getCurrentPositionDrift();
        final var currentPositionDrift2 = new ECEFPosition();
        assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
        assertEquals(currentPositionDrift1, currentPositionDrift2);

        final var currentVelocityDrift1 = kalmanDriftEstimator.getCurrentVelocityDrift();
        final var currentVelocityDrift2 = new ECEFVelocity();
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
        assertEquals(currentVelocityDrift1, currentVelocityDrift2);

        final var currentOrientationDrift1 = kalmanDriftEstimator.getCurrentOrientationDrift();
        final var currentOrientationDrift2 = new Quaternion();
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
        assertEquals(currentOrientationDrift1, currentOrientationDrift2);

        final var distanceFormatter = new DistanceFormatter();
        final var speedFormatter = new SpeedFormatter();
        final var angleFormatter = new AngleFormatter();
        final var accelerationFormatter = new AccelerationFormatter();
        final var angularSpeedFormatter = new AngularSpeedFormatter();

        final var currentPositionDriftNorm = kalmanDriftEstimator.getCurrentPositionDriftNormMeters();
        assertNotNull(currentPositionDriftNorm);
        assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current position drift: %s", distanceFormatter.format(
                currentPositionDriftNorm, DistanceUnit.METER)));
        assertTrue(currentPositionDriftNorm < 19.0);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentPositionDriftNorm < navigationPositionDrift);
        assertEquals(currentPositionDriftNorm, kalmanPositionDrift, ABSOLUTE_ERROR);

        final var currentPositionDriftNorm1 = kalmanDriftEstimator.getCurrentPositionDriftNorm();
        assertEquals(currentPositionDriftNorm, currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
        final var currentPositionDriftNorm2 = new Distance(1.0, DistanceUnit.FOOT);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(currentPositionDriftNorm2));
        assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

        final var currentVelocityDriftNorm = kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond();
        assertNotNull(currentVelocityDriftNorm);
        assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current velocity drift: %s", speedFormatter.format(
                currentVelocityDriftNorm, SpeedUnit.METERS_PER_SECOND)));
        assertTrue(currentVelocityDriftNorm < 1.4);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
        assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift, ABSOLUTE_ERROR);

        final var currentVelocityDriftNorm1 = kalmanDriftEstimator.getCurrentVelocityDriftNorm();
        assertEquals(currentVelocityDriftNorm, currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentVelocityDriftNorm1.getUnit());
        final var currentVelocityDriftNorm2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(currentVelocityDriftNorm2));
        assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

        final var currentOrientationDriftNorm = kalmanDriftEstimator.getCurrentOrientationDriftRadians();
        assertNotNull(currentOrientationDriftNorm);
        assertEquals(currentOrientationDriftNorm, currentOrientationDrift1.getRotationAngle(), 0.0);
        LOGGER.log(Level.INFO, String.format("Current orientation drift: %s", angleFormatter.format(
                AngleConverter.convert(currentOrientationDriftNorm, AngleUnit.RADIANS, AngleUnit.DEGREES),
                AngleUnit.DEGREES)));
        assertTrue(currentOrientationDriftNorm < 0.007);
        // Check that kalman corrected drift is smaller than normal navigation drift
        assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
        assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift, ABSOLUTE_ERROR);

        final var currentOrientationDriftNorm1 = kalmanDriftEstimator.getCurrentOrientationDriftAngle();
        assertEquals(currentOrientationDriftNorm, currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
        assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
        final var currentOrientationDriftNorm2 = new Angle(1.0, AngleUnit.DEGREES);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(currentOrientationDriftNorm2));
        assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

        final var currentPositionDriftPerTimeUnit = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
        assertNotNull(currentPositionDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current position drift per time unit: %s", speedFormatter.format(
                currentPositionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)));
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentPositionDriftPerTimeUnit1 = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
        assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftPerTimeUnit1.getValue().doubleValue(), 0.0);
        assertEquals(SpeedUnit.METERS_PER_SECOND, currentPositionDriftPerTimeUnit1.getUnit());
        final var currentPositionDriftPerTimeUnit2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
        assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(currentPositionDriftPerTimeUnit2));
        assertEquals(currentPositionDriftPerTimeUnit1, currentPositionDriftPerTimeUnit2);

        final var currentVelocityDriftPerTimeUnit = kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
        assertNotNull(currentVelocityDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current velocity drift per time unit: %s",
                accelerationFormatter.format(currentVelocityDriftPerTimeUnit, 
                        AccelerationUnit.METERS_PER_SQUARED_SECOND)));
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftNorm 
                        / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentVelocityDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
        assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftPerTimeUnit1.getValue().doubleValue(), 0.0);
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, currentVelocityDriftPerTimeUnit1.getUnit());
        final var currentVelocityDriftPerTimeUnit2 = new Acceleration(1.0, 
                AccelerationUnit.FEET_PER_SQUARED_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                currentVelocityDriftPerTimeUnit2));
        assertEquals(currentVelocityDriftPerTimeUnit1, currentVelocityDriftPerTimeUnit2);

        final var currentOrientationDriftPerTimeUnit = kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
        assertNotNull(currentOrientationDriftPerTimeUnit);
        LOGGER.log(Level.INFO, String.format("Current orientation drift per time unit: %s",
                angularSpeedFormatter.format(AngularSpeedConverter.convert(currentOrientationDriftPerTimeUnit,
                                AngularSpeedUnit.RADIANS_PER_SECOND, AngularSpeedUnit.DEGREES_PER_SECOND),
                        AngularSpeedUnit.DEGREES_PER_SECOND)));
        assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftNorm 
                / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

        final var currentOrientationDriftPerTimeUnit1 = 
                kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
        assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftPerTimeUnit1.getValue().doubleValue(),
                0.0);
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, currentOrientationDriftPerTimeUnit1.getUnit());
        final var currentOrientationDriftPerTimeUnit2 = new AngularSpeed(1.0, 
                AngularSpeedUnit.DEGREES_PER_SECOND);
        assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                currentOrientationDriftPerTimeUnit2));
        assertEquals(currentOrientationDriftPerTimeUnit1, currentOrientationDriftPerTimeUnit2);

        // reset
        kalmanDriftEstimator.reset();

        assertEquals(1, reset);
        assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
        assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
        assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
        assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
        assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));
    }

    @Test
    void testAddBodyKinematicsAndResetApproximateCalibration() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InertialNavigatorException,
            DriftEstimationException, NotReadyException, InvalidRotationMatrixException, RotationException,
            INSException, IntervalDetectorThresholdFactorOptimizerException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var kalmanConfig = generateKalmanConfig();
            final var initConfig = generateInitConfig();

            final var ba = generateBa();
            final var ma = generateMaCommonAxis();
            final var bg = generateBg();
            final var mg = generateMg();
            final var gg =  new Matrix(3, 3);
            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator();
            final var gyroscopeCalibrator = new EasyGyroscopeCalibrator();

            // initialize a threshold optimizer to attempt calibration by generating
            // timed body kinematics with noise and attempting to find the best
            // threshold to find optimal calibration
            final var optimizer = buildOptimizer(ba, ma, accelNoiseRootPSD, gyroNoiseRootPSD, accelerometerCalibrator,
                    gyroscopeCalibrator);
            final var thresholdFactor = optimizer.optimize();

            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);

            final var estimatedBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final var estimatedMa = optimizer.getEstimatedAccelerometerMa();

            final var estimatedBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final var estimatedMg = optimizer.getEstimatedGyroscopeMg();
            final var estimatedGg = optimizer.getEstimatedGyroscopeGg();

            // clear all previously generated data
            timedBodyKinematics.clear();

            // use real calibration values to generate measurements with errors
            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);

            final var nedFrame = generateFrame();
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final var ecefC = ecefFrame.getCoordinateTransformation();
            final var ecefPosition = ecefFrame.getECEFPosition();

            final var navigationFrame = new ECEFFrame(ecefFrame);
            final var fixer = new BodyKinematicsFixer();
            // use estimated calibration data to fix measurements
            fixer.setAccelerationBias(estimatedBa);
            fixer.setAccelerationCrossCouplingErrors(estimatedMa);
            fixer.setAngularSpeedBias(estimatedBg);
            fixer.setAngularSpeedCrossCouplingErrors(estimatedMg);
            fixer.setAngularSpeedGDependantCrossBias(estimatedGg);

            final var kalmanDriftEstimator = new KalmanDriftEstimator(nedFrame, estimatedBa, estimatedMa, estimatedBg,
                    estimatedMg, estimatedGg, kalmanConfig, initConfig, this);
            kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final var driftEstimator = new DriftEstimator(nedFrame, estimatedBa, estimatedMa, estimatedBg, estimatedMg,
                    estimatedGg);
            driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final var kalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig,
                    new ECEFFrame(ecefFrame));

            reset();
            assertEquals(0, start);
            assertEquals(0, bodyKinematicsAdded);
            assertEquals(0, reset);
            assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());
            assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                    ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);
            final var fixedKinematics = new BodyKinematics();

            final var measuredKinematics = new BodyKinematics();
            final var random = new Random();
            for (var i = 0; i < N_SAMPLES; i++) {
                BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, 
                        measuredKinematics);

                kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
                driftEstimator.addBodyKinematics(measuredKinematics);

                assertEquals(i + 1, kalmanDriftEstimator.getNumberOfProcessedSamples());
                assertFalse(kalmanDriftEstimator.isRunning());

                fixer.fix(measuredKinematics, fixedKinematics);
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, navigationFrame, fixedKinematics, 
                        navigationFrame);

                kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
            }

            assertEquals(N_SAMPLES, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());
            assertEquals(1, start);
            assertEquals(N_SAMPLES, bodyKinematicsAdded);
            assertEquals(0, reset);

            final var kalmanState = kalmanEstimator.getState();

            final var navigationPositionDrift = positionDrift(ecefFrame, navigationFrame);
            final var navigationVelocityDrift = velocityDrift(ecefFrame, navigationFrame);
            final var navigationOrientationDrift = orientationDrift(ecefFrame, navigationFrame);

            final var kalmanPositionDrift = positionDrift(ecefFrame, kalmanState);
            final var kalmanVelocityDrift = velocityDrift(ecefFrame, kalmanState);
            final var kalmanOrientationDrift = orientationDrift(ecefFrame, kalmanState);

            final var currentPositionDrift1 = kalmanDriftEstimator.getCurrentPositionDrift();
            final var currentPositionDrift2 = new ECEFPosition();
            assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
            assertEquals(currentPositionDrift1, currentPositionDrift2);

            final var currentVelocityDrift1 = kalmanDriftEstimator.getCurrentVelocityDrift();
            final var currentVelocityDrift2 = new ECEFVelocity();
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
            assertEquals(currentVelocityDrift1, currentVelocityDrift2);

            final var currentOrientationDrift1 = kalmanDriftEstimator.getCurrentOrientationDrift();
            final var currentOrientationDrift2 = new Quaternion();
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
            assertEquals(currentOrientationDrift1, currentOrientationDrift2);

            final var distanceFormatter = new DistanceFormatter();
            final var speedFormatter = new SpeedFormatter();
            final var angleFormatter = new AngleFormatter();
            final var accelerationFormatter = new AccelerationFormatter();
            final var angularSpeedFormatter = new AngularSpeedFormatter();

            final var currentPositionDriftNorm = kalmanDriftEstimator.getCurrentPositionDriftNormMeters();
            assertNotNull(currentPositionDriftNorm);
            assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(), 0.0);
            LOGGER.log(Level.INFO, String.format("Current position drift: %s", distanceFormatter.format(
                    currentPositionDriftNorm, DistanceUnit.METER)));
            if (currentPositionDriftNorm >= 92.0) {
                continue;
            }
            assertTrue(currentPositionDriftNorm < 92.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentPositionDriftNorm < navigationPositionDrift);
            assertEquals(currentPositionDriftNorm, kalmanPositionDrift, ABSOLUTE_ERROR);

            final var currentPositionDriftNorm1 = kalmanDriftEstimator.getCurrentPositionDriftNorm();
            assertEquals(currentPositionDriftNorm, currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
            final var currentPositionDriftNorm2 = new Distance(1.0, DistanceUnit.FOOT);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(currentPositionDriftNorm2));
            assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

            final var currentVelocityDriftNorm = kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond();
            assertNotNull(currentVelocityDriftNorm);
            assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(), 0.0);
            LOGGER.log(Level.INFO, String.format("Current velocity drift: %s", speedFormatter.format(
                    currentVelocityDriftNorm, SpeedUnit.METERS_PER_SECOND)));
            if (currentVelocityDriftNorm >= 6.0) {
                continue;
            }
            assertTrue(currentVelocityDriftNorm < 6.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
            assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift, ABSOLUTE_ERROR);

            final var currentVelocityDriftNorm1 = kalmanDriftEstimator.getCurrentVelocityDriftNorm();
            assertEquals(currentVelocityDriftNorm, currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, currentVelocityDriftNorm1.getUnit());
            final var currentVelocityDriftNorm2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(currentVelocityDriftNorm2));
            assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

            final var currentOrientationDriftNorm = kalmanDriftEstimator.getCurrentOrientationDriftRadians();
            assertNotNull(currentOrientationDriftNorm);
            assertEquals(currentOrientationDriftNorm, currentOrientationDrift1.getRotationAngle(), 0.0);
            LOGGER.log(Level.INFO, String.format("Current orientation drift: %s", angleFormatter.format(
                    AngleConverter.convert(currentOrientationDriftNorm, AngleUnit.RADIANS, AngleUnit.DEGREES),
                    AngleUnit.DEGREES)));
            if (currentOrientationDriftNorm >= 0.06) {
                continue;
            }
            assertTrue(currentOrientationDriftNorm < 0.06);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
            assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift, ABSOLUTE_ERROR);

            final var currentOrientationDriftNorm1 = kalmanDriftEstimator.getCurrentOrientationDriftAngle();
            assertEquals(currentOrientationDriftNorm, currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
            final var currentOrientationDriftNorm2 = new Angle(1.0, AngleUnit.DEGREES);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(currentOrientationDriftNorm2));
            assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

            final var currentPositionDriftPerTimeUnit = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
            assertNotNull(currentPositionDriftPerTimeUnit);
            LOGGER.log(Level.INFO, String.format("Current position drift per time unit: %s", speedFormatter.format(
                    currentPositionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)));
            if (currentPositionDriftPerTimeUnit >= 1.75) {
                continue;
            }
            assertTrue(currentPositionDriftPerTimeUnit < 1.75);
            assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftNorm 
                    / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

            final var currentPositionDriftPerTimeUnit1 = 
                    kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
            assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, currentPositionDriftPerTimeUnit1.getUnit());
            final var currentPositionDriftPerTimeUnit2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(currentPositionDriftPerTimeUnit2));
            assertEquals(currentPositionDriftPerTimeUnit1, currentPositionDriftPerTimeUnit2);

            final var currentVelocityDriftPerTimeUnit = kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
            assertNotNull(currentVelocityDriftPerTimeUnit);
            LOGGER.log(Level.INFO, String.format("Current velocity drift per time unit: %s",
                    accelerationFormatter.format(currentVelocityDriftPerTimeUnit,
                            AccelerationUnit.METERS_PER_SQUARED_SECOND)));
            if (currentVelocityDriftPerTimeUnit >= 0.14) {
                continue;
            }
            assertTrue(currentVelocityDriftPerTimeUnit < 0.14);
            assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftNorm 
                    / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

            final var currentVelocityDriftPerTimeUnit1 = 
                    kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
            assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, currentVelocityDriftPerTimeUnit1.getUnit());
            final var currentVelocityDriftPerTimeUnit2 = new Acceleration(1.0, 
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                    currentVelocityDriftPerTimeUnit2));
            assertEquals(currentVelocityDriftPerTimeUnit1, currentVelocityDriftPerTimeUnit2);

            final var currentOrientationDriftPerTimeUnit = kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
            assertNotNull(currentOrientationDriftPerTimeUnit);
            LOGGER.log(Level.INFO, String.format("Current orientation drift per time unit: %s",
                    angularSpeedFormatter.format(AngularSpeedConverter.convert(currentOrientationDriftPerTimeUnit,
                                    AngularSpeedUnit.RADIANS_PER_SECOND, AngularSpeedUnit.DEGREES_PER_SECOND),
                            AngularSpeedUnit.DEGREES_PER_SECOND)));
            if (currentOrientationDriftPerTimeUnit >= 1.7e-3) {
                continue;
            }
            assertTrue(currentOrientationDriftPerTimeUnit < 1.7e-3);
            assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftNorm 
                    / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

            final var currentOrientationDriftPerTimeUnit1 = 
                    kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
            assertEquals(currentOrientationDriftPerTimeUnit, 
                    currentOrientationDriftPerTimeUnit1.getValue().doubleValue(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, currentOrientationDriftPerTimeUnit1.getUnit());
            final var currentOrientationDriftPerTimeUnit2 = new AngularSpeed(1.0, 
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                    currentOrientationDriftPerTimeUnit2));
            assertEquals(currentOrientationDriftPerTimeUnit1, currentOrientationDriftPerTimeUnit2);

            // reset
            kalmanDriftEstimator.reset();

            assertEquals(1, reset);
            assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Test
    void testAddBodyKinematicsAndResetApproximateCalibrationAndEstimatedKalmanConfig() throws AlgebraException,
            InvalidSourceAndDestinationFrameTypeException, LockedException, InertialNavigatorException,
            DriftEstimationException, NotReadyException, InvalidRotationMatrixException, RotationException,
            INSException, IntervalDetectorThresholdFactorOptimizerException, RandomWalkEstimationException {

        var numValid = 0;
        for (var t = 0; t < TIMES; t++) {
            final var ba = generateBa();
            final var ma = generateMaCommonAxis();
            final var bg = generateBg();
            final var mg = generateMg();
            final var gg =  new Matrix(3, 3);

            final var accelNoiseRootPSD = getAccelNoiseRootPSD();
            final var gyroNoiseRootPSD = getGyroNoiseRootPSD();
            final var accelQuantLevel = 0.0;
            final var gyroQuantLevel = 0.0;

            final var nedFrame = generateFrame();
            final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

            final var nedPosition = nedFrame.getPosition();
            final var nedC = nedFrame.getCoordinateTransformation();

            final var ecefPosition = ecefFrame.getECEFPosition();
            final var ecefC = ecefFrame.getCoordinateTransformation();

            final var accelerometerCalibrator = new KnownGravityNormAccelerometerCalibrator();
            final var gyroscopeCalibrator = new EasyGyroscopeCalibrator();

            final var generator = new AccelerometerAndGyroscopeMeasurementsGenerator();

            // initialize a threshold optimizer to attempt calibration by generating
            // timed body kinematics with noise and attempting to find the best
            // threshold to find optimal calibration
            final var optimizer = buildOptimizer(ba, ma, accelNoiseRootPSD, gyroNoiseRootPSD, accelerometerCalibrator,
                    gyroscopeCalibrator, new NEDFrame(nedFrame), new ECEFFrame(ecefFrame), generator);
            final var thresholdFactor = optimizer.optimize();

            assertEquals(thresholdFactor, optimizer.getOptimalThresholdFactor(), 0.0);

            final var estimatedBa = Matrix.newFromArray(optimizer.getEstimatedAccelerometerBiases());
            final var estimatedMa = optimizer.getEstimatedAccelerometerMa();

            final var estimatedBg = Matrix.newFromArray(optimizer.getEstimatedGyroscopeBiases());
            final var estimatedMg = optimizer.getEstimatedGyroscopeMg();
            final var estimatedGg = optimizer.getEstimatedGyroscopeGg();

            // clear all previously generated data
            timedBodyKinematics.clear();

            final var randomWalkEstimator = new RandomWalkEstimator();
            randomWalkEstimator.setNedPositionAndNedOrientation(nedPosition, nedC);
            randomWalkEstimator.setAccelerationBias(estimatedBa);
            randomWalkEstimator.setAccelerationCrossCouplingErrors(estimatedMa);
            randomWalkEstimator.setAngularSpeedBias(estimatedBg);
            randomWalkEstimator.setAngularSpeedCrossCouplingErrors(estimatedMg);
            randomWalkEstimator.setAngularSpeedGDependantCrossBias(estimatedGg);
            randomWalkEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                    gyroQuantLevel);

            // this is the true body kinematics that should be measured on a perfect
            // sensor if there were no noise or calibration errors at current position
            // and orientations
            final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS, 
                    ecefC, ecefC, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ecefPosition);

            final var measuredKinematics = new BodyKinematics();
            final var random = new Random();
            for (var i = 0; i < N_SAMPLES; i++) {
                BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, 
                        measuredKinematics);

                randomWalkEstimator.addBodyKinematics(measuredKinematics);

                assertEquals(i + 1, randomWalkEstimator.getNumberOfProcessedSamples());
                assertFalse(randomWalkEstimator.isRunning());
            }


            final var kalmanConfigCreator = new INSLooselyCoupledKalmanConfigCreator(generator, randomWalkEstimator);
            final var kalmanConfig = kalmanConfigCreator.create();

            final var initConfigCreator = new INSLooselyCoupledKalmanInitializerConfigCreator(accelerometerCalibrator, 
                    gyroscopeCalibrator, randomWalkEstimator);
            final var initConfig = initConfigCreator.create();

            // use real calibration values to generate measurements with errors
            final var navigationFrame = new ECEFFrame(ecefFrame);
            final var fixer = new BodyKinematicsFixer();
            // use estimated calibration data to fix measurements
            fixer.setAccelerationBias(estimatedBa);
            fixer.setAccelerationCrossCouplingErrors(estimatedMa);
            fixer.setAngularSpeedBias(estimatedBg);
            fixer.setAngularSpeedCrossCouplingErrors(estimatedMg);
            fixer.setAngularSpeedGDependantCrossBias(estimatedGg);

            final var kalmanDriftEstimator = new KalmanDriftEstimator(nedFrame, estimatedBa, estimatedMa, estimatedBg,
                    estimatedMg, estimatedGg, kalmanConfig, initConfig, this);
            kalmanDriftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final var driftEstimator = new DriftEstimator(nedFrame, estimatedBa, estimatedMa, estimatedBg, estimatedMg,
                    estimatedGg);
            driftEstimator.setTimeInterval(TIME_INTERVAL_SECONDS);

            final var kalmanEstimator = new INSLooselyCoupledKalmanFilteredEstimator(kalmanConfig, initConfig,
                    new ECEFFrame(ecefFrame));

            reset();
            assertEquals(0, start);
            assertEquals(0, bodyKinematicsAdded);
            assertEquals(0, reset);
            assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());
            assertTrue(kalmanDriftEstimator.isFixKinematicsEnabled());

            final var fixedKinematics = new BodyKinematics();
            for (var i = 0; i < N_SAMPLES; i++) {
                BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics, errors, random, 
                        measuredKinematics);

                kalmanDriftEstimator.addBodyKinematics(measuredKinematics);
                driftEstimator.addBodyKinematics(measuredKinematics);

                assertEquals(i + 1, kalmanDriftEstimator.getNumberOfProcessedSamples());
                assertFalse(kalmanDriftEstimator.isRunning());

                fixer.fix(measuredKinematics, fixedKinematics);
                ECEFInertialNavigator.navigateECEF(TIME_INTERVAL_SECONDS, navigationFrame, fixedKinematics, 
                        navigationFrame);

                kalmanEstimator.update(fixedKinematics, i * TIME_INTERVAL_SECONDS);
            }

            assertEquals(N_SAMPLES, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertFalse(kalmanDriftEstimator.isRunning());
            assertEquals(1, start);
            assertEquals(N_SAMPLES, bodyKinematicsAdded);
            assertEquals(0, reset);

            final var kalmanState = kalmanEstimator.getState();

            final var navigationPositionDrift = positionDrift(ecefFrame, navigationFrame);
            final var navigationVelocityDrift = velocityDrift(ecefFrame, navigationFrame);
            final var navigationOrientationDrift = orientationDrift(ecefFrame, navigationFrame);

            final var kalmanPositionDrift = positionDrift(ecefFrame, kalmanState);
            final var kalmanVelocityDrift = velocityDrift(ecefFrame, kalmanState);
            final var kalmanOrientationDrift = orientationDrift(ecefFrame, kalmanState);

            final var currentPositionDrift1 = kalmanDriftEstimator.getCurrentPositionDrift();
            final var currentPositionDrift2 = new ECEFPosition();
            assertTrue(kalmanDriftEstimator.getCurrentPositionDrift(currentPositionDrift2));
            assertEquals(currentPositionDrift1, currentPositionDrift2);

            final var currentVelocityDrift1 = kalmanDriftEstimator.getCurrentVelocityDrift();
            final var currentVelocityDrift2 = new ECEFVelocity();
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDrift(currentVelocityDrift2));
            assertEquals(currentVelocityDrift1, currentVelocityDrift2);

            final var currentOrientationDrift1 = kalmanDriftEstimator.getCurrentOrientationDrift();
            final var currentOrientationDrift2 = new Quaternion();
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDrift(currentOrientationDrift2));
            assertEquals(currentOrientationDrift1, currentOrientationDrift2);

            final var distanceFormatter = new DistanceFormatter();
            final var speedFormatter = new SpeedFormatter();
            final var angleFormatter = new AngleFormatter();
            final var accelerationFormatter = new AccelerationFormatter();
            final var angularSpeedFormatter = new AngularSpeedFormatter();

            final var currentPositionDriftNorm = kalmanDriftEstimator.getCurrentPositionDriftNormMeters();
            assertNotNull(currentPositionDriftNorm);
            assertEquals(currentPositionDriftNorm, currentPositionDrift1.getNorm(), 0.0);
            LOGGER.log(Level.INFO, String.format("Current position drift: %s", distanceFormatter.format(
                    currentPositionDriftNorm, DistanceUnit.METER)));
            if (currentPositionDriftNorm >= 92.0) {
                continue;
            }
            assertTrue(currentPositionDriftNorm < 92.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentPositionDriftNorm < navigationPositionDrift);
            assertEquals(currentPositionDriftNorm, kalmanPositionDrift, ABSOLUTE_ERROR);

            final var currentPositionDriftNorm1 = kalmanDriftEstimator.getCurrentPositionDriftNorm();
            assertEquals(currentPositionDriftNorm, currentPositionDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(DistanceUnit.METER, currentPositionDriftNorm1.getUnit());
            final var currentPositionDriftNorm2 = new Distance(1.0, DistanceUnit.FOOT);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftNorm(currentPositionDriftNorm2));
            assertEquals(currentPositionDriftNorm1, currentPositionDriftNorm2);

            final var currentVelocityDriftNorm = kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond();
            assertNotNull(currentVelocityDriftNorm);
            assertEquals(currentVelocityDriftNorm, currentVelocityDrift1.getNorm(), 0.0);
            LOGGER.log(Level.INFO, String.format("Current velocity drift: %s", speedFormatter.format(
                    currentVelocityDriftNorm, SpeedUnit.METERS_PER_SECOND)));
            if (currentVelocityDriftNorm >= 6.0) {
                continue;
            }
            assertTrue(currentVelocityDriftNorm < 6.0);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentVelocityDriftNorm < navigationVelocityDrift);
            assertEquals(currentVelocityDriftNorm, kalmanVelocityDrift, ABSOLUTE_ERROR);

            final var currentVelocityDriftNorm1 = kalmanDriftEstimator.getCurrentVelocityDriftNorm();
            assertEquals(currentVelocityDriftNorm, currentVelocityDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, currentVelocityDriftNorm1.getUnit());
            final var currentVelocityDriftNorm2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftNorm(currentVelocityDriftNorm2));
            assertEquals(currentVelocityDriftNorm1, currentVelocityDriftNorm2);

            final var currentOrientationDriftNorm = kalmanDriftEstimator.getCurrentOrientationDriftRadians();
            assertNotNull(currentOrientationDriftNorm);
            assertEquals(currentOrientationDriftNorm, currentOrientationDrift1.getRotationAngle(), 0.0);
            LOGGER.log(Level.INFO, String.format("Current orientation drift: %s", angleFormatter.format(
                    AngleConverter.convert(currentOrientationDriftNorm, AngleUnit.RADIANS, AngleUnit.DEGREES),
                    AngleUnit.DEGREES)));
            if (currentOrientationDriftNorm >= 0.06) {
                continue;
            }
            assertTrue(currentOrientationDriftNorm < 0.06);
            // Check that kalman corrected drift is smaller than normal navigation drift
            assertTrue(currentOrientationDriftNorm < navigationOrientationDrift);
            assertEquals(currentOrientationDriftNorm, kalmanOrientationDrift, ABSOLUTE_ERROR);

            final var currentOrientationDriftNorm1 = kalmanDriftEstimator.getCurrentOrientationDriftAngle();
            assertEquals(currentOrientationDriftNorm, currentOrientationDriftNorm1.getValue().doubleValue(), 0.0);
            assertEquals(AngleUnit.RADIANS, currentOrientationDriftNorm1.getUnit());
            final var currentOrientationDriftNorm2 = new Angle(1.0, AngleUnit.DEGREES);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftAngle(currentOrientationDriftNorm2));
            assertEquals(currentOrientationDriftNorm1, currentOrientationDriftNorm2);

            final var currentPositionDriftPerTimeUnit = kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit();
            assertNotNull(currentPositionDriftPerTimeUnit);
            LOGGER.log(Level.INFO, String.format("Current position drift per time unit: %s", speedFormatter.format(
                    currentPositionDriftPerTimeUnit, SpeedUnit.METERS_PER_SECOND)));
            if (currentPositionDriftPerTimeUnit >= 1.75) {
                continue;
            }
            assertTrue(currentPositionDriftPerTimeUnit < 1.75);
            assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftNorm 
                    / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

            final var currentPositionDriftPerTimeUnit1 = 
                    kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed();
            assertEquals(currentPositionDriftPerTimeUnit, currentPositionDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(SpeedUnit.METERS_PER_SECOND, currentPositionDriftPerTimeUnit1.getUnit());
            final var currentPositionDriftPerTimeUnit2 = new Speed(1.0, SpeedUnit.KILOMETERS_PER_HOUR);
            assertTrue(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(currentPositionDriftPerTimeUnit2));
            assertEquals(currentPositionDriftPerTimeUnit1, currentPositionDriftPerTimeUnit2);

            final var currentVelocityDriftPerTimeUnit = kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit();
            assertNotNull(currentVelocityDriftPerTimeUnit);
            LOGGER.log(Level.INFO, String.format("Current velocity drift per time unit: %s",
                    accelerationFormatter.format(currentVelocityDriftPerTimeUnit, 
                            AccelerationUnit.METERS_PER_SQUARED_SECOND)));
            if (currentVelocityDriftPerTimeUnit >= 0.14) {
                continue;
            }
            assertTrue(currentVelocityDriftPerTimeUnit < 0.14);
            assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftNorm 
                    / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

            final var currentVelocityDriftPerTimeUnit1 = 
                    kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration();
            assertEquals(currentVelocityDriftPerTimeUnit, currentVelocityDriftPerTimeUnit1.getValue().doubleValue(),
                    0.0);
            assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, currentVelocityDriftPerTimeUnit1.getUnit());
            final var currentVelocityDriftPerTimeUnit2 = new Acceleration(1.0, 
                    AccelerationUnit.FEET_PER_SQUARED_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(
                    currentVelocityDriftPerTimeUnit2));
            assertEquals(currentVelocityDriftPerTimeUnit1, currentVelocityDriftPerTimeUnit2);

            final var currentOrientationDriftPerTimeUnit = kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit();
            assertNotNull(currentOrientationDriftPerTimeUnit);
            LOGGER.log(Level.INFO, String.format("Current orientation drift per time unit: %s",
                    angularSpeedFormatter.format(AngularSpeedConverter.convert(currentOrientationDriftPerTimeUnit,
                            AngularSpeedUnit.RADIANS_PER_SECOND, AngularSpeedUnit.DEGREES_PER_SECOND),
                            AngularSpeedUnit.DEGREES_PER_SECOND)));
            if (currentOrientationDriftPerTimeUnit >= 1.7e-3) {
                continue;
            }
            assertTrue(currentOrientationDriftPerTimeUnit < 1.7e-3);
            assertEquals(currentOrientationDriftPerTimeUnit, currentOrientationDriftNorm 
                    / (N_SAMPLES * TIME_INTERVAL_SECONDS), ABSOLUTE_ERROR);

            final var currentOrientationDriftPerTimeUnit1 = 
                    kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed();
            assertEquals(currentOrientationDriftPerTimeUnit, 
                    currentOrientationDriftPerTimeUnit1.getValue().doubleValue(), 0.0);
            assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, currentOrientationDriftPerTimeUnit1.getUnit());
            final var currentOrientationDriftPerTimeUnit2 = new AngularSpeed(1.0, 
                    AngularSpeedUnit.DEGREES_PER_SECOND);
            assertTrue(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(
                    currentOrientationDriftPerTimeUnit2));
            assertEquals(currentOrientationDriftPerTimeUnit1, currentOrientationDriftPerTimeUnit2);

            // reset
            kalmanDriftEstimator.reset();

            assertEquals(1, reset);
            assertEquals(0, kalmanDriftEstimator.getNumberOfProcessedSamples());
            assertNull(kalmanDriftEstimator.getCurrentPositionDrift());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDrift());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDrift());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDrift(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNormMeters());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNormMetersPerSecond());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftNorm());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftNorm(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftRadians());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftAngle());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftAngle(null));
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentPositionDriftPerTimeUnitAsSpeed(null));
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration());
            assertFalse(kalmanDriftEstimator.getCurrentVelocityDriftPerTimeUnitAsAcceleration(null));
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnit());
            assertNull(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed());
            assertFalse(kalmanDriftEstimator.getCurrentOrientationDriftPerTimeUnitAsAngularSpeed(null));

            numValid++;
            break;
        }

        assertTrue(numValid > 0);
    }

    @Override
    public void onStart(final DriftEstimator estimator) {
        checkLocked((KalmanDriftEstimator) estimator);
        start++;
    }

    @Override
    public void onBodyKinematicsAdded(
            final DriftEstimator estimator, final BodyKinematics measuredKinematics,
            final BodyKinematics fixedKinematics) {
        if (bodyKinematicsAdded == 0) {
            checkLocked((KalmanDriftEstimator) estimator);
        }
        bodyKinematicsAdded++;
    }

    @Override
    public void onReset(final DriftEstimator estimator) {
        checkLocked((KalmanDriftEstimator) estimator);
        reset++;
    }

    private void reset() {
        start = 0;
        bodyKinematicsAdded = 0;
        reset = 0;
    }

    private static void checkLocked(final KalmanDriftEstimator estimator) {
        assertTrue(estimator.isRunning());
        assertThrows(LockedException.class, () -> estimator.setListener(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceFrame(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceNedFrame(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceEcefPosition(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceEcefVelocity(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceEcefCoordinateTransformation(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceNedPosition(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceNedVelocity(null));
        assertThrows(LockedException.class, () -> estimator.setReferenceNedCoordinateTransformation(null));
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
        assertThrows(LockedException.class, () -> estimator.setAccelerationSz(0.0));
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
        assertThrows(LockedException.class, () -> estimator.setFixKinematicsEnabled(false));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(0.0));
        assertThrows(LockedException.class, () -> estimator.setTimeInterval(new Time(1.0, TimeUnit.SECOND)));
        assertThrows(LockedException.class, () -> estimator.setKalmanConfig(null));
        assertThrows(LockedException.class, () -> estimator.setInitConfig(null));
    }

    private BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer buildOptimizer(
            final Matrix ba, final Matrix ma, final double accelNoiseRootPSD, final double gyroNoiseRootPSD,
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator,
            final EasyGyroscopeCalibrator gyroscopeCalibrator) throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, WrongSizeException, LockedException, RotationException {

        final var nedFrame = generateFrame();
        final var ecefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(nedFrame);

        final var generator = new AccelerometerAndGyroscopeMeasurementsGenerator();

        return buildOptimizer(ba, ma, accelNoiseRootPSD, gyroNoiseRootPSD, accelerometerCalibrator, gyroscopeCalibrator,
                nedFrame, ecefFrame, generator);
    }

    private BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer buildOptimizer(
            final Matrix ba, final Matrix ma, final double accelNoiseRootPSD, final double gyroNoiseRootPSD,
            final KnownGravityNormAccelerometerCalibrator accelerometerCalibrator,
            final EasyGyroscopeCalibrator gyroscopeCalibrator, final NEDFrame nedFrame, final ECEFFrame ecefFrame,
            final AccelerometerAndGyroscopeMeasurementsGenerator generator)
            throws InvalidSourceAndDestinationFrameTypeException, InvalidRotationMatrixException, WrongSizeException,
            LockedException, RotationException {

        timedBodyKinematics.clear();

        // generate measurements

        final var numSequences = EasyGyroscopeCalibrator.MINIMUM_SEQUENCES_COMMON_Z_AXIS;
        final var numMeasurements = KnownGravityNormAccelerometerCalibrator.MINIMUM_MEASUREMENTS_GENERAL;
        generateBodyKinematics(nedFrame, ecefFrame, false, ma, accelNoiseRootPSD, gyroNoiseRootPSD,
                numSequences, numMeasurements);

        // we only use the generator at this point to get an estimated average of
        // initial gyroscope bias (but we could skip this as well and probably get
        //  similar gyroscope calibration accuracy).

        for (final var kinematics : timedBodyKinematics) {
            assertTrue(generator.process(kinematics));
        }

        // As an initial value for gyroscope bias, we can use the average
        // gyroscope values during initialization. A more accurate initial
        // guess for bias could be obtained by using leveling with magnetometer
        // and accelerometer readings (once both magnetometer and accelerometer
        // are calibrated).
        final var initialAvgAngularSpeed = generator.getInitialAvgAngularSpeedTriad();
        final var initialBg = initialAvgAngularSpeed.getValuesAsMatrix();

        final var gravity = ECEFGravityEstimator.estimateGravityAndReturnNew(ecefFrame);

        // configure calibrators and data source
        final var initialBa = new Matrix(3, 1);
        final var initialMa = new Matrix(3, 3);
        accelerometerCalibrator.setGroundTruthGravityNorm(gravity.getNorm());
        accelerometerCalibrator.setCommonAxisUsed(true);
        accelerometerCalibrator.setInitialBias(initialBa);
        accelerometerCalibrator.setInitialMa(initialMa);

        final var initialMg = new Matrix(3, 3);
        final var initialGg = new Matrix(3, 3);
        gyroscopeCalibrator.setCommonAxisUsed(true);
        gyroscopeCalibrator.setGDependentCrossBiasesEstimated(false);
        gyroscopeCalibrator.setInitialBias(initialBg);
        gyroscopeCalibrator.setInitialMg(initialMg);
        gyroscopeCalibrator.setInitialGg(initialGg);
        gyroscopeCalibrator.setAccelerometerBias(ba);
        gyroscopeCalibrator.setAccelerometerMa(ma);

        // create optimizer
        return new BracketedAccelerometerAndGyroscopeIntervalDetectorThresholdFactorOptimizer(dataSource,
                accelerometerCalibrator, gyroscopeCalibrator);
    }

    @SuppressWarnings("SameParameterValue")
    private void generateBodyKinematics(
            final NEDFrame nedFrame, final ECEFFrame ecefFrame, final boolean changePosition, final Matrix ma,
            final double accelNoiseRootPSD, final double gyroNoiseRootPSD, final int numSequences,
            final int numMeasurements) throws WrongSizeException, InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {

        final var ba = generateBa();
        final var bg = generateBg();
        final var mg = generateMg();
        final var gg = generateGg();

        final var accelQuantLevel = 0.0;
        final var gyroQuantLevel = 0.0;

        final var errors = new IMUErrors(ba, bg, ma, mg, gg, accelNoiseRootPSD, gyroNoiseRootPSD, accelQuantLevel,
                gyroQuantLevel);

        final var randomizer = new UniformRandomizer();

        // compute ground-truth kinematics that should be generated at provided
        // position, velocity and orientation
        final var trueKinematics = ECEFKinematicsEstimator.estimateKinematicsAndReturnNew(TIME_INTERVAL_SECONDS,
                ecefFrame, ecefFrame);

        // generate initial static samples
        final var initialStaticSamples = TriadStaticIntervalDetector.DEFAULT_INITIAL_STATIC_SAMPLES;
        final var random = new Random();
        generateStaticSamples(initialStaticSamples, trueKinematics, errors, random, 0);

        final var n = Math.max(numSequences + 1, numMeasurements);

        final var staticPeriodLength = 3 * TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;
        final var dynamicPeriodLength = TriadStaticIntervalDetector.DEFAULT_WINDOW_SIZE;

        var startSample = initialStaticSamples;
        for (var i = 0; i < n; i++) {
            // generate static samples
            generateStaticSamples(staticPeriodLength, trueKinematics, errors, random, startSample);
            startSample += staticPeriodLength;

            // generate dynamic samples
            generateDynamicSamples(dynamicPeriodLength, trueKinematics, randomizer, ecefFrame, nedFrame, errors, random,
                    startSample, changePosition);
            startSample += dynamicPeriodLength;
        }
    }

    private static NEDFrame generateFrame() throws InvalidSourceAndDestinationFrameTypeException {
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

        return new NEDFrame(nedPosition, nedC);
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

    private static Matrix generateMaCommonAxis() throws WrongSizeException {
        final var result = new Matrix(3, 3);
        result.fromArray(new double[]{
                500e-6, -300e-6, 200e-6,
                0.0, -600e-6, 250e-6,
                0.0, 0.0, 450e-6
        }, false);

        return result;
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

    private void generateStaticSamples(
            final int numSamples, final BodyKinematics trueKinematics, final IMUErrors errors, final Random random,
            final int startSample) {

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {

            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random);

            final var timedMeasuredKinematics = new TimedBodyKinematics();
            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(j * TIME_INTERVAL_SECONDS);

            timedBodyKinematics.add(timedMeasuredKinematics);
        }
    }

    @SuppressWarnings("SameParameterValue")
    private void generateDynamicSamples(
            final int numSamples, final BodyKinematics trueKinematics, final UniformRandomizer randomizer,
            final ECEFFrame ecefFrame, final NEDFrame nedFrame, final IMUErrors errors, final Random random,
            final int startSample, final boolean changePosition) throws InvalidSourceAndDestinationFrameTypeException,
            InvalidRotationMatrixException, RotationException {

        final var sqrtTimeInterval = Math.sqrt(TIME_INTERVAL_SECONDS);
        final var specificForceStandardDeviation = getAccelNoiseRootPSD() / sqrtTimeInterval;
        final var angularRateStandardDeviation = getGyroNoiseRootPSD() / sqrtTimeInterval;

        final var deltaX = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaY = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;
        final var deltaZ = changePosition ? randomizer.nextDouble(MIN_DELTA_POS_METERS, MAX_DELTA_POS_METERS) : 0.0;

        final var deltaRoll = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaPitch = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));
        final var deltaYaw = Math.toRadians(randomizer.nextDouble(MIN_DELTA_ANGLE_DEGREES, MAX_DELTA_ANGLE_DEGREES));

        final var ecefX = ecefFrame.getX();
        final var ecefY = ecefFrame.getY();
        final var ecefZ = ecefFrame.getZ();

        final var nedC = nedFrame.getCoordinateTransformation();

        final var roll = nedC.getRollEulerAngle();
        final var pitch = nedC.getPitchEulerAngle();
        final var yaw = nedC.getYawEulerAngle();

        final var beforeQ = new Quaternion();
        nedC.asRotation(beforeQ);

        final var oldNedFrame = new NEDFrame(nedFrame);
        final var newNedFrame = new NEDFrame();
        final var oldEcefFrame = new ECEFFrame(ecefFrame);
        final var newEcefFrame = new ECEFFrame();

        var oldEcefX = ecefX - deltaX;
        var oldEcefY = ecefY - deltaY;
        var oldEcefZ = ecefZ - deltaZ;
        var oldRoll = roll - deltaRoll;
        var oldPitch = pitch - deltaPitch;
        var oldYaw = yaw - deltaYaw;

        final var measuredBeforeGravityKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS,
                trueKinematics, errors, random);
        final var beforeMeanFx = measuredBeforeGravityKinematics.getFx();
        final var beforeMeanFy = measuredBeforeGravityKinematics.getFy();
        final var beforeMeanFz = measuredBeforeGravityKinematics.getFz();

        final var sequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
        sequence.setBeforeMeanSpecificForceCoordinates(beforeMeanFx, beforeMeanFy, beforeMeanFz);

        final var trueSequence = new BodyKinematicsSequence<StandardDeviationTimedBodyKinematics>();
        final var trueTimedKinematicsList = new ArrayList<StandardDeviationTimedBodyKinematics>();

        for (int i = 0, j = startSample; i < numSamples; i++, j++) {
            final var progress = (double) i / (double) numSamples;

            final var newRoll = oldRoll + interpolate(deltaRoll, progress);
            final var newPitch = oldPitch + interpolate(deltaPitch, progress);
            final var newYaw = oldYaw + interpolate(deltaYaw, progress);
            final var newNedC = new CoordinateTransformation(newRoll, newPitch, newYaw, FrameType.BODY_FRAME,
                    FrameType.LOCAL_NAVIGATION_FRAME);
            final var newNedPosition = oldNedFrame.getPosition();

            newNedFrame.setPosition(newNedPosition);
            newNedFrame.setCoordinateTransformation(newNedC);

            NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

            final var newEcefX = oldEcefX + interpolate(deltaX, progress);
            final var newEcefY = oldEcefY + interpolate(deltaY, progress);
            final var newEcefZ = oldEcefZ + interpolate(deltaZ, progress);

            newEcefFrame.setCoordinates(newEcefX, newEcefY, newEcefZ);

            ECEFtoNEDFrameConverter.convertECEFtoNED(newEcefFrame, newNedFrame);

            final var timestampSeconds = j * TIME_INTERVAL_SECONDS;

            // update true kinematics using new position and rotation
            ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, oldEcefFrame,
                    trueKinematics);

            // add error to true kinematics
            final var measuredKinematics = BodyKinematicsGenerator.generate(TIME_INTERVAL_SECONDS, trueKinematics,
                    errors, random);

            final var timedMeasuredKinematics = new TimedBodyKinematics();
            timedMeasuredKinematics.setKinematics(measuredKinematics);
            timedMeasuredKinematics.setTimestampSeconds(timestampSeconds);

            timedBodyKinematics.add(timedMeasuredKinematics);

            final var trueTimedKinematics = new StandardDeviationTimedBodyKinematics(new BodyKinematics(trueKinematics),
                    timestampSeconds, specificForceStandardDeviation, angularRateStandardDeviation);
            trueTimedKinematicsList.add(trueTimedKinematics);

            oldNedFrame.copyFrom(newNedFrame);
            oldEcefFrame.copyFrom(newEcefFrame);
            oldRoll = newRoll;
            oldPitch = newPitch;
            oldYaw = newYaw;
            oldEcefX = newEcefX;
            oldEcefY = newEcefY;
            oldEcefZ = newEcefZ;
        }

        trueSequence.setItems(trueTimedKinematicsList);

        final var afterQ = new Quaternion();
        QuaternionIntegrator.integrateGyroSequence(trueSequence, beforeQ, QuaternionStepIntegratorType.RUNGE_KUTTA,
                afterQ);

        final var newNedC = new CoordinateTransformation(afterQ.asInhomogeneousMatrix(), FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME);
        newNedFrame.setCoordinateTransformation(newNedC);

        NEDtoECEFFrameConverter.convertNEDtoECEF(newNedFrame, newEcefFrame);

        // update current ECEF and NED frames
        ecefFrame.copyFrom(newEcefFrame);
        nedFrame.copyFrom(newNedFrame);

        // after the dynamic sequence finishes, update true kinematics for a
        // static sequence at the current frame
        ECEFKinematicsEstimator.estimateKinematics(TIME_INTERVAL_SECONDS, newEcefFrame, newEcefFrame, trueKinematics);
    }

    // This is required to simulate a smooth transition of values during
    // the dynamic period, to avoid a sudden rotation or translation and simulate
    // a more natural behavior.
    private static double interpolate(final double value, final double progress) {
        return -2.0 * (Math.abs(progress - 0.5) - 0.5) * value;
    }

    private static double positionDrift(final ECEFFrame frame1, final ECEFFrame frame2) {
        final var x1 = frame1.getX();
        final var y1 = frame1.getY();
        final var z1 = frame1.getZ();

        final var x2 = frame2.getX();
        final var y2 = frame2.getY();
        final var z2 = frame2.getZ();

        final var diffX = x2 - x1;
        final var diffY = y2 - y1;
        final var diffZ = z2 - z1;

        final var position = new ECEFPosition(diffX, diffY, diffZ);
        return position.getNorm();
    }

    private static double velocityDrift(final ECEFFrame frame1, final ECEFFrame frame2) {
        final var vx1 = frame1.getVx();
        final var vy1 = frame1.getVy();
        final var vz1 = frame1.getVz();

        final var vx2 = frame2.getVx();
        final var vy2 = frame2.getVy();
        final var vz2 = frame2.getVz();

        final var diffVx = vx2 - vx1;
        final var diffVy = vy2 - vy1;
        final var diffVz = vz2 - vz1;

        final var velocity = new ECEFVelocity(diffVx, diffVy, diffVz);
        return velocity.getNorm();
    }

    private static double orientationDrift(final ECEFFrame frame1, final ECEFFrame frame2)
            throws InvalidRotationMatrixException {
        final var c1 = frame1.getCoordinateTransformationMatrix();
        final var c2 = frame2.getCoordinateTransformationMatrix();

        final var q1 = new Quaternion();
        q1.fromMatrix(c1);
        final var q2 = new Quaternion();
        q2.fromMatrix(c2);

        final var invQ1 = q1.inverseAndReturnNew();
        final var diffQ = q2.combineAndReturnNew(invQ1);
        return diffQ.getRotationAngle();
    }

    private static double positionDrift(final ECEFFrame frame, final INSLooselyCoupledKalmanState state) {
        final var x1 = frame.getX();
        final var y1 = frame.getY();
        final var z1 = frame.getZ();

        final var x2 = state.getX();
        final var y2 = state.getY();
        final var z2 = state.getZ();

        final var diffX = x2 - x1;
        final var diffY = y2 - y1;
        final var diffZ = z2 - z1;

        final var position = new ECEFPosition(diffX, diffY, diffZ);
        return position.getNorm();
    }

    private static double velocityDrift(final ECEFFrame frame, final INSLooselyCoupledKalmanState state) {
        final var vx1 = frame.getVx();
        final var vy1 = frame.getVy();
        final var vz1 = frame.getVz();

        final var vx2 = state.getVx();
        final var vy2 = state.getVy();
        final var vz2 = state.getVz();

        final var diffVx = vx2 - vx1;
        final var diffVy = vy2 - vy1;
        final var diffVz = vz2 - vz1;

        final var velocity = new ECEFVelocity(diffVx, diffVy, diffVz);
        return velocity.getNorm();
    }

    private double orientationDrift(final ECEFFrame frame, final INSLooselyCoupledKalmanState state)
            throws InvalidRotationMatrixException {
        final var c1 = frame.getCoordinateTransformationMatrix();
        final var c2 = state.getBodyToEcefCoordinateTransformationMatrix();

        final var q1 = new Quaternion();
        q1.fromMatrix(c1);
        final var q2 = new Quaternion();
        q2.fromMatrix(c2);

        final var invQ1 = q1.inverseAndReturnNew();
        final var diffQ = q2.combineAndReturnNew(invQ1);
        return diffQ.getRotationAngle();
    }

    private INSLooselyCoupledKalmanConfig generateKalmanConfig() {
        // deg^2 per hour converted to rad^2/s
        final var gyroNoisePsd = Math.pow(0.02 * DEG_TO_RAD / 60.0, 2.0);
        //micro-g^2 per Hz converted to m^2*s^-3
        final var accelerometerNoisePsd = Math.pow(200 * MICRO_G_TO_METERS_PER_SECOND_SQUARED, 2.0);
        final var accelerometerBiasPsd = 1e-7; // m^2*s^-5
        final var gyroBiasPsd = 2e-12; // rad^2 * s^-3
        final var positionNoiseSd = 2.5; // m
        final var velocityNoiseSd = 0.1; // m/s

        return new INSLooselyCoupledKalmanConfig(gyroNoisePsd, accelerometerNoisePsd, accelerometerBiasPsd, gyroBiasPsd,
                positionNoiseSd, velocityNoiseSd);
    }

    private static INSLooselyCoupledKalmanInitializerConfig generateInitConfig() {
        final var initialAttitudeUncertainty = Math.toRadians(1.0); // 1º
        final var initialVelocityUncertainty = 0.1; // m/s
        final var initialPositionUncertainty = 10.0; //m
        final var initialAccelerationBiasUncertainty = 1000 * MICRO_G_TO_METERS_PER_SECOND_SQUARED;
        final var initialGyroscopeBiasUncertainty = 10.0 * DEG_TO_RAD / 3600.0; // deg/hour converted to rad/sec

        return new INSLooselyCoupledKalmanInitializerConfig(initialAttitudeUncertainty, initialVelocityUncertainty,
                initialPositionUncertainty, initialAccelerationBiasUncertainty, initialGyroscopeBiasUncertainty);
    }
}
