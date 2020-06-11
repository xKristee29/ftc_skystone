package org.firstinspires.ftc.teamcode.apis.testing;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.apis.testing.HardwareRepresentation.PositionTarget;

public class Localization {
    /**Wrapper class for DistanceSensor that saves last readout
     * for future use
     */
    public class SensorInstance {
        DistanceSensor sensor = null;
        public boolean enable = true;
        public double lastReadout = 0;

        private DistanceUnit DISTANCE_UNIT = DistanceUnit.MM;

        /**If distance returned by sensor is greater than this
         * constant readout value will not be updated.
         */
        private final double MAXIMUM_READOUT = 2000;

        public SensorInstance(DistanceSensor sensor) {
            this.sensor = sensor;
        }

        /**Reads sensor data and saves it into lastReadout
         * if it meets the required conditions
         */
        public void update() {
            if(enable) {
                double currentReadout = sensor.getDistance(DISTANCE_UNIT);
                if(currentReadout <= MAXIMUM_READOUT) lastReadout = currentReadout;
            }
        }
    }

    public SensorInstance
            distanceFront = null, //sd1
            distanceRear = null, //sd4
            distanceFrontLeft = null, //sd6
            distanceRearLeft = null, //sd5
            distanceFrontRight = null, //sd2
            distanceRearRight = null; //sd3


    public void init(DistanceSensor sd1, DistanceSensor sd2, DistanceSensor sd3, DistanceSensor sd4, DistanceSensor sd5, DistanceSensor sd6) {
        distanceFront = new SensorInstance(sd1); //sd1
        distanceRear = new SensorInstance(sd4); //sd4
        distanceFrontLeft = new SensorInstance(sd6); //sd6
        distanceRearLeft = new SensorInstance(sd5); //sd5
        distanceFrontRight = new SensorInstance(sd2); //sd2
        distanceRearRight = new SensorInstance(sd3); //sd3
    }

    /**Updates lastReadout for each sensor
     */
    public void update() {
        distanceFront.update();
        distanceRear.update();
        distanceFrontLeft.update();
        distanceRearLeft.update();
        distanceFrontRight.update();
        distanceRearRight.update();
    }

    public static double SPEED_SCALE_FACTOR = 1;

    /**Algorithm for generating numerical factor
     * that could be applied to speeds in order
     * to end up to a numerical target distance, taking
     * into account the current numerical distance,
     * usually taken from a sensor
     */
    public static double getCorrection(double target, double value, double proportionalCoef, double brakingDist, double marginOfErr) {
        double correction = (value - target) / proportionalCoef;

        //If robot is outside braking distance give maximum absolute correction value.
        if(Math.abs(value - target) > brakingDist) {
            correction = Math.signum(correction);
        }
        //In case robot is actually in braking distance
        //check if the correction is within margin of error in
        //order to return 0, else return normal correction value
        else {
            if(Math.abs(correction) < marginOfErr) {
                correction = 0;
            }
        }
        return Range.clip(correction, -1, 1) * SPEED_SCALE_FACTOR;
    }

    /**Applies correction algorithm to front distance sensor
     */
    public double getCorrectionSpeedFrontDistance(double target, double PROPORTIONAL_COEF, double BRAKING_DIST, double MARGIN_OF_ERR) {
        return getCorrection(target, distanceFront.lastReadout, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    public double getCorrectionSpeedFrontDistance(double target) {
        final double PROPORTIONAL_COEF = 200;
        final double BRAKING_DIST = 220;
        final double MARGIN_OF_ERR = 0.25;

        return getCorrection(target, distanceFront.lastReadout, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    /**Applies correction algorithm to rear distance sensor
     */
    public double getCorrectionSpeedRearDistance(double target, double PROPORTIONAL_COEF, double BRAKING_DIST, double MARGIN_OF_ERR) {
        return getCorrection(target, distanceRear.lastReadout, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    public double getCorrectionSpeedRearDistance(double target) {
        final double PROPORTIONAL_COEF = 300;
        final double BRAKING_DIST = 300;
        final double MARGIN_OF_ERR = 0.1;

        return getCorrection(target, distanceRear.lastReadout, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    /**Applies correction algorithm to median distance of left distance sensors
     */
    public double getCorrectionSpeedLeftDistance(double target, double PROPORTIONAL_COEF, double BRAKING_DIST, double MARGIN_OF_ERR) {
        double leftDistance = (distanceFrontLeft.lastReadout + distanceRearLeft.lastReadout) / 2;
        return getCorrection(target, leftDistance, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    public double getCorrectionSpeedLeftDistance(double target) {
        final double PROPORTIONAL_COEF = 400;
        final double BRAKING_DIST = 520;
        final double MARGIN_OF_ERR = 0.25;

        double leftDistance = (distanceFrontLeft.lastReadout + distanceRearLeft.lastReadout) / 2;
        return getCorrection(target, leftDistance, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    /**Applies correction algorithm to median distance of right distance sensors
     */
    public double getCorrectionSpeedRightDistance(double target, double PROPORTIONAL_COEF, double BRAKING_DIST, double MARGIN_OF_ERR) {
        double rightDistance = (distanceFrontRight.lastReadout + distanceRearRight.lastReadout) / 2;
        return getCorrection(target, rightDistance, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    public double getCorrectionSpeedRightDistance(double target) {
        final double PROPORTIONAL_COEF = 400;
        final double BRAKING_DIST = 520;
        final double MARGIN_OF_ERR = 0.25;

        double rightDistance = (distanceFrontRight.lastReadout + distanceRearRight.lastReadout) / 2;
        return getCorrection(target, rightDistance, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    /**Applies correction algorithm to distance difference of left distance sensors,
     * used for targeting a certain angle
     */
    public double getAngleCorrectionLeft(double target, double PROPORTIONAL_COEF, double BRAKING_DIST, double MARGIN_OF_ERR) {
        double leftDiff = - (distanceFrontLeft.lastReadout - distanceRearLeft.lastReadout);
        return getCorrection(target, leftDiff, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    public double getAngleCorrectionLeft(double target) {
        final double PROPORTIONAL_COEF = 400;
        final double MARGIN_OF_ERR = 0.25;
        final double BRAKING_DIST = 500;

        return getCorrection(target, - (distanceFrontLeft.lastReadout - distanceRearLeft.lastReadout), PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    /**Applies correction algorithm to distance difference of left distance sensors,
     * target difference being 0
     */
    public double getAngleCorrectionLeft() {
        return getAngleCorrectionLeft(0);
    }

    /**Applies correction algorithm to distance difference of right distance sensors,
     * used for targeting a certain angle
     */
    public double getAngleCorrectionRight(double target, double PROPORTIONAL_COEF, double BRAKING_DIST, double MARGIN_OF_ERR) {
        double rightDiff = - (distanceFrontRight.lastReadout - distanceRearRight.lastReadout);
        return getCorrection(target, rightDiff, PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    public double getAngleCorrectionRight(double target) {
        final double PROPORTIONAL_COEF = 400;
        final double MARGIN_OF_ERR = 0.25;
        final double BRAKING_DIST = 500;

        return getCorrection(target, (distanceFrontRight.lastReadout - distanceRearRight.lastReadout), PROPORTIONAL_COEF, BRAKING_DIST, MARGIN_OF_ERR);
    }

    /**Applies correction algorithm to distance difference of right distance sensors,
     * target difference being 0
     */
    public double getAngleCorrectionRight() {
        return getAngleCorrectionRight(0);
    }

}