package org.firstinspires.ftc.teamcode.apis.testing;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Mecanum {

    /* <b>BEWARE!!</b>
       Angles range from -PI(right) to +PI(left),
       where 0 means center, not actual trigonometric 0.<br>
       Angles go positive counterclockwise.

       Rotation speed ranges from -1(right) to 1(left)
     */



    final double HALF_PI = Math.PI / 2;
    final double QUARTER_PI = Math.PI / 4;
    final double SQRT_OF_TWO = Math.sqrt(2);

    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;

    private boolean isInitialized() {
        return  (leftFrontMotor != null) &&
                (rightFrontMotor != null) &&
                (leftRearMotor != null) &&
                (rightRearMotor != null);
    }

    public void init(DcMotor leftFrontMotor, DcMotor rightFrontMotor, DcMotor leftRearMotor, DcMotor rightRearMotor) {
        this.leftFrontMotor = leftFrontMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.leftRearMotor = leftRearMotor;
        this.rightRearMotor = rightRearMotor;

        this.rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.rightRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**Sets motor speed for given strafe direction
     * @param strafeSpeed (0.0 - 1.0) limits speed of translation
     * @param strafeDirection angle in radians
     * @param rotateSpeed (-1.0 - 1.0) 1.0 rotates clockwise
     */
    public void move(double strafeSpeed , double strafeDirection, double rotateSpeed) {
        double angleSinComp = strafeSpeed * Math.sin(strafeDirection + QUARTER_PI);
        double angleCosComp = strafeSpeed * Math.cos(strafeDirection + QUARTER_PI);
        final double leftFrontSpeed     = - angleCosComp - rotateSpeed;
        final double rightFrontSpeed    = - angleSinComp + rotateSpeed;
        final double leftRearSpeed      = - angleSinComp - rotateSpeed;
        final double rightRearSpeed     = - angleCosComp + rotateSpeed;

        leftFrontMotor  .setPower(leftFrontSpeed);
        rightFrontMotor .setPower(rightFrontSpeed);
        leftRearMotor   .setPower(leftRearSpeed);
        rightRearMotor  .setPower(rightRearSpeed);
    }

    public void move(HardwareRepresentation.MotionState motionState) {
        move(motionState.strafeSpeed, motionState.strafeDirection, motionState.rotateSpeed);
    }

    public void moveXYR(double x, double y,  double rotateSpeed) {
        double strafeDirection = FastMath.fast_atan2(-x, -y);
        if(Double.isNaN(strafeDirection)) strafeDirection = 0;
        double strafeSpeed = Math.hypot(x, y);
        if(Double.isNaN(strafeSpeed)) strafeSpeed = 0;
        final double leftFrontSpeed     =  (strafeSpeed * Math.sin(strafeDirection + QUARTER_PI)) - rotateSpeed;
        final double rightFrontSpeed    =  (strafeSpeed * Math.cos(strafeDirection + QUARTER_PI)) + rotateSpeed;
        final double leftRearSpeed      =  (strafeSpeed * Math.cos(strafeDirection + QUARTER_PI)) - rotateSpeed;
        final double rightRearSpeed     =  (strafeSpeed * Math.sin(strafeDirection + QUARTER_PI)) + rotateSpeed;
        leftFrontMotor  .setPower(leftFrontSpeed);
        rightFrontMotor .setPower(rightFrontSpeed);
        leftRearMotor   .setPower(leftRearSpeed);
        rightRearMotor  .setPower(rightRearSpeed);
    }

    public void killSwitch() {
        leftFrontMotor  .setPower(0);
        rightFrontMotor .setPower(0);
        leftRearMotor   .setPower(0);
        rightRearMotor  .setPower(0);
    }

}
