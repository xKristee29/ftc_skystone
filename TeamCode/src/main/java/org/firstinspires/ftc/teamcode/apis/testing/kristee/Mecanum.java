package org.firstinspires.ftc.teamcode.apis.testing.kristee;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.apis.testing.HardwareRepresentation;

public class Mecanum {

    /* <b>BEWARE!!</b>
       Angles range from -PI(right) to +PI(left),
       where 0 means center, not actual trigonometric 0.<br>
       Angles go positive counterclockwise.

       Rotation speed ranges from -1(right) to 1(left)
     */

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

        //this.leftFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.leftRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //this.rightRearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**Sets motor speed for given strafe direction
     * @param strafeSpeed (0.0 - 1.0) limits speed of translation
     * @param strafeDirection angle in radians
     * @param rotateSpeed (-1.0 - 1.0) 1.0 rotates clockwise
     */
    public void move(double strafeSpeed , double strafeDirection, double rotateSpeed) {
        double strafeAngle = strafeDirection - Math.PI / 4 + Math.PI / 2;
        final double leftFrontSpeed     = strafeSpeed * Math.sin(strafeAngle) - rotateSpeed;
        final double rightFrontSpeed    = strafeSpeed * Math.cos(strafeAngle) + rotateSpeed;
        final double leftRearSpeed      = strafeSpeed * Math.cos(strafeAngle) - rotateSpeed;
        final double rightRearSpeed     = (strafeSpeed * 1.07) * Math.sin(strafeAngle) + rotateSpeed;

        if(this.isInitialized()) {
            leftFrontMotor  .setPower(leftFrontSpeed);
            rightFrontMotor .setPower(rightFrontSpeed);
            leftRearMotor   .setPower(leftRearSpeed);
            rightRearMotor  .setPower(rightRearSpeed);
            leftFrontMotor  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightFrontMotor .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftRearMotor   .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            rightRearMotor  .setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    public void move(HardwareRepresentation.MotionState motionState) {
        move(motionState.strafeSpeed, motionState.strafeDirection, motionState.rotateSpeed);
    }

    public void killSwitch() {
        leftFrontMotor  .setPower(0);
        rightFrontMotor .setPower(0);
        leftRearMotor   .setPower(0);
        rightRearMotor  .setPower(0);
    }
}
