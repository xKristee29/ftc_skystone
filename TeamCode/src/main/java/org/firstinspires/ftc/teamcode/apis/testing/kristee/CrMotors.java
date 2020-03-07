package org.firstinspires.ftc.teamcode.apis.testing.kristee;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;

public class CrMotors {

    // Declare motors and the control object
    public Mecanum go = null;
    public DcMotor  leftFront = null,
            rightFront = null,
            leftRear = null,
            rightRear = null;

    // Set strafe direction
    double strafe = Math.PI/2;

    public void init(HardwareMap hardwareMap){

        // Initialize motors
        leftFront = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        leftRear = hardwareMap.get(DcMotor.class,"leftRearMotor");
        rightRear = hardwareMap.get(DcMotor.class,"rightRearMotor");

        // Initialize control object
        go = new Mecanum();

        go.init(leftFront,
                rightFront,
                leftRear,
                rightRear);

    }

    // Power up the motors with declared speed and direction correction
    public void move(double strafeSpeed ,double strafeDirection,double rotateSpeed){

        go.move(strafeSpeed, strafeDirection * strafe, rotateSpeed);

    }

}
