package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.apis.testing.kristee.CrMotors;

@TeleOp(name = "Bingo")
public class NainteNapoi extends OpMode {
    public DcMotor  leftFront = null,
            rightFront = null,
            leftRear = null,
            rightRear = null;

    @Override
    public void init() {

        leftFront = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        leftRear = hardwareMap.get(DcMotor.class,"leftRearMotor");
        rightRear = hardwareMap.get(DcMotor.class,"rightRearMotor");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        leftFront.setPower(gamepad1.left_stick_y);
        rightFront.setPower(gamepad1.left_stick_y);
        leftRear.setPower(gamepad1.left_stick_y);
        rightRear.setPower(gamepad1.left_stick_y);
    }
}
