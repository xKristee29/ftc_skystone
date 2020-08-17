package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class test extends LinearOpMode {

    DcMotor SFmot = null;
    DcMotor DFmot = null;
    DcMotor SSmot = null;
    DcMotor DSmot = null;


    public void initHW(){
        SFmot = hardwareMap.get(DcMotor.class, "SFmot");
        DFmot = hardwareMap.get(DcMotor.class, "DFmot");
        SSmot = hardwareMap.get(DcMotor.class, "SSmot");
        DSmot = hardwareMap.get(DcMotor.class, "DSmot");

        DFmot.setDirection(DcMotorSimple.Direction.REVERSE);
        DSmot.setDirection(DcMotorSimple.Direction.REVERSE);

        SFmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DFmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        SSmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        DSmot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void runOpMode(){

        initHW();
        telemetry.addData(">", "Finished" );
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){

            SFmot.setPower(0.5);
            DFmot.setPower(0.5);
            SSmot.setPower(0.5);
            DSmot.setPower(0.5);

            sleep(3000);

            SFmot.setPower(0);
            DFmot.setPower(0);
            SSmot.setPower(0);
            DSmot.setPower(0);

            sleep(3000);

            idle();

        }

        SFmot.setPower(0);
        DFmot.setPower(0);
        SSmot.setPower(0);
        DSmot.setPower(0);

    }

}
