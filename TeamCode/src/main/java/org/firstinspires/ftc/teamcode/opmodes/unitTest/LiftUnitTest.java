package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.hardware.motors.NeveRest60Gearmotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@HxUnitTest
@TeleOp(name = "Lift Unit Test", group = "unitTest")
public class LiftUnitTest extends OpMode {
    private HardwareConfig hardware;
    private Trigger homingTrigger;
    DcMotorEx motorEx;
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        hardware.motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        homingTrigger = new Trigger(0);
        motorEx = (DcMotorEx) hardware.motorLift;
        hardware.raiseLiftServo();
    }

    public void start() {
        hardware.lowerLiftServo();
        motorEx.setTargetPositionTolerance(0);
        //motorEx.setVelocityPIDFCoefficients(2.5, 0.5, 0, 11.0);

        //hardware.motorLift.setTargetPosition(-500);
        //hardware.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //hardware.motorLift.setPower(1);
    }

    @Override
    public void loop() {

        /*if(homingTrigger.getState() && gamepad1.a) {
            homingTrigger = new Trigger(500);
            hardware.startLiftHoming();
        }
        if(gamepad1.right_stick_y != 0) {
            hardware.isLiftHoming = false;
            telemetry.addData("Encoder position", hardware.currentLiftPos);
        }
        if(!hardware.isLiftHoming) hardware.setLiftPower(- gamepad1.left_stick_y);
        hardware.sanityCheck();*/
    }
}
