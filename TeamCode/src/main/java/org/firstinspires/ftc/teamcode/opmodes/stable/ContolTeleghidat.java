package org.firstinspires.ftc.teamcode.opmodes.stable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;

@TeleOp(name = "Control Teleghidat")
public class ContolTeleghidat extends OpMode {
    HardwareConfig hardware = null;
    Trigger liftHomingTrig = new Trigger(0);
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
    }

    public void start() {
        hardware.startPumpHoming();
    }


    int lockDirection = 0;
    double liftPos = 0;
    boolean isLiftActive = false;
    @Override
    public void loop() {


        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(!hardware.isLiftHoming && !isLiftActive) {
            hardware.setLiftPower(gamepad2.right_stick_y);
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(!hardware.isLiftHoming && Math.abs(gamepad2.right_stick_y) < 0.1 && !isLiftActive) {
            hardware.motorLift.setTargetPosition(hardware.motorLift.getCurrentPosition());
            hardware.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.motorLift.setPower(1);
            hardware.liftRunMode = DcMotor.RunMode.RUN_TO_POSITION;
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(gamepad1.left_bumper) hardware.grabBuildPlate();
        if(gamepad1.right_bumper) hardware.releaseBuildPlate();

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(liftHomingTrig.getState() && gamepad2.dpad_down){
            hardware.startLiftHoming();
            liftHomingTrig = new Trigger(500);
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(hardware.currentLiftPos <= -470 && gamepad2.dpad_up)        hardware.rotateTurretCenter();
        else if(hardware.currentLiftPos <= -470 && gamepad2.dpad_left) hardware.rotateTurretLeft();
        else if(hardware.currentLiftPos <= -470 && gamepad2.dpad_right)hardware.rotateTurretRight();
        else if(hardware.isTurretActive && hardware.motorTurret.getCurrentPosition() == hardware.turretPos) hardware.isTurretActive=false;
        else if(gamepad2.left_stick_x == 0 && !hardware.isTurretActive) hardware.setTurretPower(0);

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(gamepad2.right_bumper) {
            hardware.mecanum.killSwitch();
            hardware.lowerLiftServo();
            hardware.sleep(1000);
            hardware.startSuction();
            hardware.sleep(1000);
            hardware.raiseLiftServo();
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(gamepad2.left_stick_x != 0 && !hardware.isTurretActive) {
            if(hardware.turretHomingSwitch.isPressed()) {
                if(gamepad2.left_stick_x < 0) {
                    lockDirection = -1;
                }
                if(gamepad2.left_stick_x > 0) {
                    lockDirection = 1;
                }
                hardware.motorTurret.setPower(gamepad2.left_stick_x * 0.7);
            }
            else {
                if (lockDirection != Math.signum(gamepad2.left_stick_x)) {
                    hardware.motorTurret.setPower(gamepad2.left_stick_x * 0.7);
                }
            }
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        hardware.motorRuleta.setPower(gamepad1.left_trigger-gamepad1.right_trigger);
        if(gamepad2.left_bumper) hardware.startPumpHoming();

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        hardware.setSliderPower(gamepad2.left_trigger-gamepad2.right_trigger);
        hardware.sanityCheck();
        if(gamepad2.a) {
            isLiftActive = true;
            hardware.setLiftLevel(-650);
        }
        else if(isLiftActive && hardware.currentLiftPos == hardware.motorLift.getCurrentPosition()) isLiftActive = false;
    }
}
