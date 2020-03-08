package org.firstinspires.ftc.teamcode.opmodes.stable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;

@TeleOp(name = "Control Teleghidat")
public class ContolTeleghidat extends OpMode {
    HardwareConfig hardware = null;
    Trigger liftHomingTrig = new Trigger(0);
    Trigger dpadTrig = new Trigger(0);
    Trigger liftTrig = new Trigger(0);
    int liftLevel = 0;
    int levelPreset[] = {0, -650, -800, -1150, -1580, -2050, -2720, -3680, -4770};
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        telemetry.addData(">>", "Press play to pay respect");
        telemetry.update();
    }

    public void start() {
        hardware.startPumpHoming();
        hardware.rotateTurretCenter();
    }



    int lockDirection = 0;
    double liftPos = 0;
    boolean isLiftActive = false;
    boolean isSliderActive = false;
    boolean checkBlock = false;
    @Override
    public void loop() {

        if(gamepad1.a){
            hardware.SPEED_MECANUM_X = 0.4;
            hardware.SPEED_MECANUM_Y = 0.3;
        }
        else{
            hardware.SPEED_MECANUM_X = 0.6;
            hardware.SPEED_MECANUM_Y = 0.5;
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(!hardware.isLiftHoming && Math.abs(gamepad2.right_stick_y) > 0.1) {
            hardware.setLiftPower(gamepad2.right_stick_y);
            isLiftActive = false;
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(!hardware.isLiftHoming && Math.abs(gamepad2.right_stick_y) < 0.1 && !isLiftActive) {
            hardware.motorLift.setTargetPosition(hardware.motorLift.getCurrentPosition());
            hardware.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            hardware.motorLift.setPower(1);
            hardware.liftRunMode = DcMotor.RunMode.RUN_TO_POSITION;
        }

        if(gamepad1.left_bumper) hardware.grabBuildPlate();
        if(gamepad1.right_bumper) hardware.releaseBuildPlate();

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );


        if(liftHomingTrig.getState() && gamepad2.dpad_down){
            hardware.startLiftHoming();
            liftHomingTrig = new Trigger(500);
            liftLevel = 0;
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );


        if(checkBlock == false){

            if(dpadTrig.getState() && gamepad2.dpad_up) {
                hardware.rotateTurretCenter();
                dpadTrig = new Trigger(250);
            }
            else if(dpadTrig.getState() && gamepad2.dpad_left){
                hardware.rotateTurretLeft();
                dpadTrig = new Trigger(250);
            }
            else if(dpadTrig.getState() && gamepad2.dpad_right){
                hardware.rotateTurretRight();
                dpadTrig = new Trigger(250);
            }
            else if(hardware.isTurretActive && hardware.motorTurret.getCurrentPosition() == hardware.turretPos) hardware.isTurretActive=false;
            else if(gamepad2.left_stick_x == 0 && !hardware.isTurretActive) hardware.setTurretPower(0);
        }
        else if(checkBlock == true) {

            if(dpadTrig.getState() && hardware.currentLiftPos <= -470 && gamepad2.dpad_up) {
                hardware.rotateTurretCenter();
                dpadTrig = new Trigger(250);
            }
            else if(dpadTrig.getState() && hardware.currentLiftPos <= -470 && gamepad2.dpad_left){
                hardware.rotateTurretLeft();
                dpadTrig = new Trigger(250);
            }
            else if(dpadTrig.getState() && hardware.currentLiftPos <= -470 && gamepad2.dpad_right){
                hardware.rotateTurretRight();
                dpadTrig = new Trigger(250);
            }
            else if(hardware.isTurretActive && hardware.motorTurret.getCurrentPosition() == hardware.turretPos) hardware.isTurretActive=false;
            else if(gamepad2.left_stick_x == 0 && !hardware.isTurretActive) hardware.setTurretPower(0);
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(gamepad2.right_bumper) {
            hardware.mecanum.killSwitch();
            hardware.lowerLiftServo();
            hardware.sleep(500);
            hardware.startSuction();
            hardware.sleep(700);
            hardware.raiseLiftServo();
            checkBlock = true;
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(gamepad2.left_bumper && !isSliderActive) {
            hardware.mecanum.killSwitch();
            hardware.startPumpHoming();
            hardware.sleep(700);
            hardware.setSliderPower(1);
            isSliderActive = true;
            checkBlock = false;
        }
        else if(isSliderActive && hardware.swSliderFront.isPressed()){
            isSliderActive = false;
        }
        else if(isSliderActive){
            hardware.setSliderPower(1);
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        if(Math.abs(gamepad2.left_stick_x) > 0.5 && !hardware.isTurretActive) {
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
        else if(!hardware.isTurretActive) hardware.motorTurret.setPower(0);

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        hardware.motorRuleta.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

        if(!isSliderActive) hardware.setSliderPower(gamepad2.left_trigger-gamepad2.right_trigger);

        if(liftTrig.getState() && gamepad2.y) {
            if(liftLevel < levelPreset.length - 1) liftLevel += 1;
            isLiftActive = true;
            hardware.setLiftLevel(levelPreset[liftLevel]);
            liftTrig = new Trigger(500);
        }
        else if(liftTrig.getState() && gamepad2.a){
            if(liftLevel > 1) liftLevel -= 1;
            isLiftActive = true;
            hardware.setLiftLevel(levelPreset[liftLevel]);
            liftTrig = new Trigger(500);
        }
        else if(isLiftActive && hardware.currentLiftPos == hardware.motorLift.getCurrentPosition()) {
            isLiftActive = false;
        }

        hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        telemetry.addData("Lift Pos: ", hardware.currentLiftPos);
        telemetry.update();

        hardware.sanityCheck();
    }
}
