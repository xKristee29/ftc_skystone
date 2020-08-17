package org.firstinspires.ftc.teamcode.opmodes.stable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;

@TeleOp(name = "Control Teleghidat")
public class ContolTeleghidat extends OpMode {
    HardwareConfigCT hardware = null;
    Trigger liftHomingTrig = new Trigger(0);
    Trigger dpadTrig = new Trigger(0);
    Trigger liftTrig = new Trigger(0);
    int liftLevel = 0;
    int levelPreset[] = {0, -650, -900, -1330, -1850, -2450, -3320, -4120, -5500};
    int lockDirection = 0;
    double liftPos = 0;
    boolean isLiftActive = false;
    boolean isSliderActive = false;
    boolean checkBlock = false;

    @Override
    public void init() {
        hardware = new HardwareConfigCT();
        hardware.initHardware(hardwareMap);
        
    }

    public void start() {
        hardware.startPumpHoming();
        hardware.rotateTurretCenter();
    }

    private void move(){
        if(gamepad1.a){
            if((!hardware.swFrontLeft.isPressed() && !hardware.swRearLeft.isPressed()) || (!hardware.swFrontRight.isPressed() && !hardware.swRearRight.isPressed())){
                hardware.mecanum.moveXYR(0,0,0);
            }
            else if(!hardware.swFrontLeft.isPressed() || !hardware.swFrontRight.isPressed()){
                hardware.mecanum.moveXYR(0,0.17,0);
            }
            else if(!hardware.swRearLeft.isPressed() || !hardware.swRearRight.isPressed()) {
                hardware.mecanum.moveXYR(0, -0.17, 0);
            }
            else {
                hardware.mecanum.moveXYR(
                        gamepad1.left_stick_x * 0.3,
                        - gamepad1.left_stick_y * 0.25,
                        gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
                );
            }
        }
        else{

            hardware.mecanum.moveXYR(
                    gamepad1.left_stick_x * hardware.SPEED_MECANUM_X,
                    - gamepad1.left_stick_y * hardware.SPEED_MECANUM_Y,
                    gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
            );

        }
    }

    public void startLiftHoming(){
        hardware.startLiftHoming();
        liftHomingTrig = new Trigger(500);
        liftLevel = 0;
    }

    public void keepLiftPos(){
        hardware.motorLift.setTargetPosition(hardware.motorLift.getCurrentPosition());
        hardware.motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.motorLift.setPower(1);
        hardware.liftRunMode = DcMotor.RunMode.RUN_TO_POSITION;
    }

    public void moveLift(){
        hardware.setLiftPower(gamepad2.right_stick_y);
        isLiftActive = false;
    }

    public void catchBlock(){
        hardware.mecanum.killSwitch();
        hardware.lowerLiftServo();
        hardware.sleep(500);
        hardware.startSuction();
        hardware.sleep(700);
        hardware.raiseLiftServo();
        checkBlock = true;
    }

    public void throwBlock(){
        hardware.mecanum.killSwitch();
        hardware.startPumpHoming();
        hardware.sleep(700);
        hardware.setSliderPower(1);
        isSliderActive = true;
        checkBlock = false;
    }

    public void addLiftLevel(){
        if(liftLevel < levelPreset.length - 1) liftLevel += 1;
        isLiftActive = true;
        hardware.setLiftLevel(levelPreset[liftLevel]);
        liftTrig = new Trigger(500);
    }

    public void decreaseLiftLevel(){
        if(liftLevel > 1) liftLevel -= 1;
        isLiftActive = true;
        hardware.setLiftLevel(levelPreset[liftLevel]);
        liftTrig = new Trigger(500);
    }

    @Override
    public void loop() {

        move();

        if(!hardware.isLiftHoming && Math.abs(gamepad2.right_stick_y) > 0.1) moveLift();

        move();

        if(!hardware.isLiftHoming && Math.abs(gamepad2.right_stick_y) < 0.1 && !isLiftActive) keepLiftPos();

        move();

        if(gamepad1.left_bumper) hardware.grabBuildPlate();
        if(gamepad1.right_bumper) hardware.releaseBuildPlate();

        move();


        if(liftHomingTrig.getState() && gamepad2.dpad_down) startLiftHoming();

        move();


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

        move();

        if(gamepad2.right_bumper) catchBlock();

        move();

        if(gamepad2.left_bumper && !isSliderActive) throwBlock();
        else if(isSliderActive && hardware.swSliderFront.isPressed()) isSliderActive = false;
        else if(isSliderActive) hardware.setSliderPower(1);

        move();

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

        move();

        hardware.motorRuleta.setPower(gamepad1.right_trigger-gamepad1.left_trigger);

        if(!isSliderActive) hardware.setSliderPower(gamepad2.left_trigger-gamepad2.right_trigger);

        move();

        if(liftTrig.getState() && gamepad2.y) addLiftLevel();
        else if(liftTrig.getState() && gamepad2.a) decreaseLiftLevel();
        else if(isLiftActive && hardware.currentLiftPos == hardware.motorLift.getCurrentPosition()) isLiftActive = false;

        move();

        telemetry.addData("Lift Pos: ", hardware.currentLiftPos);
        telemetry.addData("Dist Rear: ", hardware.localization.distanceRear.lastReadout);
        telemetry.addData("SW FL: ",hardware.swFrontLeft.isPressed());
        telemetry.addData("SW RL: ",hardware.swRearLeft.isPressed());
        telemetry.addData("SW FR: ",hardware.swFrontRight.isPressed());
        telemetry.addData("SW RR: ",hardware.swRearRight.isPressed());
        telemetry.update();

        hardware.sanityCheck();
    }
}
