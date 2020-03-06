package org.firstinspires.ftc.teamcode.opmodes.stable.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;
@Disabled
@TeleOp(name = "Control Teleghidat", group = "CT")
public class ContolTeleghidat extends OpMode {
    HardwareConfig hardware = null;
    Trigger homingTrig = new Trigger(0);
    Trigger suckTrig = new Trigger(0);

    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
    }


    double liftPos = 0;
    long time = System.currentTimeMillis();
    int lockDirection = 0;
    @Override
    public void loop() {

        // Move chassis
        /*hardware.mecanum.moveXYR(
                gamepad1.left_stick_x * hardware.SPEED_MECANUM_STRAFE,
                - gamepad1.left_stick_y * hardware.SPEED_MECANUM_STRAFE,
                gamepad1.right_stick_x * hardware.SPEED_MECANUM_ROTATION
        );

        // Slider
        if(gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
            double sum = gamepad2.right_trigger - gamepad2.left_trigger;
            if(sum < 0 && !(hardware.swSliderRear.isPressed())) hardware.servoSlider.setPosition((sum + 1) / 2);
            else if(sum > 0 && !(hardware.swSliderFront.isPressed())) hardware.servoSlider.setPosition((sum + 1) / 2);
            else hardware.servoSlider.setPosition(0.5);
        }
        else hardware.servoSlider.setPosition(0.5);

        // Lift
        hardware.setLiftPower(- gamepad2.right_stick_y);

        // Servo build plate
        if(gamepad1.a) hardware.grabBuildPlate();
        if(gamepad1.b) hardware.releaseBuildPlate();

        // Stone pump
        if(homingTrig.getState() && gamepad2.left_bumper) {
            homingTrig = new Trigger(500);
            hardware.pumpStartHoming();
        }
        if(suckTrig.getState() && gamepad2.right_bumper) {
            suckTrig = new Trigger(500);
            hardware.startSuction();
        }
        hardware.pumpSanityCheck();
        
        // Turret
        if(gamepad2.left_stick_x != 0) {
            if(!hardware.turretHomingSwitch.isPressed()) {
                if(gamepad1.left_stick_x < 0) {
                    lockDirection = -1;
                }
                if(gamepad1.left_stick_x > 0) {
                    lockDirection = 1;
                }
                hardware.motorTurret.setPower(gamepad1.left_stick_x * 0.5);
            }
            else {
                if (lockDirection != Math.signum(gamepad1.left_stick_x)) {
                    hardware.motorTurret.setPower(gamepad1.left_stick_x * 0.5);
                }
            }
        }
        else hardware.motorTurret.setPower(0);


        telemetry.addData("loopTime",System.currentTimeMillis() - time);
        System.gc();
        Thread.yield();
        time = System.currentTimeMillis();*/
    }
}
