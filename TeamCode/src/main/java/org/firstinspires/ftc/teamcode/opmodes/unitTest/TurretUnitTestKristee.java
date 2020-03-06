package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@HxUnitTest
@TeleOp(name = "Turret Unit Test Kristee", group = "unitTest")
public class TurretUnitTestKristee extends OpMode {
    private HardwareConfig hardware;
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);

    }

    int lockDirection = 0;

    @Override
    public void loop() {
        telemetry.addData("Sw Turela", !hardware.turretHomingSwitch.isPressed());
        telemetry.addData("Pasi Turela", hardware.motorTurret.getCurrentPosition());
        //hardware.motorTurret.setPower(gamepad1.left_stick_x * 0.3);

        if(gamepad1.dpad_up)hardware.motorTurret.setTargetPosition(0);
        if(gamepad1.dpad_left)hardware.motorTurret.setTargetPosition(-660);
        if(gamepad1.dpad_right)hardware.motorTurret.setTargetPosition(660);
        /*if((lockDirection != -1) && gamepad1.left_stick_x < 0) {
            if(lockDirection == 0 && !hardware.turretHomingSwitch.isPressed()){
                lockDirection = -1;
                hardware.motorTurret.setPower(gamepad1.left_stick_x * 0.5);
            }
            //else lockDirection = 0;

        }
        else if((lockDirection != 1) && gamepad1.left_stick_x > 0) {
            if(lockDirection == 0 && !hardware.turretHomingSwitch.isPressed()){
                lockDirection = 1;
                hardware.motorTurret.setPower(gamepad1.left_stick_x * 0.5);
            }
            //else lockDirection = 0;

        }
        else {
            hardware.motorTurret.setPower(0);
        }*/
        /*if(gamepad1.left_stick_x != 0) {
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
        else hardware.motorTurret.setPower(0);*/
    }
}
