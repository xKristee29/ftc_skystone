package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@HxUnitTest
@TeleOp(name = "Pump Unit Test", group = "unitTest")
public class PumpUnitTest extends OpMode {
    private HardwareConfig hardware;
    Trigger homingTrig = new Trigger(0);
    Trigger suckTrig = new Trigger(0);
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("SwPompa", hardware.swPump.isPressed());
        telemetry.addData("Steps", hardware.motorPump.getCurrentPosition());
        if(!hardware.isPumpHoming && gamepad1.left_stick_y != 0){
            hardware.motorPump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            hardware.motorPump.setPower(- gamepad1.left_stick_y * 0.3);
        }
        if(homingTrig.getState() && gamepad1.a) {
            homingTrig = new Trigger(500);
            hardware.startPumpHoming();
        }
        if(suckTrig.getState() && gamepad1.b) {
            suckTrig = new Trigger(500);
            hardware.startSuction();
        }
        hardware.sanityCheck();
    }
}
