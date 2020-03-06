package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@TeleOp(name = "Measure Tape Unit Test", group = "unitTest")
public class MeasureTapeUnitTest extends OpMode {

    private HardwareConfig hardware;
    private DcMotor motorRuleta;
    @Override
    public void init(){
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        motorRuleta = hardwareMap.get(DcMotor.class, "motorRuleta");
    }

    @Override
    public void loop(){
        motorRuleta.setPower((gamepad1.right_trigger-gamepad1.left_trigger) * 0.3);
    }
}
