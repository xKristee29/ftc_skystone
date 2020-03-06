package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@HxUnitTest
@TeleOp(name = "Gyro Unit Test", group = "unitTest")
public class GyroUnitTest extends OpMode {
    private HardwareConfig hardware;
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Angle", hardware.gyro.getValue());

    }
}
