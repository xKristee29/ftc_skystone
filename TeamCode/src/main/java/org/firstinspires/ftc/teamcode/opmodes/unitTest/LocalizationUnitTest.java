package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@HxUnitTest
@TeleOp(name = "Localization Unit Test", group = "unitTest")
public class LocalizationUnitTest extends OpMode {
    private HardwareConfig hardware;
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
    }

    @Override
    public void loop() {
        hardware.localization.update();
        telemetry.addData("SD1", hardware.localization.distanceFront.lastReadout);
        telemetry.addData("SD2", hardware.localization.distanceFrontRight.lastReadout);
        telemetry.addData("SD3", hardware.localization.distanceRearRight.lastReadout);
        telemetry.addData("SD4", hardware.localization.distanceRear.lastReadout);
        telemetry.addData("SD5", hardware.localization.distanceRearLeft.lastReadout);
        telemetry.addData("SD6", hardware.localization.distanceFrontLeft.lastReadout);
    }
}
