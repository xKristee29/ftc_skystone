package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@TeleOp(name = "Calibrare 'Mericaneasca")
public class LocalizationCalibrate extends OpMode {
    private HardwareConfig hardware;
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
    }

    @Override
    public void loop() {
        hardware.mecanum.moveXYR(
                -hardware.localization.getCorrectionSpeedLeftDistance(400),
                hardware.localization.getCorrectionSpeedFrontDistance(400),
                hardware.localization.getAngleCorrectionLeft()
        );

        hardware.localization.update();
        telemetry.addData("leftA",hardware.localization.getAngleCorrectionLeft());
        telemetry.addData("leftS",hardware.localization.getCorrectionSpeedLeftDistance(400));
        telemetry.addData("frontS",hardware.localization.getCorrectionSpeedFrontDistance(400));
    }
}
