package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@HxUnitTest
@TeleOp(name = "Slider Unit Test", group = "unitTest")
public class SliderUnitTest extends OpMode {
    private HardwareConfig hardware;
    TouchSensor swBrick;
    DcMotor motorRuleta;
    @Override
    public void init() {
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);

        motorRuleta = hardwareMap.get(DcMotor.class, "motorRuleta");
    }

    @Override
    public void loop() {
        telemetry.addData("Switch front", hardware.swSliderFront.isPressed());
        telemetry.addData("Switch rear", hardware.swSliderRear.isPressed());
        telemetry.addData("Block", hardware.skystoneColorCheck());
        /*if(gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
            double sum = gamepad2.right_trigger - gamepad2.left_trigger;
            if(sum < 0 && !(hardware.swSliderRear.isPressed())) hardware.servoSlider.setPosition((sum + 1) / 2);
            else if(sum > 0 && !(hardware.swSliderFront.isPressed())) hardware.servoSlider.setPosition((sum + 1) / 2);
            else hardware.servoSlider.setPosition(0.5);
        }
        else hardware.servoSlider.setPosition(0.5);*/
        telemetry.addData("Sw brick", swBrick.isPressed());
        motorRuleta.setPower(gamepad1.left_stick_y*0.3);
    }
}
