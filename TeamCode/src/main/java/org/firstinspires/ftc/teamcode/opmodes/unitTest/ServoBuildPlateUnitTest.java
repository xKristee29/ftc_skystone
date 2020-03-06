package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.apis.testing.Trigger;

@HxUnitTest
@TeleOp(name = "Servo Build Plate Unit Test", group = "unitTest")
public class ServoBuildPlateUnitTest extends OpMode {
    Servo servoTavaFront, servoTavaRear;
    Trigger valueChangerTrigger;
    @Override
    public void init() {
        valueChangerTrigger = new Trigger(0);
        servoTavaFront = hardwareMap.get(Servo.class, "servoTavaFront");
        servoTavaRear = hardwareMap.get(Servo.class, "servoTavaRear");

    }

    double servoValue = 1;
    @Override
    public void loop() {
        if(valueChangerTrigger.getState() && (gamepad1.dpad_up || gamepad1.dpad_down)){
            valueChangerTrigger = new Trigger(200);
            if(gamepad1.dpad_up)servoValue += 0.05;
            if(gamepad1.dpad_down)servoValue -= 0.05;
        }
        servoValue = Range.clip(servoValue, 0, 1);
        servoTavaFront.setPosition(0.3);
        servoTavaRear.setPosition(0.7);
        telemetry.addData("Angle", servoValue * 180);
    }
}
