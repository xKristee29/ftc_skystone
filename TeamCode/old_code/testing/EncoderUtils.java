package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="encoder test")
public class EncoderUtils extends OpMode {
    DcMotor lift = null;
    DcMotor glisiera = null;
    Servo servo = null;

    @Override
    public void init() {
        /*servo = hardwareMap.get(Servo.class, "servoGheara");
        servo.setPosition(0.05);*/
        lift = hardwareMap.get(DcMotor.class, "motorLift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        glisiera = hardwareMap.get(DcMotor.class, "motorGlisiera");
        glisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void loop() {
        telemetry.addData("Glisiera pasi", glisiera.getCurrentPosition());
        telemetry.addData("Lift pasi", lift.getCurrentPosition());
        glisiera.setPower(0.25*gamepad2.left_stick_y);
        lift.setPower(0.5*gamepad1.right_stick_y);
        /*if(gamepad2.left_bumper)servo.setPosition(0.05);
        if(gamepad2.right_bumper)servo.setPosition(1);*/
    }
}
