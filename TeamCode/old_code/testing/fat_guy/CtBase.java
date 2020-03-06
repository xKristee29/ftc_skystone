package old_code.testing.fat_guy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;

@TeleOp(name = "Ct simplu")
public class CtBase extends OpMode {
    DcMotor leftFrontMotor, leftRearMotor, rightFrontMotor, rightRearMotor;
    DcMotor motorLift;
    Mecanum go;

    final int LIMIT_MIN_LIFT = -40;
    final int LIMIT_MAX_LIFT = -2400;

    final double SPEED_LIFT = 0.5;
    final double SPEED_MECANUM = 0.5;

    @Override
    public void init() {
        go = new Mecanum();
        leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class,"leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class,"rightRearMotor");
        go.init(leftFrontMotor, rightFrontMotor, leftRearMotor, rightRearMotor);

        motorLift = hardwareMap.get(DcMotor.class, "motorLift");

        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    double liftPos = 0;
    @Override
    public void loop() {
        if(liftPos > - 300)go.moveXYR(gamepad1.left_stick_x * 0.5, - gamepad1.left_stick_y * 0.5, gamepad1.right_stick_x);
        else go.killSwitch();

        if(gamepad2.right_stick_y != 0)liftPos = motorLift.getCurrentPosition();
        if(gamepad2.right_stick_y > 0 && liftPos < LIMIT_MIN_LIFT) motorLift.setPower(gamepad2.right_stick_y * SPEED_LIFT);
        else if(gamepad2.right_stick_y < 0 && liftPos > LIMIT_MAX_LIFT) motorLift.setPower(gamepad2.right_stick_y * SPEED_LIFT);
        else motorLift.setPower(0);
    }
}
