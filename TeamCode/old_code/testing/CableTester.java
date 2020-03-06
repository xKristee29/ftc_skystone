package testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;
@Disabled
@TeleOp(name = "MotorTest")
public class CableTester extends OpMode {
    /*
     * Moarte lui Aladeen
     */
    Mecanum go = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    @Override
    public void init() {
        leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class,"leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class,"rightRearMotor");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRearMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        go = new Mecanum();

        go.init(leftFrontMotor,
                rightFrontMotor,
                leftRearMotor,
                rightRearMotor);
    }

    @Override
    public void loop() {
        if(gamepad1.a)leftFrontMotor.setPower(0.2);
        else leftFrontMotor.setPower(0);
        if(gamepad1.b)rightFrontMotor.setPower(0.2);
        else rightFrontMotor.setPower(0);
        if(gamepad1.x)leftRearMotor.setPower(0.2);
        else leftRearMotor.setPower(0);
        if(gamepad1.y)rightRearMotor.setPower(0.2);
        else rightRearMotor.setPower(0);
    }
}
