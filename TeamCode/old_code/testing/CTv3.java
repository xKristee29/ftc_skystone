package testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;

import java.util.concurrent.TimeUnit;

@TeleOp (name = "Control Teleghidat BBH", group = "a")
public class CTv3 extends LinearOpMode {
    /*
        Gamepad 1 (Driving):
        > left_joystick (axa Y)  - miscare fata-spate
        > left_joystick (axa X)  - translatie stanga-dreapta
        > right_joystick (axa X) - rotatie stanga-dreapta
        > LB - lasa tava
        > RB - prinde tava

        Gamepad 2 (Brate si lift):
        > left_joystick (axa Y)  - trage sau impinge glisiera
        > right_joystick (axa Y) - intinde sau strange lift
        > LB - lasa brick
        > RB - prinde brick
     */

    Mecanum go = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    DcMotor motorLift = null;
    DcMotor motorGlisiera = null;
    Servo servo_pickup = null;
    Servo servoTava = null;
    TouchSensor buttFront, buttRear;

    final int LIMIT_MIN_GLISIERA = -80;
    final int LIMIT_MAX_GLISIERA = -700;

    final int LIMIT_MIN_LIFT = -40;
    final int LIMIT_MAX_LIFT = -2400;

    final double SERVO_BRICK_ZERO_POSITION = 0;
    final double SERVO_BRICK_CATCH_POSITION = 1;

    final double SERVO_TAVA_ZERO_POSITION = -0.16;
    final double SERVO_TAVA_CATCH_POSITION = 0.16;

    final double SPEED_GLISIERA = 0.2;
    final double SPEED_LIFT = 0.5;

    boolean buttState = false;
    long time;

    public void initHw() {

        leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class,"leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class,"rightRearMotor");

        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motorGlisiera.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        servo_pickup = hardwareMap.get(Servo.class, "servoGheara");
        servoTava = hardwareMap.get(Servo.class, "servoTavaA");
        buttFront = hardwareMap.get(TouchSensor.class, "btnTavaA");
        buttRear = hardwareMap.get(TouchSensor.class, "btnTavaB");

        /*servoTava.setPosition(SERVO_TAVA_ZERO_POSITION);
        servo_pickup.setPosition(SERVO_BRICK_ZERO_POSITION);*/

        go = new Mecanum();

        go.init(leftFrontMotor,
                rightFrontMotor,
                leftRearMotor,
                rightRearMotor);
    }

    boolean servoPos1 = false;
    boolean servoPos2 = false;

    @Override
    public void runOpMode() {

        //Init
        initHw();

        waitForStart();

        try {
            /*motorGlisiera.setTargetPosition(-90);
            motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorGlisiera.setPower(0.6);*/
            /*while (opModeIsActive() && (!isStopRequested()) && motorGlisiera.isBusy()){
                if((!opModeIsActive()) || isStopRequested()) throw new InterruptedException();
                Thread.yield();
                sleep(5);
            }*/

            motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



            time = System.nanoTime();
            while (-TimeUnit.MILLISECONDS.convert(time - System.nanoTime(), TimeUnit.NANOSECONDS) <= 120000) {
                if(gamepad2.right_stick_y != 0) motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                else {
                    motorLift.setTargetPosition(motorLift.getCurrentPosition());
                    motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                // Servo brick
                if(gamepad2.right_bumper) {

                    if(servoPos1 == false) { servo_pickup.setPosition(SERVO_BRICK_CATCH_POSITION); servoPos1 = true;}

                }
                if(gamepad2.left_bumper){

                    if(servoPos1 == true) { servo_pickup.setPosition(SERVO_BRICK_ZERO_POSITION); servoPos1 = false;}

                }

                // Servo tava
                buttState = (buttFront.isPressed() && buttRear.isPressed());
                if(gamepad1.right_bumper && buttState) {

                    if(servoPos2 == false) { servoTava.setPosition(SERVO_TAVA_CATCH_POSITION); servoPos2 = true;}

                }
                if(gamepad1.left_bumper){

                    if(servoPos2 == true) { servoTava.setPosition(SERVO_TAVA_ZERO_POSITION); servoPos2 = false;}

                }

                // Motor glisiera
                /*if(motorGlisiera.getCurrentPosition() < LIMIT_MIN_GLISIERA
                        && motorGlisiera.getCurrentPosition() > LIMIT_MAX_GLISIERA){

                    motorGlisiera.setPower(Range.clip(gamepad2.left_stick_y,-SPEED_GLISIERA,SPEED_GLISIERA));

                }
                else if(motorGlisiera.getCurrentPosition() >= LIMIT_MIN_GLISIERA){

                    motorGlisiera.setPower(Range.clip(gamepad2.left_stick_y,-SPEED_GLISIERA,0));

                }
                else if(motorGlisiera.getCurrentPosition() <= LIMIT_MAX_GLISIERA){

                    motorGlisiera.setPower(Range.clip(gamepad2.left_stick_y,0,SPEED_GLISIERA));

                }*/

                // Motor lift
                if(motorLift.getCurrentPosition() < LIMIT_MIN_LIFT && motorLift.getCurrentPosition() > LIMIT_MAX_LIFT){

                    motorLift.setPower(Range.clip(gamepad2.right_stick_y * SPEED_LIFT,-SPEED_LIFT,SPEED_LIFT));

                }
                else if(motorLift.getCurrentPosition() >= LIMIT_MIN_LIFT){

                    motorLift.setPower(Range.clip(gamepad2.right_stick_y * SPEED_LIFT,-SPEED_LIFT,0));

                }
                else if(motorLift.getCurrentPosition() <= LIMIT_MAX_LIFT){

                    motorLift.setPower(Range.clip(gamepad2.right_stick_y * SPEED_LIFT,0,SPEED_LIFT));

                }
                Thread.yield();
                // Control
                double joyangle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                joyangle -= Math.PI / 2;
                double speed = (Math.sqrt(Math.pow(-gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)));
                double rspeed = gamepad1.right_stick_x * 0.5;
                go.move(speed, joyangle, rspeed);

                long timeMilis = 120000 - Math.abs(TimeUnit.MILLISECONDS.convert(time - System.nanoTime(), TimeUnit.NANOSECONDS));

                telemetry.addData("GhearaBrick: ", (servoPos1 == true) ? "ON" : "OFF");
                telemetry.addData("GhearaTava: ", (servoPos2 == true) ? "ON" : "OFF");
                telemetry.addData("Butt_Tava", (buttState) ? "ON" : "OFF");
                telemetry.addData("Mins: ",timeMilis/60000);
                telemetry.addData("Secs: ",timeMilis/1000 %60);
                telemetry.update();
                Thread.yield();

                if (isStopRequested()) throw new InterruptedException();

            }
            throw new InterruptedException();

        }
        catch (InterruptedException e) {
            servoTava.setPosition(0);
            servo_pickup.setPosition(0);
            go.killSwitch();
        }

    }
}
