package org.firstinspires.ftc.teamcode.opmodes.stable;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.apis.testing.HxIMU;
import org.firstinspires.ftc.teamcode.apis.testing.Localization;
import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;
import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class HardwareConfigCT {

    // Declaram motoarele
    public Mecanum mecanum = null;
    public DcMotor leftFront = null;
    public DcMotor rightFront = null;
    public DcMotor leftRear = null;
    public DcMotor rightRear = null;

    // Declaram senzorii de distanta
    public DistanceSensor sd1 = null;
    public DistanceSensor sd2 = null;
    public DistanceSensor sd3 = null;
    public DistanceSensor sd4 = null;
    public DistanceSensor sd5 = null;
    public DistanceSensor sd6 = null;

    public Servo servoTavaFront = null;
    public Servo servoTavaRear = null;

    public TouchSensor switchBuildPlateFront;
    public TouchSensor switchBuildPlateRear;
    public TouchSensor swFrontLeft,
                       swFrontRight,
                       swRearLeft,
                       swRearRight;

    //LIFT
    public DcMotor motorLift;
    public TouchSensor liftHomingSwitch;
    final int LIMIT_MIN_LIFT = -40;
    final int LIMIT_MAX_LIFT = -5800;
    final double SPEED_LIFT = 0.5;
    public Servo servoLiftRaise;

    //TURRET
    public DcMotor motorTurret;
    public TouchSensor turretHomingSwitch;
    final int LIMIT_MIN_TURRET = -40;
    final int LIMIT_MAX_TURRET = -2400;
    final double SPEED_TURRET = 0.5;

    //SLIDER
    public Servo servoSlider;
    public TouchSensor swSliderFront, swSliderRear;
    public DcMotor motorRuleta;

    //PUMP
    public DcMotor motorPump;
    public TouchSensor swPump;

    public TouchSensor swBrick;

    public HxIMU gyro;

    public double SPEED_MECANUM_X = 0.6;
    public double SPEED_MECANUM_Y = 0.5;
    public double SPEED_MECANUM_ROTATION = 0.3;
    public int turretPos = 0;
    public boolean isTurretActive = false;

    public Localization localization = null;

    public void initHardware(HardwareMap hardwareMap) {
        localization = new Localization();

        // Initializarea senzorilor de distanta
        sd1 = hardwareMap.get(VL53L0X.class, "sd1");
        sd2 = hardwareMap.get(VL53L0X.class, "sd2");
        sd3 = hardwareMap.get(VL53L0X.class, "sd3");
        sd4 = hardwareMap.get(VL53L0X.class, "sd4");
        sd5 = hardwareMap.get(VL53L0X.class, "sd5");
        sd6 = hardwareMap.get(VL53L0X.class, "sd6");
        localization.init(sd1, sd2, sd3, sd4, sd5, sd6);

        // Initializarea motoarelor
        leftFront = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        rightFront = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        leftRear = hardwareMap.get(DcMotor.class,"leftRearMotor");
        rightRear = hardwareMap.get(DcMotor.class,"rightRearMotor");

        mecanum = new Mecanum();

        mecanum.init(leftFront,
                rightFront,
                leftRear,
                rightRear);

        servoTavaFront = hardwareMap.get(Servo.class, "servoTavaFront");
        servoTavaRear = hardwareMap.get(Servo.class, "servoTavaRear");

        switchBuildPlateFront = hardwareMap.get(TouchSensor.class, "swTavaFront");
        switchBuildPlateRear = hardwareMap.get(TouchSensor.class, "swTavaRear");

        swFrontLeft = hardwareMap.get(TouchSensor.class, "swFrontLeft");
        swFrontRight = hardwareMap.get(TouchSensor.class, "swFrontRight");
        swRearLeft = hardwareMap.get(TouchSensor.class, "swRearLeft");
        swRearRight = hardwareMap.get(TouchSensor.class, "swRearRight");

        //LIFT
        liftHomingSwitch = hardwareMap.get(TouchSensor.class, "swLift");
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLift.setTargetPosition(0);
        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ((DcMotorEx)motorLift).setTargetPositionTolerance(0);
        ((DcMotorEx)motorLift).setVelocityPIDFCoefficients(2.5, 0.5, 0, 11.0);
        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        servoLiftRaise = hardwareMap.get(Servo.class, "servoLiftRaise");
        raiseLiftServo();

        //TURRET
        turretHomingSwitch = hardwareMap.get(TouchSensor.class, "swTurela");
        motorTurret = hardwareMap.get(DcMotor.class, "motorTurela");
        motorTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTurret.setTargetPosition(0);
        ((DcMotorEx) motorTurret).setTargetPositionTolerance(0);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(0.7);
        //motorTurret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //SLIDER
        servoSlider = hardwareMap.get(Servo.class, "servoGlisiera");
        swSliderFront = hardwareMap.get(TouchSensor.class, "swGlisieraFront");
        swSliderRear = hardwareMap.get(TouchSensor.class, "swGlisieraRear");

        motorRuleta = hardwareMap.get(DcMotor.class, "motorRuleta");

        //PUMP
        motorPump = hardwareMap.get(DcMotor.class, "motorPompa");
        motorPump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorPump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorPump.setPower(0);
        swPump = hardwareMap.get(TouchSensor.class, "swPompa");

        swBrick = hardwareMap.get(TouchSensor.class,"swBrick");
    }

    //LIFT
    public int currentLiftPos = 0;
    public DcMotor.RunMode liftRunMode = DcMotor.RunMode.RUN_TO_POSITION;
    public boolean isLiftHoming = false;

    public void setLiftPower(double value) {
        double power = value;
        if(power != 0) {
            liftRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
            motorLift.setMode(liftRunMode);
            currentLiftPos = motorLift.getCurrentPosition();
        }
        if(power > 0 && currentLiftPos < LIMIT_MIN_LIFT) {
            motorLift.setPower(power * SPEED_LIFT);
        }
        else if(power < 0 && currentLiftPos > LIMIT_MAX_LIFT) {
            motorLift.setPower(power * SPEED_LIFT);
        }
        else {
            motorLift.setMode(liftRunMode);
            motorLift.setPower(0);
        }
    }

    public void setLiftLevel(int value) {
        liftRunMode = DcMotor.RunMode.RUN_TO_POSITION;
        currentLiftPos = value;
        if(value < LIMIT_MIN_LIFT && value > LIMIT_MAX_LIFT){
            motorLift.setTargetPosition(value);
            motorLift.setPower(1);
            motorLift.setMode(liftRunMode);
            motorLift.setPower(1);
        }
        else liftRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
    }

    public void startLiftHoming() {
        if(!liftHomingSwitch.isPressed()) {
            isLiftHoming = true;
            liftRunMode = DcMotor.RunMode.RUN_USING_ENCODER;
            motorLift.setMode(liftRunMode);
            motorLift.setPower(0.7);
        }
    }

    public boolean isPumpHoming = false;

    public void sanityCheck() {
        if(isLiftHoming && liftHomingSwitch.isPressed()) {
            motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorLift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorLift.setPower(0);
            setLiftPower(0);
            isLiftHoming = false;
            currentLiftPos = 0;
        }
        if(isPumpHoming && swPump.isPressed()) {
            motorPump.setPower(0);
            motorPump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorPump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isPumpHoming = false;
        }
        if(isPumpHoming && motorPump.getCurrentPosition() <= 0) {
            motorPump.setPower(0);
            motorPump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isPumpHoming = false;
        }
        if(isPumpHoming && ((ExpansionHubMotor)motorPump).getCurrentDraw(ExpansionHubEx.CurrentDrawUnits.AMPS) > 6) {
            motorPump.setPower(0);
            motorPump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorPump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            isPumpHoming = false;
        }
    }

    public void grabBuildPlate() {
        if(switchBuildPlateRear.isPressed() && switchBuildPlateFront.isPressed()) {
            servoTavaFront.setPosition(0.3);
            servoTavaRear.setPosition(0.7);
        }
    }

    public void grabBuildPlateNoBtn() {
        servoTavaFront.setPosition(0.3);
        servoTavaRear.setPosition(0.7);
    }

    public void releaseBuildPlate() {
        servoTavaFront.setPosition(1);
        servoTavaRear.setPosition(0);
    }

    public void raiseLiftServo() {
        servoLiftRaise.setPosition(0.555);
    }

    public void lowerLiftServo() {
        servoLiftRaise.setPosition(1);
    }

    public void startPumpHoming() {
        if(!swPump.isPressed()){
            motorPump.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorPump.setPower(-0.3);
            isPumpHoming = true;
        }
        else{
            motorPump.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public void startSuction() {
        if(!isPumpHoming && swPump.isPressed()) {
            motorPump.setTargetPosition(250);
            motorPump.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorPump.setPower(1);
        }
    }

    public void setSliderPower(double power) {
        if(power < 0 && !(swSliderRear.isPressed())) servoSlider.setPosition((power + 1) / 2);
        else if(power > 0 && !(swSliderFront.isPressed())) servoSlider.setPosition((power + 1) / 2);
        else servoSlider.setPosition(0.5);
    }

    public void rotateTurretLeft() {
        motorTurret.setPower(0);
        motorTurret.setTargetPosition(-660);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(1);
        turretPos = -660;
        isTurretActive = true;
    }

    public void rotateTurretRight() {
        motorTurret.setPower(0);
        motorTurret.setTargetPosition(660);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(1);
        turretPos = 660;
        isTurretActive = true;
    }

    public void rotateTurretCenter() {
        motorTurret.setPower(0);
        motorTurret.setTargetPosition(0);
        motorTurret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorTurret.setPower(1);
        turretPos = 0;
        isTurretActive = true;
    }

    public void setTurretPower(double speed){
        if(speed != 0 && currentLiftPos < -60) {
            motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorTurret.setPower(speed);
        }
        else {
            motorTurret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorTurret.setPower(0);
        }
    }

    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void setTapePower(double power) {
        motorRuleta.setPower(- power);
    }
}
