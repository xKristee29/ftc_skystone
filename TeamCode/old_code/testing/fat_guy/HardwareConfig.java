package testing.fat_guy;

import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.apis.testing.Localization;
import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;
import org.firstinspires.ftc.teamcode.apis.testing.SkystoneMotionPlayer;

public class HardwareConfig {
    // Declaram motoarele
    Mecanum mecanum = null;
    DcMotor leftFront = null;
    DcMotor rightFront = null;
    DcMotor leftRear = null;
    DcMotor rightRear = null;

    // Declaram senzorii de distanta
    DistanceSensor sd1 = null;
    DistanceSensor sd2 = null;
    DistanceSensor sd3 = null;
    DistanceSensor sd4 = null;
    DistanceSensor sd5 = null;
    DistanceSensor sd6 = null;

    Servo servoTava = null;

    Localization localization = null;
    SkystoneMotionPlayer motionPlayer = null;

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

        servoTava = hardwareMap.get(Servo.class, "servo_tava");

        mecanum.init(leftFront,
                rightFront,
                leftRear,
                rightRear);

        motionPlayer = new SkystoneMotionPlayer(localization);
    }
}
