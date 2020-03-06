package testing.fat_guy;

//Facut de un gras

import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDIO;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.apis.testing.Localization;
import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;
@Disabled
@TeleOp(name = "WILL - autonom_perete")
public class FatGuyAutonom extends OpMode {

    // Declaram motoarele
    Mecanum go = null;
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


    Localization localization = null;

    @Override
    public void init(){
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

        // Setam directia motoarelor stanga
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        go = new Mecanum();

        go.init(leftFront,
                rightFront,
                leftRear,
                rightRear);
    }

    private double speed = 1;
    private double sens = 1;

    @Override
    public void loop (){

        // Citim distantele
        localization.update();

        // Pornim motoarele cu viteza declarata si corectarea directiei
        double correction_fr = 0;
        if(sens < 0) {
            correction_fr =  - localization.getCorrectionSpeedRearDistance(200);
        }
        else if(sens > 0) {
            correction_fr =  localization.getCorrectionSpeedFrontDistance(200);
        }

        if(correction_fr == 0) {
            sens = - sens;
        }

        double  x = Range.clip(localization.getCorrectionSpeedRightDistance(200), -1, 1),
                y = Range.clip(sens + correction_fr, -1, 1);
        go.move(speed * Math.sqrt(x * x + y * y),Math.atan2(y, x) - (Math.PI / 2),localization.getAngleCorrectionRight());

    }
}

