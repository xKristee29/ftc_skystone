package testing.fat_guy;

import android.graphics.Color;
import android.util.Log;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;
@Disabled
@Autonomous(name = "Detectie cub",group = "Complet")
public class Detectie extends LinearOpMode {

    private HardwareConfig hardware = null;
    private TouchSensor buttFront, buttRear;
    private ColorSensor colorSensor = null;
    private ColorSensor colorSensorBlock;
    private Orientation r_mat = null;
    private Servo servoPickup = null;

    private int colorHSV;
    private float sat;
    private float val;

    private double frontWall = 0;
    private double leftWall = 0;

    private void initHW(){

        // Init
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        buttFront = hardwareMap.get(TouchSensor.class, "btnTavaA");
        buttRear = hardwareMap.get(TouchSensor.class, "btnTavaB");
        colorSensor = hardwareMap.get(ColorSensor.class, "sc2");
        colorSensorBlock = hardwareMap.get(ColorSensor.class, "sc1");
        hardware.servoTavaA.setPosition(0);
        servoPickup = hardwareMap.get(Servo.class, "servoSkystone");


    }

    @Override
    public void runOpMode() throws InterruptedException {
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(imuParams);
        while (!Thread.interrupted() && !imu.isGyroCalibrated()){
            Thread.yield();
        }
        // Init
        initHW();

        // Wait for start the autonomous period
        waitForStart();

        // Run
        try{

            double x, y, rotationSpeed;
            double imuAngle, targetAngle;

            targetAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            frontWall = 70;
            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested()) throw new InterruptedException();

                x = 0.5;
                y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 300, 520, 0);
                r_mat = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                imuAngle = -r_mat.thirdAngle;
                rotationSpeed = 1.3 * (imuAngle - targetAngle)/180;

                colorHSV = Color.argb(colorSensorBlock.alpha(), colorSensorBlock.red(), colorSensorBlock.green(), colorSensorBlock.blue());
                sat = JavaUtil.colorToSaturation(colorHSV);
                val = JavaUtil.colorToValue(colorHSV);
                double hue = JavaUtil.colorToHue(colorHSV);

                if((hue < 30 || hue > 60)) {
                    Log.v("WILLDBG_HUE", Double.toString(hue));
                    servoPickup.setPosition(0.53);
                    telemetry.addData("hue",hue);break;
                }



                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            sleep(1000);
            hardware.mecanum.moveXYR(0,-0.2, 0);
            sleep(2000);
            hardware.mecanum.moveXYR(-0.5,0, 0);
            sleep(2000);
            hardware.mecanum.killSwitch();
            throw new InterruptedException();

        }
        catch(InterruptedException e){

            // Stop the robot if his job it's done
            // Or if it requested to stop from app
            hardware.mecanum.killSwitch();
            hardware.servoTavaA.setPosition(-0.16);
            sleep(1000);

        }
    }
}
