package testing.kristee.Autonomus.Autonoame;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.apis.testing.HxIMU;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@Autonomous(name = "BlueTava2",group = "Complet")
public class BlueTava2 extends LinearOpMode {

    private HardwareConfig hardware = null;
    private TouchSensor buttFront, buttRear;
    private ColorSensor colorSensor = null;
    private BNO055IMU imu = null;
    private BNO055IMU.Parameters imuParams = null;
    private HxIMU hxIMU = null;

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
        hardware.servoTavaA.setPosition(0);


    }

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(BNO055IMU.class, "imu 1");
        imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(imuParams);
        while (!Thread.interrupted() && !imu.isGyroCalibrated()){
            Thread.yield();
        }
        hxIMU = new HxIMU();
        hxIMU.init(imu);
        // Init
        initHW();

        // Wait for start the autonomous period
        waitForStart();

        // Run
        try{

            double x, y, rotationSpeed;
            double imuAngle, targetAngle;

            // Go to building site
            leftWall  = 400;
            frontWall = 360;
            targetAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested()) throw new InterruptedException();

                x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.25);
                y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 400, 520, 0.25);
                rotationSpeed = hxIMU.getCorrection(targetAngle);

                if(x == 0 && y == 0) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            //------------------------------------------------------------------------------------//


            // Close up to building site and hang the building site
            while (!isStopRequested() && !(buttFront.isPressed() && buttRear.isPressed())) {

                x = 0.4;
                y = 0;
                rotationSpeed = 0;

                if(isStopRequested()) throw new InterruptedException();

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();

            if (isStopRequested()) throw new InterruptedException();

            hardware.servoTavaA.setPosition(0.16);
            sleep(1000);

            if (isStopRequested()) throw new InterruptedException();
            //------------------------------------------------------------------------------------//


            // Go with building site in building corner
            leftWall = 120;
            frontWall = 250;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 350, 300, 0.25);
                x = (hardware.localization.distanceFrontLeft.lastReadout > leftWall) ? -0.7 : 0;
                rotationSpeed = hxIMU.getCorrection(targetAngle);
                if(x == 0 && y == 0 && rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            hardware.servoTavaA.setPosition(-0.16);
            sleep(1000);
            //------------------------------------------------------------------------------------//

            // Go under sky bridge
            leftWall = 100;
            frontWall = 800;

            hardware.mecanum.move(-0.4,0,0);
            sleep(2000);

            colorHSV = Color.argb(colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue());
            sat = JavaUtil.colorToSaturation(colorHSV);
            val = JavaUtil.colorToValue(colorHSV);

            while (!isStopRequested() && (sat <= 0.4 && val <= 0.6)) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = -0.4;
                x = (hardware.localization.distanceFrontLeft.lastReadout > leftWall) ? -0.4 : 0;
                rotationSpeed = 0;

                colorHSV = Color.argb(colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue());
                sat = JavaUtil.colorToSaturation(colorHSV);
                val = JavaUtil.colorToValue(colorHSV);

                if(sat > 0.4 && val > 0.55) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);



                Thread.yield();

                telemetry.addData("X: ", x);
                telemetry.addData("Y: ", y);
                telemetry.update();

            }
            hardware.mecanum.killSwitch();

            hardware.mecanum.moveXYR(0, 0.6, 0);
            sleep(500);
            //------------------------------------------------------------------------------------//

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
