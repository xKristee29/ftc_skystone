package testing.kristee.Autonomus.Autonoame;

import android.graphics.Color;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.apis.testing.HxIMU;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@Autonomous(name = "BlueBrick", group = "Complet")
public class BlueBrick extends LinearOpMode {

    private HardwareConfig hardware = null;
    private ColorSensor colorSensorLine = null,
                        colorSensorBlock = null;
    private BNO055IMU imu = null;
    private BNO055IMU.Parameters imuParams = null;
    private HxIMU hxIMU = null;


    private int colorHSV;
    private float sat;
    private float val;

    private double frontWall;
    private byte brick;

    public void initHW(){

        colorSensorBlock = hardwareMap.get(ColorSensor.class,"sc1");
        colorSensorLine = hardwareMap.get(ColorSensor.class, "sc2");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        hardware.servoBrick.setPosition(0);

    }

    public final void sleepImuHold(long milliseconds, double x, double y, double r, double targetAngle) {
        long currentTime = System.currentTimeMillis();
        try {
            while (System.currentTimeMillis() - currentTime <= milliseconds) {
                Thread.yield();
                if(Thread.interrupted())throw new InterruptedException();
                hardware.mecanum.moveXYR(x, y, r + hxIMU.getCorrection(targetAngle));
            }
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initHW();

        double x,y,rotationSpeed;
        double targetAngle = hxIMU.getValue();

        hardware.localization.update();

        telemetry.addData("dist: ", hardware.localization.distanceFront.lastReadout);
        telemetry.update();

        waitForStart();

        try{

            frontWall = 55;
            while (!isStopRequested()) {

                hardware.localization.update();

                telemetry.addData("dist: ", hardware.localization.distanceFront.lastReadout);
                telemetry.update();

                if(isStopRequested()) throw new InterruptedException();

                x = 0;
                y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 300, 520, 0) * 0.8;
                rotationSpeed = hxIMU.getCorrection(targetAngle);

                colorHSV = Color.argb(colorSensorBlock.alpha(), colorSensorBlock.red(), colorSensorBlock.green(), colorSensorBlock.blue());
                sat = JavaUtil.colorToSaturation(colorHSV);
                val = JavaUtil.colorToValue(colorHSV);

                if(y <= 0.1 && hardware.localization.distanceFront.lastReadout > 30) break;


                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            sleep(500);

            long brickSwitch = 700;
            long podBrick = 3000;
            long parkLine = 800;
            long extractBrick = 300;

            brick = 0;
            switch (brick){

                case 0:
                    while (!isStopRequested()) {

                        hardware.localization.update();

                        telemetry.addData("dist: ", hardware.localization.distanceFront.lastReadout);
                        telemetry.update();

                        if(isStopRequested()) throw new InterruptedException();

                        x = 0;
                        y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 300, 520, 0) * 0.8;
                        rotationSpeed = hxIMU.getCorrection(targetAngle);

                        colorHSV = Color.argb(colorSensorBlock.alpha(), colorSensorBlock.red(), colorSensorBlock.green(), colorSensorBlock.blue());
                        sat = JavaUtil.colorToSaturation(colorHSV);
                        val = JavaUtil.colorToValue(colorHSV);

                        if(y <= 0.1 && hardware.localization.distanceFront.lastReadout > 30) break;


                        hardware.mecanum.moveXYR(x, y, rotationSpeed);

                        Thread.yield();

                    }
                    val = 0;
                    for (int i=1;i<=5;i++){
                        colorHSV = Color.argb(colorSensorBlock.alpha(), colorSensorBlock.red(), colorSensorBlock.green(), colorSensorBlock.blue());
                        val += JavaUtil.colorToValue(colorHSV);
                    }
                    val /= 5;

                    if (val<=0.35) break;

                    brick++;

                    sleepImuHold(brickSwitch, 0.7,0,0,targetAngle);
                    hardware.mecanum.killSwitch();

                case 1:
                    while (!isStopRequested()) {

                        hardware.localization.update();

                        telemetry.addData("dist: ", hardware.localization.distanceFront.lastReadout);
                        telemetry.update();

                        if(isStopRequested()) throw new InterruptedException();

                        x = 0;
                        y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 300, 520, 0) * 0.8;
                        rotationSpeed = hxIMU.getCorrection(targetAngle);

                        colorHSV = Color.argb(colorSensorBlock.alpha(), colorSensorBlock.red(), colorSensorBlock.green(), colorSensorBlock.blue());
                        sat = JavaUtil.colorToSaturation(colorHSV);
                        val = JavaUtil.colorToValue(colorHSV);

                        if(y <= 0.1 && hardware.localization.distanceFront.lastReadout > 30) break;


                        hardware.mecanum.moveXYR(x, y, rotationSpeed);

                        Thread.yield();

                    }
                    val = 0;
                    for (int i=1;i<=5;i++){
                        colorHSV = Color.argb(colorSensorBlock.alpha(), colorSensorBlock.red(), colorSensorBlock.green(), colorSensorBlock.blue());
                        val += JavaUtil.colorToValue(colorHSV);
                    }
                    val /= 5;

                    if (val<=0.35) break;

                    brick++;

                    sleepImuHold(brickSwitch, 0.7,0,0,targetAngle);
                    hardware.mecanum.killSwitch();

            }
            hardware.servoBrick.setPosition(1);
            sleep(1000);

            sleepImuHold(extractBrick, 0,-0.7,0,targetAngle);

            sleepImuHold(podBrick+brick*brickSwitch, -0.7,0,0,targetAngle);

            hardware.mecanum.killSwitch();
            hardware.servoBrick.setPosition(0);
            sleep(1000);

            sleepImuHold(podBrick+(brick+2)*brickSwitch*105/100, 0.7,0,0,targetAngle);

            while (!isStopRequested()) {

                hardware.localization.update();

                telemetry.addData("dist: ", hardware.localization.distanceFront.lastReadout);
                telemetry.update();

                if(isStopRequested()) throw new InterruptedException();

                x = 0;
                y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 300, 520, 0) * 0.8;
                rotationSpeed = hxIMU.getCorrection(targetAngle);

                colorHSV = Color.argb(colorSensorBlock.alpha(), colorSensorBlock.red(), colorSensorBlock.green(), colorSensorBlock.blue());
                sat = JavaUtil.colorToSaturation(colorHSV);
                val = JavaUtil.colorToValue(colorHSV);

                if(y <= 0.1 && hardware.localization.distanceFront.lastReadout > 30) break;


                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }

            hardware.mecanum.killSwitch();
            hardware.servoBrick.setPosition(1);
            sleep(1000);

            sleepImuHold(extractBrick, 0,-0.7,0,targetAngle);

            sleepImuHold(podBrick+(brick+3)*brickSwitch, -0.7,0,0,targetAngle);

            hardware.mecanum.killSwitch();
            hardware.servoBrick.setPosition(0);
            sleep(1000);

            sleepImuHold(parkLine, 0.7,0,0,targetAngle);

            throw new InterruptedException();
        }
        catch (InterruptedException e){

        }
    }
}
