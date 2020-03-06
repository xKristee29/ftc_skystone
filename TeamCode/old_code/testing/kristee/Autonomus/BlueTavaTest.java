package testing.kristee.Autonomus;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@Autonomous(name = "BlueTavaTest")
public class BlueTavaTest extends LinearOpMode {

    private HardwareConfig hardware = null;
    private TouchSensor buttFront, buttRear;
    private ColorSensor colorSensor = null;
    private Orientation r_mat = null;

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

            // Go to building site
            leftWall  = 400;
            frontWall = 360;
            targetAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested()) throw new InterruptedException();

                x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.25);
                y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 400, 520, 0.25);
                r_mat = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                imuAngle = -r_mat.thirdAngle;
                rotationSpeed = 1.3 * (imuAngle - targetAngle)/180;

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
            targetAngle = targetAngle - 90;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = 0;
                x = -0.5;
                r_mat = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                imuAngle = -r_mat.thirdAngle;
                rotationSpeed = 2 * (imuAngle - targetAngle)/180;
                if(rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            hardware.servoTavaA.setPosition(-0.16);
            hardware.mecanum.moveXYR(0,0.7,0);
            sleep(500);
            hardware.mecanum.killSwitch();
            //------------------------------------------------------------------------------------//

            // Go under sky bridge
            frontWall = 700;
            leftWall = 650;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 350, 300, 0.25);
                x = hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 350, 300, 0.25);
                r_mat = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                imuAngle = -r_mat.thirdAngle;
                rotationSpeed = 2 * (imuAngle - targetAngle)/180;
                if(x == 0 && y == 0 && rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }

            targetAngle = targetAngle - 90;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                r_mat = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                imuAngle = -r_mat.thirdAngle;
                rotationSpeed = 2 * (imuAngle - targetAngle)/180;
                if(rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(0, 0, rotationSpeed);

                Thread.yield();

            }

            hardware.mecanum.moveXYR(0, 0.6, 0);
            sleep(1000);
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
