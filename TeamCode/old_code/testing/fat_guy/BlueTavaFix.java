package testing.fat_guy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;
@Disabled
@Autonomous(name = "BlueTavaFIX")
public class BlueTavaFix extends LinearOpMode {


    private HardwareConfig hardware = null;
    private TouchSensor buttFront, buttRear;

    private int index = 0;
    private double frontWall = 0;
    private double leftWall = 0;
    private double speed = 0.8;

    private void next(){

        index = index + 1;

    }

    private void initHW(){

        // Init
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        buttFront = hardwareMap.get(TouchSensor.class, "btnTavaA");
        buttRear = hardwareMap.get(TouchSensor.class, "btnTavaB");
        hardware.servoTavaA.setPosition(0);


    }

    @Override
    public void runOpMode() throws InterruptedException {

        // Init
        initHW();

        // Wait for start the autonomous period
        waitForStart();

        // Run
        try{

            double x, y, rotationSpeed;

            /*switch (index){

                case 0:

                    // Go to building site
                    frontWall = 350;
                    leftWall  = 400;

                    while (isStopRequested()) {

                        hardware.localization.update();

                        if(isStopRequested()) throw new InterruptedException();

                        x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.25);
                        y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 400, 520, 0.25);
                        rotationSpeed = hardware.localization.getAngleCorrectionLeft();

                        if(x == 0 && y == 0 && rotationSpeed == 0) break;

                        hardware.mecanum.moveXYR(x, y, rotationSpeed);

                        Thread.yield();

                    }
                    hardware.mecanum.killSwitch();

                    next();
                    break;

                case 1:

                    // Close up to building site and hang the building site
                    while (isStopRequested() && !(buttFront.isPressed() && buttRear.isPressed())) {

                        y = 0.3;
                        x = 0;
                        rotationSpeed = 0;

                        if(isStopRequested()) throw new InterruptedException();

                        hardware.mecanum.moveXYR(x, y, rotationSpeed);

                        Thread.yield();

                    }
                    hardware.mecanum.killSwitch();

                    if (isStopRequested()) throw new InterruptedException();

                    hardware.servoTava.setPosition(0.2);
                    sleep(1000);

                    if (isStopRequested()) throw new InterruptedException();

                    next();
                    break;

                case 2:

                    // Go with building site in building corner
                    leftWall = 100;

                    while (isStopRequested()) {

                        hardware.localization.update();

                        if(isStopRequested()) throw new InterruptedException();

                        x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.25);
                        y = (hardware.localization.distanceFrontLeft.lastReadout > leftWall) ? -0.8 : 0;
                        rotationSpeed = hardware.localization.getAngleCorrectionLeft();

                        if(x == 0 && y == 0 && rotationSpeed == 0) break;

                        hardware.mecanum.moveXYR(x, y, rotationSpeed);

                        Thread.yield();

                    }
                    hardware.mecanum.killSwitch();

                    next();
                    break;

                case 3: throw new InterruptedException();

            }*/

            // Go to building site
            leftWall  = 400;
            frontWall = 350;

            while (isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested()) throw new InterruptedException();

                x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.25);
                y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 400, 520, 0.25);
                rotationSpeed = hardware.localization.getAngleCorrectionLeft();

                if(x == 0 && y == 0 && rotationSpeed == 0) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            //------------------------------------------------------------------------------------//


            // Close up to building site and hang the building site
            while (isStopRequested() && !(buttFront.isPressed() && buttRear.isPressed())) {

                x = 0.3;
                y = 0;
                rotationSpeed = 0;

                if(isStopRequested()) throw new InterruptedException();

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();

            if (isStopRequested()) throw new InterruptedException();

            hardware.servoTavaA.setPosition(0.2);
            sleep(1000);

            if (isStopRequested()) throw new InterruptedException();
            //------------------------------------------------------------------------------------//


            // Go with building site in building corner
            leftWall = 100;

            while (isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.25);
                y = (hardware.localization.distanceFrontLeft.lastReadout > leftWall) ? -0.8 : 0;
                rotationSpeed = hardware.localization.getAngleCorrectionLeft();

                if(x == 0 && y == 0 && rotationSpeed == 0) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            //------------------------------------------------------------------------------------//

            throw new InterruptedException();

        }
        catch(InterruptedException e){

            // Stop the robot if his job it's done
            // Or if it requested to stop from app
            hardware.mecanum.killSwitch();
            hardware.servoTavaA.setPosition(0);
            sleep(1000);

        }
    }
}
