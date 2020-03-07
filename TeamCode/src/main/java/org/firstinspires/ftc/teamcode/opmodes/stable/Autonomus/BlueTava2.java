package org.firstinspires.ftc.teamcode.opmodes.stable.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@Autonomous(name = "BlueTava1", group = "Complet")
public class BlueTava2 extends LinearOpMode {

    private HardwareConfig hardware = null;

    private void initHW(){
        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        hardware.releaseBuildPlate();
        telemetry.addData(">>", "Press play to pay respect");
        telemetry.update();
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initHW();

        waitForStart();

        try{

            double x,y,rotationSpeed;
            double targetAngle = hardware.gyro.getValue();
            double leftWall,frontWall;

            ///////////////////////////////////////////////////
            leftWall  = 400;
            frontWall = 360;
            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested()) throw new InterruptedException();

                x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.6)*0.5;
                y = + hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 400, 520, 0.3)*0.5;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);

                if(x <= 0.15 && y <= 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            ////////////////////////////////////////////////////
            while (!isStopRequested() && !(hardware.switchBuildPlateRear.isPressed() && hardware.switchBuildPlateFront.isPressed())) {

                x = 0.5;
                y = 0;
                rotationSpeed = 0;

                if(isStopRequested()) throw new InterruptedException();



                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.grabBuildPlateNoBtn();
            hardware.mecanum.killSwitch();
            sleep(1000);

            if (isStopRequested()) throw new InterruptedException();

            if (isStopRequested()) throw new InterruptedException();
            ////////////////////////////////////////////////////
            leftWall = 150;
            frontWall = 360;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 350, 300, 0.25)*0.5;
                x = (hardware.localization.distanceFrontLeft.lastReadout > leftWall) ? -0.5 : 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(x == 0 && y == 0 && rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            hardware.releaseBuildPlate();
            sleep(500);
            ////////////////////////////////////////////////////
            leftWall = 100;
            frontWall = 1100;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = hardware.localization.getCorrectionSpeedFrontDistance(frontWall, 350, 300, 0.25)*0.5;
                x = 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(x == 0 && y == 0 && rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                Thread.yield();

            }

            hardware.mecanum.moveXYR(0, -0.4, 0);
            sleep(1000);

            throw new InterruptedException();
        }
        catch (InterruptedException e){

            hardware.mecanum.killSwitch();
            hardware.releaseBuildPlate();
        }

    }
}
