package org.firstinspires.ftc.teamcode.opmodes.stable.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

@Autonomous(name = "RedTava1", group = "Complet")
public class RedTava1 extends LinearOpMode {

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
            double leftWall,rearWall;

            ///////////////////////////////////////////////////
            leftWall  = 400;
            rearWall = 200;
            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested()) throw new InterruptedException();

                x = - hardware.localization.getCorrectionSpeedLeftDistance(leftWall, 400, 520, 0.6)*0.5;
                y = - hardware.localization.getCorrectionSpeedRearDistance(rearWall, 400, 400, 0.1)*0.5;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);

                if(x <= 0.15 && y <= 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("SD4: ",hardware.localization.distanceRear);
                telemetry.update();

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
            rearWall = 300;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = - hardware.localization.getCorrectionSpeedRearDistance(rearWall, 350, 400, 0.25)*0.5;
                x = (hardware.localization.distanceFrontLeft.lastReadout > leftWall) ? -0.5 : 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(x == 0 && y == 0 && rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("SD4: ",hardware.localization.distanceRear);
                telemetry.update();

                Thread.yield();

            }
            hardware.mecanum.killSwitch();
            hardware.releaseBuildPlate();
            sleep(500);
            ////////////////////////////////////////////////////
            leftWall = 100;
            rearWall = 900;

            while (!isStopRequested()) {

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = - hardware.localization.getCorrectionSpeedRearDistance(rearWall, 350, 400, 0.25)*0.5;
                x = 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(x == 0 && y == 0 && rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("SD4: ",hardware.localization.distanceRear);
                telemetry.update();

                Thread.yield();

            }

            hardware.mecanum.moveXYR(0.5, 0, 0);
            sleep(2400);
            hardware.mecanum.moveXYR(0, 0.4, 0);
            sleep(1000);

            throw new InterruptedException();
        }
        catch (InterruptedException e){

            hardware.mecanum.killSwitch();
            hardware.releaseBuildPlate();
        }

    }
}
