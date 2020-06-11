package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotParams;
import org.firstinspires.ftc.teamcode.opmodes.stable.HardwareConfig;

import java.util.List;

@TeleOp(name = "TFODtest")
public class WebCamTest extends LinearOpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = RobotParams.VUFORIA_LICENSE_KEY;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private HardwareConfig hardware = null;

    private int skystonePos = -1;

    private double stoneDistRight[] = {1040,0,0,0,0,0};
    private double stoneDistR = 800;

    public void initHW(){

        hardware = new HardwareConfig();
        hardware.initHardware(hardwareMap);
        hardware.startPumpHoming();
        hardware.startLiftHoming();

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        while (skystonePos == -1) {
            if (tfod != null) {
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    // step through the list of recognitions and display boundary info.
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel() == "Skystone"){
                            double posLeft = recognition.getLeft();
                            double posRight = recognition.getRight();
                            double width = (posRight - posLeft)/10;

                            if(posLeft + 3 * width > 320)
                                 skystonePos =  1;
                            else if(posLeft + 7 * width < 320)
                                 skystonePos =  0;
                            else skystonePos =  2;
                        }
                    }
                }
            }
            else skystonePos = -2;
        }

        if (tfod != null) {
            tfod.shutdown();
        }

        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

    }

    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

    }

    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.8;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    @Override
    public void runOpMode() throws InterruptedException{

        initHW();

        waitForStart();

        try{

            double x,y,rotationSpeed;
            double targetAngle = hardware.gyro.getValue();
            double rightWall,rearWall;

            rearWall = stoneDistR;
            rightWall = stoneDistRight[skystonePos];

            while(!isStopRequested()){

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = - hardware.localization.getCorrectionSpeedRearDistance(rearWall, 350, 400, 0.4)*0.3;
                x = hardware.localization.getCorrectionSpeedRightDistance(rightWall, 400, 120, 0.4)*0.4;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(x <= 0.1 && y <= 0.1 && rotationSpeed < 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("RR: ",hardware.localization.distanceRearRight.lastReadout);
                telemetry.update();

                Thread.yield();
            }
            hardware.mecanum.killSwitch();

            hardware.startLiftHoming();

            while(!isStopRequested() && !hardware.swBrick.isPressed()){

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = 0.3;
                x = 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(hardware.swBrick.isPressed()) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("SD4: ",hardware.localization.distanceRear);
                telemetry.update();

                Thread.yield();
            }
            hardware.mecanum.killSwitch();

            hardware.lowerLiftServo();
            hardware.sleep(700);
            hardware.startSuction();
            hardware.sleep(700);
            hardware.raiseLiftServo();
            hardware.sleep(700);

            while(!isStopRequested()){

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = - hardware.localization.getCorrectionSpeedRearDistance(700, 350, 400, 0.4)*0.3;
                x = 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(y <= 0.15) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("SD4: ",hardware.localization.distanceRear);
                telemetry.update();

                Thread.yield();
            }
            hardware.mecanum.killSwitch();

            targetAngle += 90;

            while(!isStopRequested()){

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = 0;
                x = 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(Math.abs(rotationSpeed) < 0.15 && hardware.gyro.getValue() != targetAngle) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("Corr: ",rotationSpeed);
                telemetry.update();

                Thread.yield();
            }
            hardware.mecanum.killSwitch();

            /*while(!isStopRequested()){

                hardware.localization.update();

                if(isStopRequested())throw new InterruptedException();

                y = 0;
                x = 0;
                rotationSpeed = hardware.gyro.getCorrection(targetAngle);
                if(Math.abs(rotationSpeed) < 0.3) break;

                hardware.mecanum.moveXYR(x, y, rotationSpeed);

                telemetry.addData("Corr: ",rotationSpeed);
                telemetry.update();

                Thread.yield();
            }
            hardware.mecanum.killSwitch();*/

            hardware.mecanum.moveXYR(0,0.7,0);
            sleep(2000);

            hardware.mecanum.killSwitch();
            hardware.startPumpHoming();
            hardware.sleep(700);

            hardware.mecanum.moveXYR(0,-0.5,0);
            sleep(700);
            hardware.mecanum.killSwitch();

            throw new InterruptedException();

        }catch(InterruptedException e){
            hardware.mecanum.killSwitch();
            hardware.startPumpHoming();
        }

    }
}
