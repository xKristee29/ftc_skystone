package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.RobotParams;

import java.util.List;

@TeleOp(name = "TFODtest")
public class WebCamTest extends OpMode {

    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";

    private static final String VUFORIA_KEY = RobotParams.VUFORIA_LICENSE_KEY;

    private VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    private int skystonePos = 2;

    @Override
    public void init(){

        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        if (tfod != null) {
            tfod.activate();
        }

        while (skystonePos == 2) {
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
                                 skystonePos = -1;
                            else skystonePos =  0;
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
    public void loop(){

        switch(skystonePos){

            case -1:
                telemetry.addData("Pos: ", "Left");
                break;

            case 0:
                telemetry.addData("Pos: ", "Mid");
                break;

            case 1:
                telemetry.addData("Pos: ", "Right");
                break;

        }
        telemetry.update();

    }
}
