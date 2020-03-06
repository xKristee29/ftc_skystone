package testing.fat_guy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.apis.testing.HardwareRepresentation.MotionState;
import org.firstinspires.ftc.teamcode.apis.testing.Localization;
import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;
import org.firstinspires.ftc.teamcode.apis.testing.SkystoneMotionPlayer;
@Disabled
@Autonomous(name = "Demo Buc Auto Fatza Delay")
public class AutonomousFront2Delay extends LinearOpMode {

    HardwareConfig hardwareConfig = null;
    Mecanum go = null;
    Localization localization = null;
    SkystoneMotionPlayer motionPlayer = null;

    private void initHardware() {
        hardwareConfig = new HardwareConfig();
        hardwareConfig.initHardware(hardwareMap);
        go = hardwareConfig.mecanum;
        localization = hardwareConfig.localization;
        motionPlayer = hardwareConfig.motionPlayer;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initHardware();


        waitForStart();

        try {
                if(isStopRequested()) throw new InterruptedException();
                sleep(25000);
                //go.move(1, -Math.PI/2, 0);
                //sleep(50);
                go.move(1, 0, 0);
                sleep(1000);
                go.killSwitch();
        }
        catch (InterruptedException e) {
            go.killSwitch();
        }
    }

    private void showDebugTelemetry(MotionState motionState) {
        if(motionPlayer.getPositionTarget().isTargetFrontSet) telemetry.addData("corr_front", localization.getCorrectionSpeedFrontDistance(motionPlayer.getPositionTarget().targetFront));
        if(motionPlayer.getPositionTarget().isTargetRearSet) telemetry.addData("corr_rear", localization.getCorrectionSpeedRearDistance(motionPlayer.getPositionTarget().targetRear));
        if(motionPlayer.getPositionTarget().isTargetLeftSet) telemetry.addData("corr_left", localization.getCorrectionSpeedLeftDistance(motionPlayer.getPositionTarget().targetLeft));
        if(motionPlayer.getPositionTarget().isTargetRightSet) telemetry.addData("corr_right", localization.getCorrectionSpeedRightDistance(motionPlayer.getPositionTarget().targetRight));
        if(motionPlayer.getPositionTarget().isTargetAngleLeftSet) telemetry.addData("corr_ag_left", localization.getAngleCorrectionLeft());
        if(motionPlayer.getPositionTarget().isTargetAngleRightSet) telemetry.addData("corr_ag_right", localization.getAngleCorrectionRight());

        telemetry.addData("rotate_speed", motionState.rotateSpeed);
        telemetry.addData("strafe_angle", motionState.strafeDirection);
        telemetry.addData("strafe_speed", motionState.strafeSpeed);
        telemetry.addData("isDone", motionState.isDone);
        telemetry.addData("index", motionPlayer.getIndex());


        telemetry.addData("sensorFrontLeft", localization.distanceFrontLeft.lastReadout);
        telemetry.addData("sensorFrontRight", localization.distanceFrontRight.lastReadout);
        telemetry.addData("sensorRearLeft", localization.distanceRearLeft.lastReadout);
        telemetry.addData("sensorRearRight", localization.distanceRearRight.lastReadout);
        telemetry.addData("sensorFront", localization.distanceFront.lastReadout);
        telemetry.addData("sensorRear", localization.distanceRear.lastReadout);
        telemetry.update();
    }
}
