package testing.fat_guy;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.apis.testing.Localization;
import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;
import org.firstinspires.ftc.teamcode.apis.testing.SkystoneMotionPlayer;
import org.firstinspires.ftc.teamcode.apis.testing.HardwareRepresentation.*;

@Disabled
@Autonomous(name = "AutonomousBlueTrayWall", group = "skystone")
public class AutonomousBlueTrayWall extends LinearOpMode {

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

        //Add corrections
        PositionTarget positionTarget = new PositionTarget();
        positionTarget.setCorrectionTargets(
                new CorrectionTarget(Correction.DIST_DIFF_LEFT, 0),
                new CorrectionTarget(Correction.DIST_FRONT, 500),
                new CorrectionTarget(Correction.DIST_LEFT, 700)
        );
        motionPlayer.addPositionTarget(positionTarget);

        positionTarget = new PositionTarget();

        positionTarget.setCorrectionTargets(
                new CorrectionTarget(Correction.DIST_DIFF_LEFT, 0),
                new CorrectionTarget(Correction.DIST_FRONT, 500),
                new CorrectionTarget(Correction.DIST_LEFT, 100)
        );
        motionPlayer.addPositionTarget(positionTarget);

        waitForStart();

        try {
            while (opModeIsActive()) {
                if(isStopRequested()) throw new InterruptedException();
                localization.update();
                MotionState motionState = motionPlayer.getCompoundCorrection();

                boolean contactSwitchA = false, contactSwitchB = false;

                if(!motionState.isDone)go.move(motionState);
                else switch (motionPlayer.getIndex()){
                    case 0:
                        //cazul 0 finalizat
                        while ((!contactSwitchA) && (!contactSwitchB)) {
                            //acualizam contactSwitchA si contactSwitchB
                            go.move(0.25, 0, 0);
                            if(isStopRequested()) throw new InterruptedException();
                        }
                        go.move(0, 0,0);
                        //lasa gheara
                        sleep(1000);

                        motionPlayer.next();
                        break;
                    case 1:
                        //ridica gheara
                        sleep(1000);
                        throw new InterruptedException();
                }
                showDebugTelemetry(motionState);
            }
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
        telemetry.update();
    }
}

