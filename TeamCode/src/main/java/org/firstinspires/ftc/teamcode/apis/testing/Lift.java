package org.firstinspires.ftc.teamcode.apis.testing;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    DcMotor motorLift = null;
    DcMotor motorGlisiera = null;

    final int LIMIT_MIN_GLISIERA = -60;
    final int LIMIT_MAX_GLISIERA = -800;

    final int LIMIT_MIN_LIFT = -10;
    final int LIMIT_MAX_LIFT = -3700;

    final int STEPS_PER_BRICK = -200;
    final int STEPS_HEIGHT_BASE = -50;

    long startTime = 0;

    public void setTargetPositionGlisiera(int targetPosition) {
        if(LIMIT_MIN_GLISIERA > targetPosition && targetPosition > LIMIT_MAX_GLISIERA) {
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorGlisiera.setTargetPosition(targetPosition);
        }
    }

    public void setTargetPositionLift(int targetPosition) {
        if(LIMIT_MIN_LIFT > targetPosition && targetPosition > LIMIT_MAX_LIFT) {
            motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLift.setPower(0.5);
            motorLift.setTargetPosition(targetPosition);
        }
    }

    void setLiftPower(double joystickPosition) {
        //joystickPosition = Math.pow(joystickPosition,3);
        if(joystickPosition == 0)  {
            //go into set position mode

            setTargetPositionLift(
                    (int)Math.ceil(
                            (
                                (double)motorLift.getCurrentPosition()- STEPS_HEIGHT_BASE)
                                / (double)STEPS_PER_BRICK
                            ) * STEPS_PER_BRICK
                            + STEPS_HEIGHT_BASE
            );
        }
        else {
            motorLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            if(
                    joystickPosition > 0 &&
                    motorLift.getCurrentPosition() > LIMIT_MAX_LIFT
            ) {
                motorLift.setPower(-joystickPosition);
            }
            if(
                    joystickPosition < 0 &&
                    LIMIT_MIN_LIFT > motorLift.getCurrentPosition()
            ) {
                motorLift.setPower(-joystickPosition);
            }
        }
    }



    public void init(HardwareMap hardwareMap) {
        motorLift = hardwareMap.get(DcMotor.class, "motorLift");
        motorLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorGlisiera = hardwareMap.get(DcMotor.class, "motorGlisiera");
        motorGlisiera.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorGlisiera.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorGlisiera.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorGlisiera.setPower(0.5);
        motorGlisiera.setTargetPosition(-60);
    }


}
