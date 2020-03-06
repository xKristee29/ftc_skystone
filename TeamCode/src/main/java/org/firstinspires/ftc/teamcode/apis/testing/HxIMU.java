package org.firstinspires.ftc.teamcode.apis.testing;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class HxIMU {
    BNO055IMU imu = null;
    public void init(BNO055IMU imu) {
        this.imu = imu;
    }

    boolean isStarted = false;
    double angle = 0;
    double prevAngle = 0;
    double offset = 0;

    public double getValue() {
        prevAngle = angle;
        angle = - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        if(angle < 0) angle += 90;
        if(!isStarted) {
            prevAngle = angle;
            isStarted = true;
        }
        if(prevAngle > 85 && angle < 5) {
            offset += 90;
        }
        if(prevAngle < 5 && angle > 85) {
            offset -= 90;
        }
        return angle + offset;
    }

    public double getCorrection(double target) {
        return (getValue() - target) * 1.3 / 180;
    }
}
