package testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.apis.testing.Localization;
import org.firstinspires.ftc.teamcode.apis.testing.Mecanum;
import org.firstinspires.ftc.teamcode.apis.testing.Utils;

import static org.firstinspires.ftc.teamcode.apis.testing.Utils.sleep;
@Disabled
@TeleOp (name = "Control Teleghidat Gyro")
public class CT2 extends OpMode {
    /*Hackerii nord-coreeni au dat gres
    * Will-Jung-Un fiind nevoit sa rescrie codul de unul singur.
    */
    BNO055IMU imu = null;

    double targetAngle = 0;
    double getAngularCorrection(double angle) {
        return 0;
    }
    Mecanum go = null;
    DcMotor leftFrontMotor = null;
    DcMotor rightFrontMotor = null;
    DcMotor leftRearMotor = null;
    DcMotor rightRearMotor = null;
    @Override
    public void init() {
        BNO055IMU.Parameters imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        leftFrontMotor = hardwareMap.get(DcMotor.class,"leftFrontMotor");
        rightFrontMotor = hardwareMap.get(DcMotor.class,"rightFrontMotor");
        leftRearMotor = hardwareMap.get(DcMotor.class,"leftRearMotor");
        rightRearMotor = hardwareMap.get(DcMotor.class,"rightRearMotor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(imuParams);
        while (!Thread.interrupted() && !imu.isGyroCalibrated()){
            Thread.yield();
        }
        telemetry.addData("Gyro is calibrated","yay!");
        go = new Mecanum();

        go.init(leftFrontMotor,
                rightFrontMotor,
                leftRearMotor,
                rightRearMotor);

        targetAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }

    double formatAngle(double angle){
        //return Math.atan2(Math.sin(angle), Math.cos(angle));
        double result = angle;
        while(result <= -Math.PI) result += 2 * Math.PI;
        while(result > Math.PI) result -= 2 * Math.PI;
        return result;
    }
    private boolean flag = true;
    @Override
    public void loop() {
        double joyangle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x);
        joyangle -= Math.PI/2;
        double speed = Math.sqrt(Math.pow(-gamepad1.left_stick_y,2)+Math.pow(gamepad1.left_stick_x,2));
        double rspeed = gamepad1.right_stick_x;

        Orientation r_mat = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double imuAngle = -r_mat.thirdAngle;

        if (Math.abs(rspeed) == 0) {
            //if(flag) {targetAngle = -imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS).thirdAngle;flag = false;}
            double correction = Range.clip((imuAngle- targetAngle)*1.3/180, -1, 1);
            if(Math.abs(correction) > 0.15)go.move(speed, joyangle, correction);
            else go.move(speed, joyangle, 0);
        }

        else {
            go.move(speed, joyangle, rspeed);
            targetAngle = -r_mat.thirdAngle;
            flag = true;
        }
        telemetry.addData("imu_rx", r_mat.firstAngle);
        telemetry.addData("imu_ry", r_mat.secondAngle);
        telemetry.addData("imu_rz", r_mat.thirdAngle);
        telemetry.addData("correction", (imuAngle - targetAngle)*1.5);
        telemetry.addData("target", targetAngle/10);
        telemetry.update();

        Thread.yield();
    }
    //Draga william aici e o bucata minunata de cod
    //Cum un intelept spunea "Mai date-n ... mea". Cheloo 2000


}
