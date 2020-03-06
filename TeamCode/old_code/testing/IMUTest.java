package testing;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.apis.testing.HxIMU;

@TeleOp(name = "imutest")
public class IMUTest extends OpMode {
    BNO055IMU imu = null;
    HxIMU imu2 = null;
    @Override
    public void init() {
        imu2 = new HxIMU();
        BNO055IMU.Parameters imuParams;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imuParams = new BNO055IMU.Parameters();
        imuParams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuParams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParams.mode = BNO055IMU.SensorMode.IMU;
        imu.initialize(imuParams);
        while (!Thread.interrupted() && !imu.isGyroCalibrated()){
            Thread.yield();
        }
        imu2.init(imu);
        telemetry.addData("IMU", "Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("third angle", imu2.getValue());
    }
}
