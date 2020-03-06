package testing;

import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Test Senzori Distanta", group = "bench")
public class SensorBenchmark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DistanceSensor sd1 = hardwareMap.get(VL53L0X.class, "sd1");
        DistanceSensor sd2 = hardwareMap.get(VL53L0X.class, "sd2");
        DistanceSensor sd3 = hardwareMap.get(VL53L0X.class, "sd3");
        DistanceSensor sd4 = hardwareMap.get(VL53L0X.class, "sd4");
        DistanceSensor sd5 = hardwareMap.get(VL53L0X.class, "sd5");
        DistanceSensor sd6 = hardwareMap.get(VL53L0X.class, "sd6");
        long nanosecs;
        waitForStart();
        double ds1,ds2,ds3,ds4,ds5,ds6;
        while (!Thread.currentThread().isInterrupted()) {
            nanosecs = System.nanoTime();

            ds1 = sd1.getDistance(DistanceUnit.MM);
            ds2 = sd2.getDistance(DistanceUnit.MM);
            ds3 = sd3.getDistance(DistanceUnit.MM);
            ds4 = sd4.getDistance(DistanceUnit.MM);
            ds5 = sd5.getDistance(DistanceUnit.MM);
            ds6 = sd6.getDistance(DistanceUnit.MM);

            nanosecs = System.nanoTime() - nanosecs;
            telemetry.addData("delay ms", TimeUnit.MILLISECONDS.convert(nanosecs, TimeUnit.NANOSECONDS));
            telemetry.addData("SD1: ", ds1);
            telemetry.addData("SD2: ", ds2);
            telemetry.addData("SD3: ", ds3);
            telemetry.addData("SD4: ", ds4);
            telemetry.addData("SD5: ", ds5);
            telemetry.addData("SD6: ", ds6);
            telemetry.update();
        }
    }
}
