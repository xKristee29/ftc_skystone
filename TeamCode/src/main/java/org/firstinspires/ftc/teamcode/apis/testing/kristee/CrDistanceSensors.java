package org.firstinspires.ftc.teamcode.apis.testing.kristee;

import com.qualcomm.hardware.stmicroelectronics.VL53L0X;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class CrDistanceSensors {

    // Declare distance sensors
    DistanceSensor leftFrontDistance = null,
                   leftRearDistance = null,
                   rightFrontDistance = null,
                   rightRearDistance = null,
                   FrontDistance = null,
                   RearDistance = null;

    public void init(HardwareMap hardwareMap){

        // Initialize distance sensors
        FrontDistance = hardwareMap.get(VL53L0X.class, "sd1");
        rightFrontDistance = hardwareMap.get(VL53L0X.class, "sd2");
        rightRearDistance = hardwareMap.get(VL53L0X.class, "sd3");
        RearDistance = hardwareMap.get(VL53L0X.class, "sd4");
        leftRearDistance = hardwareMap.get(VL53L0X.class, "sd5");
        leftFrontDistance = hardwareMap.get(VL53L0X.class, "sd6");

    }

    // Send distance read by sd1
    public double getFD() {return FrontDistance.getDistance(DistanceUnit.MM);}

    // Send distance read by sd2
    public double getRFD() {return rightFrontDistance.getDistance(DistanceUnit.MM);}

    // Send distance read by sd3
    public double getRRD() {return rightRearDistance.getDistance(DistanceUnit.MM);}

    // Send distance read by sd4
    public double getRD() {return RearDistance.getDistance(DistanceUnit.MM);}

    // Send distance read by sd5
    public double getLRD() {return leftRearDistance.getDistance(DistanceUnit.MM);}

    // Send distance read by sd6
    public double getLFD() {return leftFrontDistance.getDistance(DistanceUnit.MM);}

}
