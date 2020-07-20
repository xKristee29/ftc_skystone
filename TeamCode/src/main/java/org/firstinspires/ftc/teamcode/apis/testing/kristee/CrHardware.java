package org.firstinspires.ftc.teamcode.apis.testing.kristee;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class CrHardware {

    public CrMotors go = null;
    public CrDistanceSensors distSensors = null;
    public double speed = 0.7;
    public int timeX = 1700;
    public int timeY = 1100;

    public void init(HardwareMap hardwareMap){

        distSensors = new CrDistanceSensors();
        go = new CrMotors();

        distSensors.init(hardwareMap);
        go.init(hardwareMap);

    }

}
