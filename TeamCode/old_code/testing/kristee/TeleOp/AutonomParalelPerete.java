package testing.kristee.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.apis.testing.kristee.CrHardware;
@Disabled
@TeleOp(name = "autonom_perete")
public class AutonomParalelPerete extends OpMode {

    // Declare hardware
    CrHardware hardware = null;
    
    // Declare variables
    double FD,          // Distance from front sensor
           RD,          // Distance from rear sensor
           RFD,         // Distance from right front sensor
           RRD,         // Distance from right rear sensor
           LFD,         // Distance from left front sensor
           LRD,         // Distance from left rear sensor
           speed = 1,   // Rotation speed of motors
           strafe = 1;  // Translation direction

    @Override
    public void init(){
    
        // Initialize distance sensors and motors
        hardware = new CrHardware();
        hardware.init(hardwareMap);
    }

    @Override
    public void loop (){
    
        // Read the distances
        FD = hardware.distSensors.getFD();
        RD = hardware.distSensors.getRD();
        RFD = hardware.distSensors.getRFD();
        RRD = hardware.distSensors.getRRD();
        LFD = hardware.distSensors.getLFD();
        LRD = hardware.distSensors.getLRD();

        // Power up the motors with declared speed and direction correction
        hardware.go.move(speed,0,(RFD - RRD) / 20);
        
        // Modify movement direction in translation
        if (speed > 0 && FD <= 25){
            strafe = -strafe;
        }
        else if(speed < 0 && RD <= 25){
            strafe = -strafe;
        }

        // Modify movement direction and brake slowly when it's approaching the wall
        if (strafe > 0 && FD <= 100){
            speed = 1 * (FD - 25) / 20;
        }
        else if (strafe < 0 && RD <= 100){
            speed = -1 * (RD - 25) / 20;
        }
        else if (strafe > 0){
            speed = 1;
        }
        else if (strafe < 0){
            speed = -1;
        }
        
        // Separating by movement of translation against the wall
        if (RFD < 23 || RRD < 23) {
            hardware.go.move(speed,strafe,0);
        }
        else if (RFD > 26 || RRD < 26) {
            hardware.go.move(speed,-strafe,0);
        }
    }
}
