package testing.kristee.Autonomus.AutonoameVechi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.apis.testing.kristee.CrHardware;
@Disabled
@Autonomous(name = "BlueTava/RedBrick1")

public class BlueTava_RedBrick1 extends LinearOpMode {

    CrHardware hardware = null;

    double speed = 0.7;
    double strafe = 1;

    private void initHardware(){

        hardware = new CrHardware();
        hardware.init(hardwareMap);

    }

    @Override
    public void runOpMode() throws InterruptedException{

        initHardware();

        waitForStart();

        try{

            if(isStopRequested()) throw new InterruptedException();

            hardware.go.move(speed, -strafe, 0);
            sleep(hardware.time - 100);
            hardware.go.move(-speed, 0, 0);
            sleep(hardware.time-hardware.diffTime);
            throw new InterruptedException();

        }
        catch(InterruptedException e){

            hardware.go.go.killSwitch();

        }
    }

}
