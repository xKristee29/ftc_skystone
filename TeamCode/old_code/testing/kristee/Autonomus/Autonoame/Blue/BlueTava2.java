package testing.kristee.Autonomus.Autonoame.Blue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.apis.testing.kristee.CrHardware;

@Autonomous(name = "[S] BlueTava2", group = "Simplu")

public class BlueTava2 extends LinearOpMode {

    CrHardware hardware = null;

    double speed;
    double strafe = 1;

    private void initHardware(){

        hardware = new CrHardware();
        hardware.init(hardwareMap);
        speed = hardware.speed;

    }

    @Override
    public void runOpMode() throws InterruptedException{

        initHardware();

        waitForStart();

        try{

            if(isStopRequested()) throw new InterruptedException();

            hardware.go.move(-speed, 0, 0);
            sleep(hardware.time-hardware.diffTime);
            throw new InterruptedException();

        }
        catch(InterruptedException e){

            hardware.go.go.killSwitch();

        }
    }

}
