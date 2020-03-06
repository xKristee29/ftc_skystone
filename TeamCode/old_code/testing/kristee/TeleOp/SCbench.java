package testing.kristee.TeleOp;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "SCbench", group = "bench")
public class SCbench extends LinearOpMode {

    private ColorSensor colorSensor = null;

    private int colorHSV;
    private float hue;
    private float sat;
    private float val;

    public void initHardware(){

        colorSensor = hardwareMap.get(ColorSensor.class, "sc2");

    }

    @Override
    public void runOpMode() {

        initHardware();

        while(isStarted());

        while(!isStopRequested()){

            colorHSV = Color.argb(colorSensor.alpha(), colorSensor.red(), colorSensor.green(), colorSensor.blue());

            hue = JavaUtil.colorToHue(colorHSV);
            telemetry.addData("Hue", hue);

            sat = JavaUtil.colorToSaturation(colorHSV);
            telemetry.addData("Saturation", sat);

            val = JavaUtil.colorToValue(colorHSV);
            telemetry.addData("Value", val);

            if (val <= 0.2 && sat <= 0.3) {
                telemetry.addData("Check Val", "Black");
            }
            else if (sat < 0.2) {
                telemetry.addData("Check Sat", "White");
            }
            telemetry.update();

        }
    }

}
