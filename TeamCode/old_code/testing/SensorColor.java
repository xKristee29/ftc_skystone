
package testing;


import android.graphics.Color;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Disabled
@TeleOp(name = "Sensor: Color", group = "bench")
public class SensorColor extends LinearOpMode {

    private ColorSensor colorSensor = hardwareMap.get(ColorSensor.class, "senzor");
    private DistanceSensor distanceSensor = hardwareMap.get(DistanceSensor.class, "senzor");
    private NormalizedColorSensor colorSensorNormalized = (NormalizedColorSensor) colorSensor;

    @Override public void runOpMode() throws InterruptedException {
        float[] hsvValues = new float[3];
        boolean seeingBlue;
        NormalizedRGBA colors = colorSensorNormalized.getNormalizedColors();
        int color = colors.toColor();
        Color.RGBToHSV(Color.red(color), Color.green(color), Color.blue(color), hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        if( hue > 120 && hue < 260 && saturation >= 0.6)
            seeingBlue = true;
        double distanceMm = distanceSensor.getDistance(DistanceUnit.MM);
    }

}
