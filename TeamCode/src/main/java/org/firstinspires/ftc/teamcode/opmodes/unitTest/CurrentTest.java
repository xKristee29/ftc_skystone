package org.firstinspires.ftc.teamcode.opmodes.unitTest;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;

public class CurrentTest extends OpMode {
    ExpansionHubMotor motorA;

    @Override
    public void init() {
        motorA = (ExpansionHubMotor) hardwareMap.dcMotor.get("leftFrontMotor");
    }

    @Override
    public void loop() {

    }
}
