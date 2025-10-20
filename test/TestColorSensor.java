package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="TestColorSensor", group="Test")
public class TestColorSensor extends LinearOpMode {
    ColorSensor colorSensor;
    final int SCALE_FACTOR = 255;
    float[] hsvValues = {0F, 0F, 0F};


    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        waitForStart();

        while (opModeIsActive()) {
            Color.RGBToHSV(
                    (colorSensor.red() * SCALE_FACTOR),
                    (colorSensor.green() * SCALE_FACTOR),
                    (colorSensor.blue() * SCALE_FACTOR),
                    hsvValues
            );

            telemetry.addData("Hue Value", hsvValues[0]);
            telemetry.update();
        }
    }
}
