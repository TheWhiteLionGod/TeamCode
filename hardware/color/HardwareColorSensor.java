package org.firstinspires.ftc.teamcode.hardware.color;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.ColorSensor;

public class HardwareColorSensor implements ColorSensorHandler {
    private final ColorSensor colorSensor;
    public HardwareColorSensor(ColorSensor colorSensor) { this.colorSensor = colorSensor; }
    @Override public int red() { return colorSensor.red(); }
    @Override public int green() { return colorSensor.green(); }
    @Override public int blue() { return colorSensor.blue(); }

    @Override public float[] hsv() {
        float[] hsv = new float[3];
        Color.RGBToHSV(red(), green(), blue(), hsv);
        return hsv;
    }
}
