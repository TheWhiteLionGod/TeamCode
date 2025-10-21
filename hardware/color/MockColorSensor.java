package org.firstinspires.ftc.teamcode.hardware.color;

public class MockColorSensor implements ColorSensorHandler {
    @Override public int red() { return 0; }
    @Override public int green() { return 0; }
    @Override public int blue() { return 0; }
    @Override public float[] hsv() { return new float[3]; }
}
