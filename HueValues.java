package org.firstinspires.ftc.teamcode;

public enum HueValues {
    GREEN_MIN(150), GREEN_MAX(180),
    PURPLE_MIN(200), PURPLE_MAX(240);
    private final int hue;
    HueValues(int hue) {
        this.hue = hue;
    }

    public int getHue() {
        return hue;
    }
}
