package org.firstinspires.ftc.teamcode;

public enum Timings {
    GEAR_COOLDOWN(250),
    CAROUSEL_SPIN_TIME(250),
    LAUNCHER_SHOOT_TIME(250);

    private final double milliseconds;

    Timings(double milliseconds) {
        this.milliseconds = milliseconds;
    }

    public double getMilliseconds() {
        return milliseconds;
    }

    public double getSeconds() {
        return milliseconds * 1000;
    }
}
