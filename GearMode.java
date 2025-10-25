package org.firstinspires.ftc.teamcode;

public enum GearMode {
    MIN_GEAR(1.0), MAX_GEAR(3.0);

    private final double gear;
    GearMode(double gear) {
        this.gear = gear;
    }

    public double getGear() {
        return gear;
    }
}
