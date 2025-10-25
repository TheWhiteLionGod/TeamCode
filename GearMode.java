package org.firstinspires.ftc.teamcode;

public enum GearMode {
    FIRST_GEAR(1), SECOND_GEAR(2), THIRD_GEAR(3);
    private final double gear;
    GearMode(double gear) {
        this.gear = gear;
    }

    public double getGear() {
        return gear;
    }

    public GearMode gearUp() {
        return values()[
                this.ordinal() + 1 == values().length
                ? this.ordinal()
                : this.ordinal() + 1
            ];
    }

    public GearMode gearDown() {
        return values()[this.ordinal() == 0 ? 0 : this.ordinal() - 1];
    }

    public double getMultiplier() {
        return gear / values().length;
    }
}
