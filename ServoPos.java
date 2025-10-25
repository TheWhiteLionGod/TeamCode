package org.firstinspires.ftc.teamcode;

public enum ServoPos {
    CAROUSEL_POS_1(0.0), CAROUSEL_POS_2(0.0), CAROUSEL_POS_3(0.0),
    LIFT_IN_POS(0.0), LIFT_OUT_POS(0.0);
    private final double pos;
    ServoPos(double pos) {
        this.pos = pos;
    }

    public double getPos() {
        return pos;
    }
}
