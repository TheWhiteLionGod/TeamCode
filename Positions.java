package org.firstinspires.ftc.dynabytes;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public enum Positions {
    START(0, 0, 0),
    RED_DOWN(-5.25 * 12, -2.75 * 12, Math.toRadians(0)),
    BLUE_DOWN(-5.25 * 12, 2.75 * 12, Math.toRadians(0)),
    RED_UP(4*12, -4*12, Math.toRadians(135)),
    BLUE_UP(4*12, 4*12, Math.toRadians(225)),
    RED_BASE(-3.25*12, 2.75*12, Math.toRadians(0)),
    BLUE_BASE(-3.25*12, -2.75*12, Math.toRadians(0)),
    SCORE_RED(4*12, -4*12, Math.toRadians(315)),
    SCORE_BLUE(4 * 12, 4 * 12, Math.toRadians(45)),
    BLUE_TAG(4.86 * 12, 4.64 * 12, Math.toRadians(45)),
    RED_TAG(4.86 * 12, -4.64 * 12, Math.toRadians(315)),
    OBELISK(6 * 12, 0, Math.toRadians(0)),
    RED_GPP(-3 * 12, -5 * 12, Math.toRadians(270)),
    RED_PGP(-1 * 12, -5 * 12, Math.toRadians(270)),
    RED_PPG(12, -5 * 12, Math.toRadians(270)),
    BLUE_PPG(-3 * 12, 5 * 12, Math.toRadians(90)),
    BLUE_PGP(-1 * 12, 5 * 12, Math.toRadians(90)),
    BLUE_GPP(12, 5 * 12, Math.toRadians(90)),
    CAMERA(0, 0, Math.toRadians(0));


    public final double x, y, heading;

    Positions(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d getPose2D() {
        return new Pose2d(x, y, heading);
    }

    public static Pose2d getInverse(Pose2d pose) {
        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        // Standard 2D pose inversion formula
        return new Pose2d(
                -x * Math.cos(heading) - y * Math.sin(heading),
                x * Math.sin(heading) - y * Math.cos(heading),
                -heading
        );
    }

}
