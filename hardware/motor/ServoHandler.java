package org.firstinspires.ftc.teamcode.hardware.motor;

public interface ServoHandler {
    void setPosition(double pos);
    double getPosition();
    void setDirection(Direction dir);
    Direction getDirection();
}
