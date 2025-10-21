package org.firstinspires.ftc.teamcode.hardware.motor;

public interface MotorHandler {
    void setPower(double pwr);
    double getPower();
    void setVelocity(double ticks);
    double getVelocity();
    void setZeroPowerBehavior(ZeroPowerBehavior behavior);
    ZeroPowerBehavior getZeroPowerBehavior();
    void setDirection(Direction dir);
    Direction getDirection();
}
