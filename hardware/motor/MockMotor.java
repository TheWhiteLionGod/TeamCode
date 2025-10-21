package org.firstinspires.ftc.teamcode.hardware.motor;

public class MockMotor implements MotorHandler {
    public MockMotor() {}
    @Override public void setPower(double pwr) {}
    @Override public double getPower() { return 0.0; }
    @Override public void setVelocity(double ticks) {}
    @Override public double getVelocity() { return 0.0; }
    @Override public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {}
    @Override public ZeroPowerBehavior getZeroPowerBehavior() { return ZeroPowerBehavior.FLOAT; }
    @Override public void setDirection(Direction dir) {}
    @Override public Direction getDirection() { return Direction.FORWARD; }
}
