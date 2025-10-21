package org.firstinspires.ftc.teamcode.hardware.motor;

public class MockServo implements ServoHandler {
    @Override public void setPosition(double position) {}
    @Override public double getPosition() { return 0.0; }
    @Override public void setDirection(Direction dir) {}
    @Override public Direction getDirection() { return Direction.FORWARD; }
}
