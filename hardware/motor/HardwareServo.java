package org.firstinspires.ftc.teamcode.hardware.motor;

import com.qualcomm.robotcore.hardware.Servo;

public class HardwareServo implements ServoHandler {
    private final Servo servo;
    public HardwareServo(Servo servo) { this.servo = servo; }
    @Override public void setPosition(double position) { servo.setPosition(position); }
    @Override public double getPosition() { return servo.getPosition(); }

    @Override
    public void setDirection(Direction dir) {
        servo.setDirection(dir == Direction.FORWARD
                ? Servo.Direction.FORWARD
                : Servo.Direction.REVERSE
        );
    }

    @Override
    public Direction getDirection() {
        return servo.getDirection() == Servo.Direction.FORWARD
                ? Direction.FORWARD
                : Direction.REVERSE;
    }
}
