package org.firstinspires.ftc.teamcode.hardware.motor;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class HardwareMotor implements MotorHandler {
    private final DcMotorEx motor;
    public HardwareMotor(DcMotorEx motor) { this.motor = motor; }
    @Override public void setPower(double pwr) {motor.setPower(pwr); }
    @Override public double getPower() { return motor.getPower(); }
    @Override public void setVelocity(double ticks) { motor.setVelocity(ticks); }
    @Override public double getVelocity() { return motor.getVelocity(); }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior == ZeroPowerBehavior.BRAKE
                ? DcMotorEx.ZeroPowerBehavior.BRAKE
                : DcMotorEx.ZeroPowerBehavior.FLOAT
        );
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return motor.getZeroPowerBehavior() == DcMotorEx.ZeroPowerBehavior.BRAKE
                ? ZeroPowerBehavior.BRAKE
                : ZeroPowerBehavior.FLOAT;
    }

    @Override
    public void setDirection(Direction dir) {
        motor.setDirection(dir == Direction.FORWARD
                ? DcMotorEx.Direction.FORWARD
                : DcMotorEx.Direction.REVERSE
        );
    }

    @Override
    public Direction getDirection() {
        return motor.getDirection() == DcMotorEx.Direction.FORWARD
                ? Direction.FORWARD
                : Direction.REVERSE;
    }
}
