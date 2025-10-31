package org.firstinspires.ftc.teamcode.mechanisms.intake;

import org.firstinspires.ftc.teamcode.hardware.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.motor.MotorHandler;
import org.firstinspires.ftc.teamcode.hardware.motor.ZeroPowerBehavior;

public class Roller {
    private final MotorHandler roller;

    public Roller(SafeHardwareMap safeHardwareMap) {
        roller = safeHardwareMap.getMotor("Roller");
        roller.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
    }

    // Intake Functions
    public void forward() {
        roller.setPower(0.5);
    }

    public void reverse() {
        roller.setPower(-0.5);
    }

    public void stop() {
        roller.setPower(0);
    }
}
