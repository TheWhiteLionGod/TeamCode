package org.firstinspires.ftc.teamcode.mechanisms.drivetrain;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GearMode;
import org.firstinspires.ftc.teamcode.Timings;
import org.firstinspires.ftc.teamcode.hardware.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.motor.Direction;
import org.firstinspires.ftc.teamcode.hardware.motor.MotorHandler;
import org.firstinspires.ftc.teamcode.hardware.motor.ZeroPowerBehavior;

import java.time.Duration;
import java.time.Instant;

public class FieldDrive {
    private final MotorHandler BL, FL, FR, BR; // Wheel Motors
    private final Telemetry telemetry; // Telemetry
    private Instant gearSwitchTime = Instant.now(); // Cooldown
    private GearMode gearMode = GearMode.THIRD_GEAR; // Current Gear

    public FieldDrive(SafeHardwareMap safeHardwareMap, Telemetry telemetry) {
        BL = safeHardwareMap.getMotor("BL");
        FL = safeHardwareMap.getMotor("FL");
        FR = safeHardwareMap.getMotor("FR");
        BR = safeHardwareMap.getMotor("BR");

        BL.setDirection(Direction.REVERSE);
        FL.setDirection(Direction.REVERSE);

        BL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        this.telemetry = telemetry;
    }

    // Change Gear Mode of Robot
    public void gearUp() { changeGearMode(gearMode.gearUp()); }

    public void gearDown() { changeGearMode(gearMode.gearDown()); }

    private void changeGearMode(GearMode nextGear) {
        if (Duration.between(gearSwitchTime, Instant.now()).toMillis() >= Timings.GEAR_COOLDOWN.getMilliseconds()) {
            gearMode = nextGear;
            gearSwitchTime = Instant.now();

            telemetry.addData("Current Gear Mode", gearMode.getGear());
            telemetry.update();
        }
    }


    // Field Drive Movement
    public void fieldDrive(double pwr_x, double pwr_y, double yawRadians) {
        /* Adjust Joystick X/Y inputs by navX MXP yaw angle */
        double temp = pwr_y * Math.cos(yawRadians) + pwr_x * Math.sin(yawRadians);
        pwr_x = -pwr_y * Math.sin(yawRadians) + pwr_x * Math.cos(yawRadians);
        pwr_y = temp;

        /* At this point, Joystick X/Y (strafe/forward) vectors have been */
        /* rotated by the gyro angle, and can be sent to drive system */
        robotDrive(pwr_x, pwr_y);
    }

    // Regular Movement
    public void robotDrive(double pwrx, double pwry) {
        double gear_pwr = gearMode.getMultiplier();
        BL.setPower(gear_pwr*(-pwrx-pwry));
        FR.setPower(gear_pwr*(-pwrx-pwry));

        FL.setPower(gear_pwr*(pwrx-pwry));
        BR.setPower(gear_pwr*(pwrx-pwry));
    }

    // Turning
    public void turn(double pwr) {
        double gear_pwr = gearMode.getMultiplier();
        BL.setPower(gear_pwr*pwr);
        FR.setPower(gear_pwr*-pwr);

        FL.setPower(0);
        BR.setPower(0);
    }

    // Stopping Drivetrain
    public void stop() {
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }
}
