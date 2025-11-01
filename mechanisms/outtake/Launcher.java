package org.firstinspires.ftc.teamcode.mechanisms.outtake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ServoPos;
import org.firstinspires.ftc.teamcode.Timings;
import org.firstinspires.ftc.teamcode.hardware.FunctionThread;
import org.firstinspires.ftc.teamcode.hardware.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.motor.Direction;
import org.firstinspires.ftc.teamcode.hardware.motor.MotorHandler;
import org.firstinspires.ftc.teamcode.hardware.motor.ServoHandler;
import org.firstinspires.ftc.teamcode.hardware.motor.ZeroPowerBehavior;

public class Launcher {
    private final MotorHandler launcher;
    private final ServoHandler leftLift, rightLift;
    private final Telemetry telemetry;
    private FunctionThread launcherThread;

    public Launcher(SafeHardwareMap safeHardwareMap, Telemetry telemetry) {
        launcher = safeHardwareMap.getMotor("Launcher");
        launcher.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        leftLift = safeHardwareMap.getServo("LeftLift");
        rightLift = safeHardwareMap.getServo("RightLift");
        rightLift.setDirection(Direction.REVERSE);

        this.telemetry = telemetry;
    }

    public void start() {
        if (launcherThread == null || !launcherThread.isAlive()) {
            launcherThread = new FunctionThread(this::startLauncher, this::stopLauncher);
            launcherThread.start();
        }
        else {
            telemetry.addLine("Launcher Already Running");
            telemetry.update();
        }
    }

    public void reverse() {
        if (!isRunning()) {
            launcherThread = new FunctionThread(this::reverseLauncher, this::stopLauncher);
            launcherThread.start();
        }
        else {
            telemetry.addLine("Launcher Already Running");
            telemetry.update();
        }
    }

    public void stop() {
        if (launcherThread != null && launcherThread.isAlive()) {
            launcherThread.interrupt();
        }
    }

    public boolean isRunning() {
        return launcherThread != null && launcherThread.isAlive();
    }

    private void startLauncher() throws InterruptedException {
        launcher.setPower(1);
        Thread.sleep(1000);

        leftLift.setPosition(ServoPos.LIFT_OUT_POS.getPos());
        Thread.sleep(100);
        rightLift.setPosition(ServoPos.LIFT_OUT_POS.getPos());

        Thread.sleep((long) Timings.LAUNCHER_SHOOT_TIME.getMilliseconds());
    }

    private void reverseLauncher() throws InterruptedException {
        launcher.setPower(-1);
        Thread.sleep(1000);
    }

    private void stopLauncher() {
        leftLift.setPosition(ServoPos.LIFT_IN_POS.getPos());
        rightLift.setPosition(ServoPos.LIFT_IN_POS.getPos());
        launcher.setPower(0);
    }
}
