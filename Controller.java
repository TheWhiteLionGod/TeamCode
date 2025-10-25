package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.FunctionThread;
import org.firstinspires.ftc.teamcode.hardware.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.hardware.motor.Direction;
import org.firstinspires.ftc.teamcode.hardware.motor.ZeroPowerBehavior;

@TeleOp(name="Controller", group="FTC2025")
public class Controller extends Robot {
    @Override
    public void run() {
        while (canRun()) {
            updateOdometry();

            // Switching Gears
            if (gamepad1.y) {
                changeGearMode(1);
            }
            else if (gamepad1.a) {
                changeGearMode(-1);
            }

            // Field Drive Movement
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                fieldDriveMove(gamepad1.left_stick_x, gamepad1.left_stick_y);
            }
            else {
                stopDrivetrain();
            }

            // Turning
            if (gamepad1.right_stick_x != 0) {
                turnDrivetrain(gamepad1.right_stick_x);
            }

            // Going to "Base"
            if (gamepad1.left_stick_button) {
                goToBlueBase();
            }
            else if (gamepad1.right_stick_button) {
                goToRedBase();
            }

            // Roller Controls
            if (gamepad1.left_trigger != 0) {
                forwardIntake();
            }
            else if (gamepad1.left_bumper) {
                backwardIntake();
            }
            else {
                stopIntake();
            }

            // Launcher Controls
            if ((runLauncherThread == null || !runLauncherThread.isAlive())
                    && (spinCarouselThread == null || !spinCarouselThread.isAlive())) {
                if (gamepad1.right_trigger != 0) {
                    runLauncherThread = new FunctionThread(this::startLauncher, this::stopLauncher);
                    runLauncherThread.start();
                }
                else if (gamepad1.right_bumper && runLauncherThread != null && runLauncherThread.isAlive()) {
                    runLauncherThread.interrupt();
                }
            }

            // Carousel Controls
            if ((spinCarouselThread == null || !spinCarouselThread.isAlive())
                    && (runLauncherThread == null || !runLauncherThread.isAlive())) {

                if (gamepad1.x) {
                    spinCarouselThread = new FunctionThread(this::findGreenBall, () -> {});
                }
                else if (gamepad1.b) {
                    spinCarouselThread = new FunctionThread(this::findPurpleBall, () -> {});
                }
                else if (gamepad1.dpad_down) {
                    spinCarouselThread = new FunctionThread(this::spinCarousel,
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }
                else if (gamepad1.dpad_left) {
                    spinCarouselThread = new FunctionThread(() -> spinCarousel(ServoPos.CAROUSEL_POS_1.getPos()),
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }
                else if (gamepad1.dpad_up) {
                    spinCarouselThread = new FunctionThread(() -> spinCarousel(ServoPos.CAROUSEL_POS_2.getPos()),
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }
                else if (gamepad1.dpad_right) {
                    spinCarouselThread = new FunctionThread(() -> spinCarousel(ServoPos.CAROUSEL_POS_3.getPos()),
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }

                if (spinCarouselThread != null)
                    try { spinCarouselThread.start(); }
                    catch (IllegalThreadStateException ignored) {}
            }

            sleep(50);
        }

        stopDrivetrain(); // Stopping Drivetrain
        stopIntake(); // Stopping Intake

        // Stopping Carousel
        if (spinCarouselThread != null && spinCarouselThread.isAlive()) {
            spinCarouselThread.interrupt(); // Stopping Carousel Spinning
        }

        // Stopping Launcher and Lift
        if (runLauncherThread != null && runLauncherThread.isAlive()) {
            runLauncherThread.interrupt(); // Stopping Shooting Mechanism
        }

        visionPortal.close();
    }
}
