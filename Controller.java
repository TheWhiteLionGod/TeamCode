package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.hardware.FunctionThread;

@TeleOp(name="Controller", group="FTC2025")
public class Controller extends Robot {
    @Override
    public void run() {
        while (canRun()) {
            odometry.update();

            // Switching Gears
            if (gamepad1.y) {
                drivetrain.gearUp();
            }
            else if (gamepad1.a) {
                drivetrain.gearDown();
            }

            // Field Drive Movement
            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                drivetrain.fieldDrive(
                        gamepad1.left_stick_x,
                        gamepad1.left_stick_y,
                        odometry.getPoseEstimate().getHeading()
                );
            }
            // Turning
            else if (gamepad1.right_stick_x != 0) {
                drivetrain.turn(gamepad1.right_stick_x);
            }
            else {
                drivetrain.stop();
            }

            // Going to "Base"
            if (gamepad1.left_stick_button) {
                odometry.goToBlueBase();
            }
            else if (gamepad1.right_stick_button) {
                odometry.goToRedBase();
            }

            // Roller Controls
            if (gamepad1.left_trigger != 0) {
                roller.forward();
            }
            else if (gamepad1.left_bumper) {
                roller.reverse();
            }
            else {
                roller.stop();
            }

            // Launcher Controls
            if ((runLauncherThread == null || !runLauncherThread.isAlive())
                    && (spinCarouselThread == null || !spinCarouselThread.isAlive())) {
                if (gamepad1.right_trigger != 0) {
                    runLauncherThread = new FunctionThread(this::startLauncher, this::stopLauncher);
                    runLauncherThread.start();
                }
            }
            else if (gamepad1.right_bumper && runLauncherThread != null && runLauncherThread.isAlive()) {
                runLauncherThread.interrupt();
            }

            // Carousel Controls
            if ((spinCarouselThread == null || !spinCarouselThread.isAlive())
                    && (runLauncherThread == null || !runLauncherThread.isAlive())) {

                if (gamepad1.x) {
                    spinCarouselThread = new FunctionThread(() -> carousel.findGreenBall(colorSensor), () -> {});
                }
                else if (gamepad1.b) {
                    spinCarouselThread = new FunctionThread(() -> carousel.findPurpleBall(colorSensor), () -> {});
                }
                else if (gamepad1.dpad_down) {
                    spinCarouselThread = new FunctionThread(carousel::spin,
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }
                else if (gamepad1.dpad_left) {
                    spinCarouselThread = new FunctionThread(() -> carousel.spin(ServoPos.CAROUSEL_POS_1.getPos()),
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }
                else if (gamepad1.dpad_up) {
                    spinCarouselThread = new FunctionThread(() -> carousel.spin(ServoPos.CAROUSEL_POS_2.getPos()),
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }
                else if (gamepad1.dpad_right) {
                    spinCarouselThread = new FunctionThread(() -> carousel.spin(ServoPos.CAROUSEL_POS_3.getPos()),
                            () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds()));
                }

                if (spinCarouselThread != null)
                    try { spinCarouselThread.start(); }
                    catch (IllegalThreadStateException ignored) {}
            }

            sleep(50);
        }

        drivetrain.stop(); // Stopping Drivetrain
        roller.stop(); // Stopping Intake

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
