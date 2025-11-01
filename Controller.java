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
            if (gamepad1.y) { drivetrain.gearUp(); }
            else if (gamepad1.a) { drivetrain.gearDown(); }

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
            else { drivetrain.stop(); }

            // Going to "Base"
            if (gamepad1.left_stick_button) {
                odometry.goToBlueBase();
            }
            else if (gamepad1.right_stick_button) {
                odometry.goToRedBase();
            }

            // Roller Controls
            if (gamepad1.left_trigger != 0) { roller.forward(); }
            else if (gamepad1.left_bumper) { roller.reverse(); }
            else { roller.stop(); }

            // Launcher Controls
            if (gamepad1.right_trigger != 0) {
                // Making Sure Carousel isn't Running
                if (!carousel.isRunning() && !launcher.isRunning()) {
                    launcher.run();
                }
            }
            else if (gamepad1.right_bumper && launcher.isRunning()) {
                launcher.stop();
            }

            // Carousel Controls
            if (!carousel.isRunning() && !launcher.isRunning()) {
                if (gamepad1.x) {
                    carousel.findGreenBall(colorSensor);
                }
                else if (gamepad1.b) {
                    carousel.findPurpleBall(colorSensor);
                }
                else if (gamepad1.dpad_down) {
                    carousel.startCarousel();
                }
                else if (gamepad1.dpad_left) {
                    carousel.startCarousel(ServoPos.CAROUSEL_POS_1.getPos());
                }
                else if (gamepad1.dpad_up) {
                    carousel.startCarousel(ServoPos.CAROUSEL_POS_2.getPos());
                }
                else if (gamepad1.dpad_right) {
                    carousel.startCarousel(ServoPos.CAROUSEL_POS_3.getPos());
                }
            }

            sleep(50);
        }

        drivetrain.stop(); // Stopping Drivetrain
        roller.stop(); // Stopping Intake

        // Stopping Carousel
        carousel.stopCarousel();

        // Stopping Launcher and Lift
        launcher.stop();

        visionPortal.close();
    }
}
