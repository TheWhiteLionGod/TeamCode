package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.AprilTagId;
import org.firstinspires.ftc.teamcode.BallColor;
import org.firstinspires.ftc.teamcode.Timings;
import org.firstinspires.ftc.teamcode.hardware.FunctionThread;
import org.firstinspires.ftc.teamcode.Positions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
@Autonomous(name = "AutoShootMotif", group = "FTC2025")
public class AutoShootMotif extends Robot {
    @Override
    public void configure() {
        super.configure();
        if (alliance == null) { alliance = Alliance.BLUE; }

        if (getAprilTag() == null) {
            odometry.setPoseEstimate(alliance == Alliance.BLUE
                    ? Positions.BLUE_UP.getPose2D()
                    : Positions.RED_UP.getPose2D()
            );
        }
        else {
            odometry.setPoseEstimate(alliance == Alliance.BLUE
                    ? Positions.BLUE_DOWN.getPose2D()
                    : Positions.RED_DOWN.getPose2D()
            );
        }
    }

    @Override
    public void run() {
        // Going to  Obelisk
        odometry.moveRobot(Trajectories.READ_OBELISK.getTrajectory(odometry), this::canRun);
        AprilTagDetection detection = getAprilTag(); // Reading April Tag

        AprilTagId tagId = detection == null ? null : AprilTagId.getTagFromId(detection.id);
        if (tagId == null)
            tagId = AprilTagId.PGP_TAG;

        Pose2d motifBallPos;
        BallColor[] shootingOrder;

        switch (tagId) {
            case GPP_TAG:
                shootingOrder = new BallColor[]{BallColor.GREEN, BallColor.PURPLE, BallColor.PURPLE};
                motifBallPos = (alliance == Alliance.BLUE) ? Positions.BLUE_GPP.getPose2D() : Positions.RED_GPP.getPose2D();
                break;
            case PPG_TAG:
                shootingOrder = new BallColor[]{BallColor.PURPLE, BallColor.PURPLE, BallColor.GREEN};
                motifBallPos = (alliance == Alliance.BLUE) ? Positions.BLUE_PPG.getPose2D() : Positions.RED_PPG.getPose2D();
                break;
            default: // PGP or null detection
                shootingOrder = new BallColor[]{BallColor.PURPLE, BallColor.GREEN, BallColor.PURPLE};
                motifBallPos = (alliance == Alliance.BLUE) ? Positions.BLUE_PGP.getPose2D() : Positions.RED_PGP.getPose2D();
                break;
        }

        // Shooting Preloaded Balls
        odometry.followTrajectorySequenceAsync(
                alliance == Alliance.BLUE
                        ? Trajectories.SHOOT_BLUE.getTrajectory(odometry)
                        : Trajectories.SHOOT_RED.getTrajectory(odometry)
        );

        // Sorting Ball While Moving
        if (shootingOrder[0] == BallColor.GREEN) {
            carousel.findGreenBall();
        }
        else {
            carousel.findPurpleBall();
        }

        while (odometry.isBusy()) {
            odometry.update();
        }

        // Waiting For Carousel to Finish
        while (carousel.isRunning()) {
            sleep(50);
        }

        // Shooting First Ball
        launcher.start();
        while (launcher.isRunning()) {
            sleep(50);
        }

        // Shooting Second and Third Ball
        for (int i = 1; i < 3; i++) {
            // Getting Correct Ball
            if (shootingOrder[i] == BallColor.GREEN) {
                carousel.findGreenBall();
            }
            else {
                carousel.findPurpleBall();
            }


            // Waiting For Carousel to Finish Spinning
            while (carousel.isRunning()) {
                sleep(50);
            }

            // Shooting Ball
            launcher.start();
            while (launcher.isRunning()) {
                sleep(50);
            }
        }

        // Moving to Center
        odometry.moveRobot(Trajectories.trajectoryTo(Positions.START.getPose2D(), odometry), this::canRun);

        // Picking Up Motif Matching Ball
        roller.forward();
        odometry.followTrajectorySequenceAsync(Trajectories.trajectoryTo(motifBallPos, odometry));

        // Spinning Carousel while Driving
        while (odometry.isBusy()) {
            odometry.update();
            if (!carousel.isRunning()) {
                carousel.start();
            }
        }
        roller.stop(); // Stopping Intake as Drivetrain has Arrived

        // Sorting Carousel and Shooting At Same Time
        odometry.followTrajectorySequenceAsync(
                alliance == Alliance.BLUE
                ? Trajectories.SHOOT_BLUE.getTrajectory(odometry)
                : Trajectories.SHOOT_RED.getTrajectory(odometry));

        // Waiting For Carousel to Finish Spinning from Intake while Driving
        while (odometry.isBusy() && carousel.isRunning()) {
            odometry.update();
        }

        // Will Block Only If Carousel is Still Running and Robot No Longer Driving
        while (carousel.isRunning()) {
            sleep(50);
        }

        for (int i = 0; i < 3; i++) {
            // Getting Correct Ball
            if (shootingOrder[i] == BallColor.GREEN) {
                carousel.findGreenBall();
            }
            else {
                carousel.findPurpleBall();
            }

            // Waiting For Carousel to Finish Spinning
            while (odometry.isBusy() || carousel.isRunning()) {
                odometry.update();
            }

            // Shooting Ball
            launcher.start();
            while (launcher.isRunning()) {
                sleep(50);
            }
        }

        // Moving Robot 2 Feet Away From Obelisk to Not Block Allies
        odometry.moveRobot(
                Trajectories.trajectoryTo(
                        Positions.OBELISK.getPose2D().minus(
                                new Pose2d(2 * 12, 0, Math.toRadians(0))
                        ),
                        odometry
                ),
                this::canRun
        );
    }
}