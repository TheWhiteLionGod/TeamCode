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
            drive.setPoseEstimate(alliance == Alliance.BLUE
                    ? Positions.BLUE_UP.getPose2D()
                    : Positions.RED_UP.getPose2D()
            );
        }
        else {
            drive.setPoseEstimate(alliance == Alliance.BLUE
                    ? Positions.BLUE_DOWN.getPose2D()
                    : Positions.RED_DOWN.getPose2D()
            );
        }
    }

    @Override
    public void run() {
        // Going to  Obelisk
        moveRobot(Trajectories.READ_OBELISK.getTrajectory(drive));
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
        drive.followTrajectorySequenceAsync(
                alliance == Alliance.BLUE
                        ? Trajectories.SHOOT_BLUE.getTrajectory(drive)
                        : Trajectories.SHOOT_RED.getTrajectory(drive)
        );

        // Sorting Ball While Moving
        spinCarouselThread = new FunctionThread(
                shootingOrder[0] == BallColor.GREEN
                        ? this::findGreenBall
                        : this::findPurpleBall,
                () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds())
        );

        while (drive.isBusy()) {
            updateOdometry();
        }

        // Waiting For Carousel to Finish
        try { spinCarouselThread.join(); }
        catch (InterruptedException ignored) {}

        // Shooting First Ball
        runLauncherThread = new FunctionThread(this::startLauncher, this::stopLauncher);
        runLauncherThread.start();

        try { runLauncherThread.join(); }
        catch (InterruptedException ignored) {}

        // Shooting Second and Third Ball
        for (int i = 1; i < 3; i++) {
            // Getting Correct Ball
            spinCarouselThread = (shootingOrder[i] == BallColor.GREEN)
                    ? new FunctionThread(this::findGreenBall, () -> {})
                    : new FunctionThread(this::findPurpleBall, () -> {});
            spinCarouselThread.start();

            // Waiting For Carousel to Finish Spinning
            try { spinCarouselThread.join(); }
            catch (InterruptedException ignored) {}

            // Shooting Ball
            runLauncherThread = new FunctionThread(this::startLauncher, this::stopLauncher);
            runLauncherThread.start();

            try { runLauncherThread.join(); }
            catch (InterruptedException ignored) {}
        }

        // Moving to Center
        drive.followTrajectorySequence(Trajectories.trajectoryTo(Positions.START.getPose2D(), drive));

        // Picking Up Motif Matching Ball
        forwardIntake();
        drive.followTrajectorySequenceAsync(Trajectories.trajectoryTo(motifBallPos, drive));

        // Spinning Carousel while Driving
        while (drive.isBusy()) {
            updateOdometry();
            if (spinCarouselThread == null || !spinCarouselThread.isAlive()) {
                // Spin Carousel then Wait 500 Milliseconds Before Doing it Again
                spinCarouselThread = new FunctionThread(
                        this::spinCarousel,
                        () -> Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds())
                );
                spinCarouselThread.start();
            }
        }
        stopIntake(); // Stopping Intake as Drivetrain has Arrived

        // Sorting Carousel and Shooting At Same Time
        drive.followTrajectorySequenceAsync(
                alliance == Alliance.BLUE
                ? Trajectories.SHOOT_BLUE.getTrajectory(drive)
                : Trajectories.SHOOT_RED.getTrajectory(drive));

        // Waiting For Carousel to Finish Spinning from Intake while Driving
        while (drive.isBusy() && spinCarouselThread.isAlive()) {
            updateOdometry();
        }

        // Will Block Only If Carousel is Still Running and Robot No Longer Driving
        try { spinCarouselThread.join(); }
        catch (InterruptedException ignored) {}

        for (int i = 0; i < 3; i++) {
            // Getting Correct Ball
            spinCarouselThread = (shootingOrder[i] == BallColor.GREEN)
                    ? new FunctionThread(this::findGreenBall, () -> {})
                    : new FunctionThread(this::findPurpleBall, () -> {});
            spinCarouselThread.start();

            // Waiting For Carousel to Finish Spinning
            while (drive.isBusy() || spinCarouselThread.isAlive()) {
                updateOdometry();
            }

            // Shooting Ball
            runLauncherThread = new FunctionThread(this::startLauncher, this::stopLauncher);
            runLauncherThread.start();

            try { runLauncherThread.join(); }
            catch (InterruptedException ignored) {}
        }

        // Moving Robot 2 Feet Away From Obelisk to Not Block Allies
        moveRobot(Trajectories.trajectoryTo(Positions.OBELISK.getPose2D().minus(new Pose2d(2 * 12, 0, Math.toRadians(0))), drive));
    }
}