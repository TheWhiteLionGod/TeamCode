package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Alliance;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.FunctionThread;
import org.firstinspires.ftc.teamcode.Positions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.roadrunner.trajectorysequence.TrajectorySequence;
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

        int tagId = (detection != null) ? detection.id : Constants.PGP_TAG_ID;

        Pose2d motifBallPos;
        int[] shootingOrder;

        switch (tagId) {
            case Constants.GPP_TAG_ID:
                shootingOrder = new int[]{Constants.GREEN_BALL, Constants.PURPLE_BALL, Constants.PURPLE_BALL};
                motifBallPos = (alliance == Alliance.BLUE) ? Positions.BLUE_GPP.getPose2D() : Positions.RED_GPP.getPose2D();
                break;
            case Constants.PPG_TAG_ID:
                shootingOrder = new int[]{Constants.PURPLE_BALL, Constants.PURPLE_BALL, Constants.GREEN_BALL};
                motifBallPos = (alliance == Alliance.BLUE) ? Positions.BLUE_PPG.getPose2D() : Positions.RED_PPG.getPose2D();
                break;
            default: // PGP or null detection
                shootingOrder = new int[]{Constants.PURPLE_BALL, Constants.GREEN_BALL, Constants.PURPLE_BALL};
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
                shootingOrder[0] == Constants.GREEN_BALL
                        ? this::findGreenBall
                        : this::findPurpleBall,
                () -> Thread.sleep(Constants.CAROUSEL_SPIN_TIME)
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
            spinCarouselThread = (shootingOrder[i] == Constants.GREEN_BALL)
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
                        () -> Thread.sleep(Constants.CAROUSEL_SPIN_TIME)
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
            spinCarouselThread = (shootingOrder[i] == Constants.GREEN_BALL)
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