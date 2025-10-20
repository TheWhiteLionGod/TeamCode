package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.hardware.FunctionThread;
import org.firstinspires.ftc.teamcode.Positions;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.SafeHardwareMap;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "ShootMotifBlue", group = "FTC2025")
public class AutoShootMotif extends Robot {
    @Override
    public void configure() {
        SafeHardwareMap safeHardwareMap = new SafeHardwareMap(hardwareMap, telemetry);

        launcher = safeHardwareMap.getMotor("Launcher");
        roller = safeHardwareMap.getMotor("Roller");

        carousel = safeHardwareMap.getServo("Carousel");
        lift = safeHardwareMap.getServo("Lift");
        colorSensor = safeHardwareMap.getColorSensor("ColorSensor");

        aprilTag = safeHardwareMap.getAprilTagProcessor();
        visionPortal = safeHardwareMap.getVisionPortal(aprilTag, "Camera");

        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(Positions.BLUE_DOWN.getPose2D());
        telemetry.update();
    }

    @Override
    public void run() {
        // Going to  Obelisk
        moveRobot(Trajectories.READ_OBELISK.getTrajectory(drive));

        // Deciding Motif Pattern
        AprilTagDetection detection = getAprilTag();
        TrajectorySequence motifBallTraj = Trajectories.trajectoryTo(Positions.BLUE_PGP.getPose2D(), drive);
        int[] shootingOrder = new int[]{Constants.PURPLE_BALL, Constants.GREEN_BALL, Constants.PURPLE_BALL};

        if (detection != null && detection.id == Constants.GPP_TAG_ID) {
            motifBallTraj = Trajectories.trajectoryTo(Positions.BLUE_GPP.getPose2D(), drive);
            shootingOrder = new int[]{Constants.GREEN_BALL, Constants.PURPLE_BALL, Constants.PURPLE_BALL};
        }
        else if (detection != null && detection.id == Constants.PPG_TAG_ID) {
            motifBallTraj = Trajectories.trajectoryTo(Positions.BLUE_PPG.getPose2D(), drive);
            shootingOrder = new int[]{Constants.PURPLE_BALL, Constants.PURPLE_BALL, Constants.GREEN_BALL};
        }
        visionPortal.close();

        // Picking Up Motif Matching Ball
        forwardIntake();
        drive.followTrajectorySequenceAsync(motifBallTraj);

        while (drive.isBusy()) {
            updateOdometry();
            if (spinCarouselThread == null || !spinCarouselThread.isAlive()) {
                // Spin Carousel then Wait 500 Milliseconds Before Doing it Again
                spinCarouselThread = new FunctionThread(this::spinCarousel, () -> Thread.sleep(500));
                spinCarouselThread.start();
            }
        }
        stopIntake();

        // Sorting Carousel and Shooting At Same Time
        drive.followTrajectorySequenceAsync(Trajectories.SHOOT_BLUE.getTrajectory(drive));

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