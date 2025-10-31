package org.firstinspires.ftc.teamcode.mechanisms.odometry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Trajectories;

public class Odometry {
    private final SampleMecanumDrive drive; // Roadrunner Driver
    private final Telemetry telemetry;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
        drive = new SampleMecanumDrive(hardwareMap);
        this.telemetry = telemetry;
    }

    public void updateOdometry() {
        drive.update(); // Update Roadrunner
//        updatePoseFromAprilTags(); // Update April Tag
    }

    public Pose2d getPoseEstimate() {
        return drive.getPoseEstimate();
    }

    public void setPoseEstimate(Pose2d pos) {
        drive.setPoseEstimate(pos);
    }

    public TrajectorySequenceBuilder trajectorySequenceBuilder(Pose2d start) {
        return drive.trajectorySequenceBuilder(start);
    }

    public void followTrajectorySequence(TrajectorySequence traj) {
        drive.followTrajectorySequence(traj);
    }

    public void followTrajectorySequenceAsync(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);
    }

    public boolean isBusy() {
        return drive.isBusy();
    }

    // Making Robot Follow Trajectory
    public void moveRobot(TrajectorySequence traj, BoolInterface canRun) {
        drive.followTrajectorySequenceAsync(traj);

        // Updating Robot Position
        while (canRun.run() && drive.isBusy()) {
            drive.update();
            telemetry.addData("Robot Position", drive.getPoseEstimate());
            telemetry.update();
        }

        // Printing Deviation in Position Once Completed
        telemetry.addData("Expected End Position", traj.end());
        telemetry.addData("Current Position", drive.getPoseEstimate());
        telemetry.update();
    }

    // These Functions Make Robot Go To Base
    public void goToRedBase() {
        try { moveRobot(Trajectories.GO_RED_BASE.getTrajectory(this), () -> true); }
        catch (EmptyPathSegmentException e) {
            telemetry.addLine("Robot Already At Position");
            telemetry.update();
        }
    }

    public void goToBlueBase() {
        try { moveRobot(Trajectories.GO_BLUE_BASE.getTrajectory(this), () -> true); }
        catch (EmptyPathSegmentException e) {
            telemetry.addLine("Robot Already At Position");
            telemetry.update();
        }
    }
}
