package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.trajectorysequence.TrajectorySequence;

import javafx.geometry.Pos;

public enum Trajectories {
    SAMPLE_TRAJ,
    SHOOT_RED, SHOOT_BLUE,
    GO_RED_BASE, GO_BLUE_BASE,
    READ_OBELISK;

    public TrajectorySequence getTrajectory(SampleMecanumDrive drive) {
        switch (this) {
            case SAMPLE_TRAJ:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(new Pose2d(2*12, 0, Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -2*12, 0), 0)

                        .splineToLinearHeading(new Pose2d(2*12, 0, Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -2*12, 0), 0)

                        .splineToLinearHeading(new Pose2d(2*12, 0, Math.toRadians(90)), Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(0, -2*12, 0), 0)

                        .splineToSplineHeading(new Pose2d(4*12, 2*12, Math.toRadians(180)), Math.toRadians(180))

                        .build();
                
            case SHOOT_RED:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(Positions.START.getPose2D())
                        .lineToLinearHeading(Positions.SCORE_RED.getPose2D())
                        .build();
                
            case SHOOT_BLUE:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(Positions.START.getPose2D())
                        .lineToLinearHeading(Positions.SCORE_BLUE.getPose2D())
                        .build();

            case GO_RED_BASE:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(Positions.RED_BASE.getPose2D(), 0)
                        .build();

            case GO_BLUE_BASE:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .splineToLinearHeading(Positions.BLUE_BASE.getPose2D(), 0)
                        .build();

            case READ_OBELISK:
                return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(Positions.START.getPose2D())
                        .build();
                
            default:
                throw new IllegalStateException("Unknown trajectory type");
        }
    }

    public static TrajectorySequence trajectoryTo(Pose2d pose, SampleMecanumDrive drive) {
        return drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(pose)
                .build();
    }
}
