package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Positions;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.odometry.Odometry;

@Disabled
@TeleOp(name = "SampleAuto", group = "FTC2025")
public class SampleAuto extends Robot {
    @Override
    public void configure() {
        super.configure();
        odometry = new Odometry(hardwareMap, telemetry);
        odometry.setPoseEstimate(Positions.START.getPose2D());
    }

    public void run() {
        // Running Trajectory
        odometry.moveRobot(Trajectories.SAMPLE_TRAJ.getTrajectory(odometry), this::canRun);
    }
}