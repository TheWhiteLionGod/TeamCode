package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Positions;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.mechanisms.odometry.Odometry;

@Disabled
@Autonomous(name = "AutoPickAndShoot", group = "FTC2025")
public class AutoPickAndShoot extends Robot {
    @Override
    public void configure() {
        super.configure();
        odometry = new Odometry(hardwareMap, telemetry);
        odometry.setPoseEstimate(Positions.BLUE_DOWN.getPose2D());
    }

    @Override
    public void run() {
        // Moving Robot
        odometry.moveRobot(Trajectories.trajectoryTo(Positions.BLUE_PPG.getPose2D(), odometry), this::canRun);
        odometry.moveRobot(Trajectories.SHOOT_BLUE.getTrajectory(odometry), this::canRun);
    }
}