package org.firstinspires.ftc.dynabytes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.dynabytes.Robot;
import org.firstinspires.ftc.dynabytes.Positions;
import org.firstinspires.ftc.dynabytes.Trajectories;
import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;

@Autonomous(name = "AutoPickAndShoot", group = "FTC2025")
public class AutoPickAndShoot extends Robot {
    @Override
    public void configure() {
        // Creating Drivetrain
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(Positions.BLUE_DOWN.getPose2D());
    }

    @Override
    public void run() {
        // Moving Robot
        moveRobot(Trajectories.trajectoryTo(Positions.BLUE_PPG.getPose2D(), drive));
        moveRobot(Trajectories.SHOOT_BLUE.getTrajectory(drive));
    }
}