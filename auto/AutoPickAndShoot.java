package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Positions;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;

@Disabled
@Autonomous(name = "AutoPickAndShoot", group = "FTC2025")
public class AutoPickAndShoot extends Robot {
    @Override
    public void configure() {
        super.configure();
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