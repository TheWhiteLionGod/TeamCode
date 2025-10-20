package org.firstinspires.ftc.dynabytes.test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.dynabytes.Robot;

@Autonomous(name="Limelight Test", group="Auto")
public class TestLimelight extends Robot {
    Limelight3A limelight;
    @Override
    public void configure() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.pipelineSwitch(0); // Switch to pipeline number 0
    }

    @Override
    public void run() {
        limelight.start();

        while (canRun()) {
            limelight.updateRobotOrientation(Math.toDegrees(drive.getPoseEstimate().getHeading()));
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid() && !result.getFiducialResults().isEmpty()) {
                int id = result.getFiducialResults().get(0).getFiducialId();
                telemetry.addData("ID", id);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            telemetry.update();
        }
    }
}
