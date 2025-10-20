package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.hardware.SafeHardwareMap;
import org.firstinspires.ftc.teamcode.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Camera Test", group = "Test")
public class TestCamera extends Robot {
    VisionProcessor aprilTag;
    VisionPortal visionPortal;

    @Override
    public void configure() {
        SafeHardwareMap safeHardwareMap = new SafeHardwareMap(hardwareMap, telemetry);
        aprilTag = safeHardwareMap.getAprilTagProcessor();
        visionPortal = safeHardwareMap.getVisionPortal(aprilTag, "Camera");
    }

    @Override
    public void run() {
        while (canRun()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            // Step through the list of detections and display info for each one.
            for (AprilTagDetection detection : currentDetections) {
                telemetry.addData("ID", detection.id);
            }

            telemetry.update();
        }

        visionPortal.close();
    }
}