package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Camera Test", group = "Test")
public class TestCamera extends Robot {
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    @Override
    public void configure() {
        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "camera"))
                .addProcessor(aprilTag)
                .build();
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