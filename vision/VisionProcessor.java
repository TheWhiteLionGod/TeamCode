package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public interface VisionProcessor {
    List<AprilTagDetection> getDetections();
}
