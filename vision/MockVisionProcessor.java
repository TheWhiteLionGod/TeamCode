package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Collections;
import java.util.List;

public class MockVisionProcessor implements VisionProcessor {
    @Override
    public List<AprilTagDetection> getDetections() {
        return Collections.emptyList();
    }
}
