package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.lang.reflect.Method;
import java.util.Collection;
import java.util.List;
import java.util.Collections;

public class ReflectionVisionProcessor implements VisionProcessor {
    private final Object processor;
    private final Method getDetections;

    public ReflectionVisionProcessor(Object processor) throws ReflectiveOperationException {
        this.processor = processor;
        this.getDetections = processor.getClass().getMethod("getDetections");
    }

    @Override
    @SuppressWarnings("unchecked")
    public List<AprilTagDetection> getDetections() {
        try { return (List<AprilTagDetection>) getDetections.invoke(processor); }
        catch (Exception e) { return Collections.emptyList(); }
    }

    public Object getProcessor() { return processor; }
}
