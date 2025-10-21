package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.hardware.vision.*;
import org.firstinspires.ftc.teamcode.hardware.motor.*;
import org.firstinspires.ftc.teamcode.hardware.color.*;

import java.lang.reflect.Method;

public class SafeHardwareMap {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public SafeHardwareMap(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public MotorHandler getMotor(String name) {
        try { return new HardwareMotor(hardwareMap.get(DcMotorEx.class, name)); }
        catch (IllegalArgumentException e) {
            telemetry.addData("Failed to Get Motor", name);
            return new MockMotor();
        }
    }

    public ServoHandler getServo(String name) {
        try { return new HardwareServo(hardwareMap.get(Servo.class, name)); }
        catch (IllegalArgumentException e) {
            telemetry.addData("Failed to Get Motor", name);
            return new MockServo();
        }
    }

    public ColorSensorHandler getColorSensor(String name) {
        try { return new HardwareColorSensor(hardwareMap.colorSensor.get(name)); }
        catch (IllegalArgumentException e) {
            telemetry.addData("Failed to Get Motor", name);
            return new MockColorSensor();
        }
    }

    public VisionProcessor getAprilTagProcessor() {
        try {
            // Trying to Get April Tag Classes
            Class<?> builderClass = Class.forName("org.firstinspires.ftc.vision.AprilTagProcessor$Builder");

            // Creating Builder Object
            Object builder = builderClass.getDeclaredConstructor().newInstance();

            // Building Object
            Method build = builderClass.getMethod("build");
            Object processor = build.invoke(builder);

            return new ReflectionVisionProcessor(processor);
        }
        catch (Exception e) {
            telemetry.addLine("Failed to Get Motor: Camera");
            return new MockVisionProcessor();
        }
    }

    public VisionCamera getVisionPortal(VisionProcessor processor, String cameraName) {
        try {
            // If Vision Processor is Mock, Vision Portal Should Be Mock
            if (processor instanceof MockVisionProcessor) { return new MockVisionCamera(); }

            // Getting Classes
            Class<?> portalBuilderClass = Class.forName("org.firstinspires.ftc.vision.VisionPortal$Builder");

            // Creating Object of Builder Class
            Object portalBuilder = portalBuilderClass.getDeclaredConstructor().newInstance();

            // Set Camera Method on Builder
            WebcamName webcam = hardwareMap.get(WebcamName.class, cameraName);
            Method setCamera = portalBuilderClass.getMethod("setCamera", WebcamName.class);
            setCamera.invoke(portalBuilder, webcam);

            // Finding Add Processor Method and Adding Processor
            Object aprilTagProcessor = ((ReflectionVisionProcessor) processor).getProcessor();
            for (Method m : portalBuilderClass.getMethods()) {
                if (m.getName().equals("addProcessor") && m.getParameterTypes().length == 1) {
                    m.invoke(portalBuilder, aprilTagProcessor);
                    break;
                }
            }

            Method build = portalBuilderClass.getMethod("build");
            Object portal = build.invoke(portalBuilder);

            return new ReflectionVisionCamera(portal);
        }
        catch (Exception e) {
            return new MockVisionCamera();
        }
    }
}
