package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.MockVisionPortal;
import org.firstinspires.ftc.teamcode.vision.MockVisionProcessor;
import org.firstinspires.ftc.teamcode.vision.ReflectionVisionPortal;
import org.firstinspires.ftc.teamcode.vision.ReflectionVisionProcessor;
import org.firstinspires.ftc.teamcode.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;

import java.lang.reflect.Method;

public class SafeHardwareMap {
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    public SafeHardwareMap(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    public DcMotor getMotor(String name) {
        try { return hardwareMap.get(DcMotor.class, name); }
        catch (IllegalArgumentException e) {
            telemetry.addData("Failed to Get Motor", name);
            return new DcMotor() {
                @Override public Manufacturer getManufacturer() { return null; }
                @Override public String getDeviceName() { return ""; }
                @Override public String getConnectionInfo() { return ""; }
                @Override public int getVersion() { return 0; }
                @Override public void resetDeviceConfigurationForOpMode() {}
                @Override public void close() {}
                private double power = 0;
                @Override public void setPower(double p) { power = p; }
                @Override public double getPower() { return power; }
                @Override public void setMode(RunMode mode) {}
                @Override public RunMode getMode() { return RunMode.RUN_WITHOUT_ENCODER; }
                @Override public void setZeroPowerBehavior(ZeroPowerBehavior behavior) {}
                @Override public ZeroPowerBehavior getZeroPowerBehavior() { return ZeroPowerBehavior.BRAKE; }
                public void setPowerFloat() {}
                public boolean getPowerFloat() { return false; }
                @Override public MotorConfigurationType getMotorType() { return null; }
                @Override public void setMotorType(MotorConfigurationType motorType) {}
                @Override public DcMotorController getController() { return null; }
                @Override public int getPortNumber() { return 0; }
                @Override public void setDirection(Direction dir) {}
                @Override public Direction getDirection() { return Direction.FORWARD; }
                @Override public int getCurrentPosition() { return 0; }
                @Override public void setTargetPosition(int pos) {}
                @Override  public int getTargetPosition() { return 0; }
                @Override public boolean isBusy() { return false; }
            };
        }
    }

    public Servo getServo(String name) {
        try { return hardwareMap.get(Servo.class, name); }
        catch (IllegalArgumentException e) {
            telemetry.addData("Failed to Get Motor", name);
            return new Servo() {
                private double pos = 0;
                @Override public Manufacturer getManufacturer() { return null; }
                @Override public String getDeviceName() { return ""; }
                @Override public String getConnectionInfo() { return ""; }
                @Override public int getVersion() { return 0; }
                @Override public void resetDeviceConfigurationForOpMode() {}
                @Override public void close() {}
                @Override public void setPosition(double p) { pos = p; }
                @Override public double getPosition() { return pos; }
                @Override public void scaleRange(double min, double max) {}
                @Override public ServoController getController() { return null; }
                @Override public int getPortNumber() { return 0; }
                @Override public void setDirection(Direction dir) {}
                @Override public Direction getDirection() { return Direction.FORWARD; }
            };
        }
    }

    public ColorSensor getColorSensor(String name) {
        try { return hardwareMap.get(ColorSensor.class, name); }
        catch (Exception e) {
            telemetry.addData("Failed to Get Motor", name);
            return new ColorSensor() {
                @Override public Manufacturer getManufacturer() { return null; }
                @Override public String getDeviceName() { return ""; }
                @Override public String getConnectionInfo() { return ""; }
                @Override public int getVersion() { return 0; }
                @Override public void resetDeviceConfigurationForOpMode() {}
                @Override public void close() {}
                @Override public int red() { return 0; }
                @Override public int green() { return 0; }
                @Override public int blue() { return 0; }
                @Override public int alpha() { return 0; }
                public int argb() { return 0; }
                public void enableLed(boolean enable) {}
                public void setI2cAddress(I2cAddr newAddress) {}
                public I2cAddr getI2cAddress() { return null; }
            };
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

    public VisionPortal getVisionPortal(VisionProcessor processor, String cameraName) {
        try {
            // If Vision Processor is Mock, Vision Portal Should Be Mock
            if (processor instanceof MockVisionProcessor) { return new MockVisionPortal(); }

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

            return new ReflectionVisionPortal(portal);
        }
        catch (Exception e) {
            return new MockVisionPortal();
        }
    }
}
