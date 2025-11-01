package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.*;
import org.firstinspires.ftc.teamcode.hardware.color.ColorSensorHandler;
import org.firstinspires.ftc.teamcode.hardware.motor.*;
import org.firstinspires.ftc.teamcode.hardware.vision.*;
import org.firstinspires.ftc.teamcode.mechanisms.drivetrain.FieldDrive;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Carousel;
import org.firstinspires.ftc.teamcode.mechanisms.intake.Roller;
import org.firstinspires.ftc.teamcode.mechanisms.odometry.Odometry;
import org.firstinspires.ftc.teamcode.mechanisms.outtake.Launcher;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Disabled
@TeleOp(name = "Robot", group = "FTC2025")
public abstract class Robot extends LinearOpMode {
    public Odometry odometry;
    public FieldDrive drivetrain;
    public Roller roller;
    public Carousel carousel;
    public Launcher launcher;
    public ColorSensorHandler colorSensor; // Color Sensor
    public VisionProcessor aprilTag;
    public VisionCamera visionPortal;
    public LinearOpMode game = this; // Game Object
    public Alliance alliance; // Game Alliance

    public enum Velocity {
        LAUNCHER_VELOCITY(1000);

        private final double velocity;
        Velocity(double velocity) {
            this.velocity = velocity;
        }

        public double getVelocity() {
            return velocity;
        }
    }

    @Override
    public void runOpMode() {
        configure();
        waitForStart();
        run();
    }

    // "Configure" Runs Before Op Mode. Default Configuration Below
    public void configure() {
        SafeHardwareMap safeHardwareMap = new SafeHardwareMap(hardwareMap, telemetry);

        // Creating New Drivetrain
        drivetrain = new FieldDrive(safeHardwareMap, telemetry);

        roller = new Roller(safeHardwareMap);
        launcher = new Launcher(safeHardwareMap, telemetry);

        carousel = new Carousel(safeHardwareMap, telemetry);
        colorSensor = safeHardwareMap.getColorSensor("ColorSensor");

        aprilTag = safeHardwareMap.getAprilTagProcessor();
        visionPortal = safeHardwareMap.getVisionPortal(aprilTag, "Camera");

        odometry = new Odometry(hardwareMap, telemetry);
        telemetry.update();
    }

    // All Robot Must Define their own Run Method
    abstract public void run();

    // Checks if OpModeIsActive
    public boolean canRun() {
        return game.opModeIsActive();
    }

    // Reading April Tags
    public AprilTagDetection getAprilTag() {
        if (aprilTag == null)
            return null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null || detections.isEmpty())
            return null;

        return detections.get(0);
    }

    public void updatePoseFromAprilTags() {
        AprilTagDetection aprilTag = getAprilTag();
        if (aprilTag == null) return;

        Pose2d tagFieldPos;
        AprilTagId aprilTagId = AprilTagId.getTagFromId(aprilTag.id);
        if (aprilTagId == null) return;

        switch (aprilTagId) {
            case BLUE_TAG:
                tagFieldPos = Positions.BLUE_TAG.getPose2D();
                break;

            case RED_TAG:
                tagFieldPos = Positions.RED_TAG.getPose2D();
                break;

            case GPP_TAG:
            case PGP_TAG:
            case PPG_TAG:
                tagFieldPos = Positions.OBELISK.getPose2D();
                break;

            default:
                return;
        }

        // --- Step 1: Input and coordinate alignment ---
        // Convert FTC camera coordinates to standard robot coordinates
        Pose2d tagCamPos = new Pose2d(
                aprilTag.ftcPose.y,
                -aprilTag.ftcPose.x,
                Math.toRadians(aprilTag.ftcPose.yaw)
        );

        // --- Step 2: Calculate the camera's field pose ---
        // A. Invert the tag-camera pose to find the camera-tag pose
        Pose2d camTagPos = new Pose2d(
                -(tagCamPos.getX() * Math.cos(-tagCamPos.getHeading()) - tagCamPos.getY() * Math.sin(-tagCamPos.getHeading())),
                -(tagCamPos.getX() * Math.sin(-tagCamPos.getHeading()) + tagCamPos.getY() * Math.cos(-tagCamPos.getHeading())),
                -tagCamPos.getHeading()
        );

        // B. Transform the camera's tag-relative pose into a field-relative pose
        Pose2d camFieldPos = new Pose2d(
                tagFieldPos.getX() + camTagPos.getX() * Math.cos(tagFieldPos.getHeading()) - camTagPos.getY() * Math.sin(tagFieldPos.getHeading()),
                tagFieldPos.getY() + camTagPos.getX() * Math.sin(tagFieldPos.getHeading()) + camTagPos.getY() * Math.cos(tagFieldPos.getHeading()),
                tagFieldPos.getHeading() + camTagPos.getHeading()
        );

        // --- Step 3: Calculate the robot's field pose ---
        // Subtract the camera's offset to find the robot's center
        Pose2d camRobotPos = Positions.CAMERA.getPose2D();
        Pose2d robotPos = new Pose2d(
                camFieldPos.getX() - (camRobotPos.getX() * Math.cos(camFieldPos.getHeading()) - camRobotPos.getY() * Math.sin(camFieldPos.getHeading())),
                camFieldPos.getY() - (camRobotPos.getX() * Math.sin(camFieldPos.getHeading()) + camRobotPos.getY() * Math.cos(camFieldPos.getHeading())),
                camFieldPos.getHeading() - camRobotPos.getHeading()
        );

        telemetry.addData("Robot Positon in Road Runner", odometry.getPoseEstimate());
        telemetry.addData("Robot Position in April Tag", robotPos);
        telemetry.addData("Difference", odometry.getPoseEstimate().minus(robotPos));
        telemetry.update();

        odometry.setPoseEstimate(robotPos);
    }
}