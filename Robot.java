package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.path.EmptyPathSegmentException;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.*;
import org.firstinspires.ftc.teamcode.hardware.color.ColorSensorHandler;
import org.firstinspires.ftc.teamcode.hardware.motor.*;
import org.firstinspires.ftc.teamcode.hardware.vision.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.time.Duration;
import java.time.Instant;
import java.util.List;

@Disabled
@TeleOp(name = "Robot", group = "FTC2025")
public abstract class Robot extends LinearOpMode {
    public SampleMecanumDrive drive; // Roadrunner Driver
    public MotorHandler BL, FL, FR, BR; // Wheel Motors
    public MotorHandler roller, launcher; // Intake Outtake Motors
    public ServoHandler carousel, lift; // Carousel and Lift Servos
    public ColorSensorHandler colorSensor; // Color Sensor
    public VisionProcessor aprilTag;
    public VisionCamera visionPortal;
    public FunctionThread spinCarouselThread, runLauncherThread; // Threads
    public LinearOpMode game = this; // Game Object
    public Alliance alliance; // Game Alliance

    // Gear Mode Variables
    Instant gearSwitchTime = Instant.now();
    GearMode gearMode = GearMode.THIRD_GEAR;

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

        BL = safeHardwareMap.getMotor("BL");
        FL = safeHardwareMap.getMotor("FL");
        FR = safeHardwareMap.getMotor("FR");
        BR = safeHardwareMap.getMotor("BR");

        BL.setDirection(Direction.REVERSE);
        FL.setDirection(Direction.REVERSE);

        BL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        FL.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        launcher = safeHardwareMap.getMotor("Launcher");
        roller = safeHardwareMap.getMotor("Roller");

        launcher.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        roller.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);

        carousel = safeHardwareMap.getServo("Carousel");
        lift = safeHardwareMap.getServo("Lift");
        colorSensor = safeHardwareMap.getColorSensor("ColorSensor");

        aprilTag = safeHardwareMap.getAprilTagProcessor();
        visionPortal = safeHardwareMap.getVisionPortal(aprilTag, "Camera");

        drive = new SampleMecanumDrive(hardwareMap);
        telemetry.update();
    }

    // All Robot Must Define their own Run Method
    abstract public void run();

    // Checks if OpModeIsActive
    public boolean canRun() {
        return game.opModeIsActive();
    }

    // Update Odometry via RoadRunner
    public void updateOdometry() {
        drive.update(); // Update Roadrunner
        updatePoseFromAprilTags(); // Update April Tag
    }

    // Making Robot Follow Trajectory
    public void moveRobot(TrajectorySequence traj) {
        drive.followTrajectorySequenceAsync(traj);

        // Updating Robot Position
        while (canRun() && drive.isBusy()) {
            drive.update();
            telemetry.addData("Robot Position: ", drive.getPoseEstimate());
            telemetry.update();
        }

        // Printing Deviation in Position Once Completed
        telemetry.addData("Expected End Position: ", traj.end());
        telemetry.addData("Current Position: ", drive.getPoseEstimate());
        telemetry.update();
    }

    // These Functions Make Robot Go To Base
    public void goToRedBase() {
        try {
            drive.followTrajectorySequence(Trajectories.GO_RED_BASE.getTrajectory(drive));
        }
        catch (EmptyPathSegmentException e) {
            System.out.println("Empty Path Segment Exception has Occurred\n" +
                    "This means that the robot is already at the defined position");
        }
    }

    public void goToBlueBase() {
        try {
            drive.followTrajectorySequence(Trajectories.GO_BLUE_BASE.getTrajectory(drive));
        }
        catch (EmptyPathSegmentException e) {
            System.out.println("Empty Path Segment Exception has Occurred\n" +
                    "This means that the robot is already at the defined position");
        }
    }

    // Change Gear Mode of Robot
    public void gearModeUp() {
        changeGearMode(gearMode.gearUp());
    }

    public void gearModeDown() {
        changeGearMode(gearMode.gearDown());
    }

    private void changeGearMode(GearMode nextGear) {
        if (Duration.between(gearSwitchTime, Instant.now()).toMillis() >= Timings.GEAR_COOLDOWN.getMilliseconds()) {
            gearMode = nextGear;
            gearSwitchTime = Instant.now();

            telemetry.addData("Current Gear Mode", gearMode.getGear());
            telemetry.update();
        }
    }


    // Field Drive Movement
    public void fieldDriveMove(double pwr_x, double pwr_y) {
        /* Adjust Joystick X/Y inputs by navX MXP yaw angle */
        double yaw_radians = drive.getPoseEstimate().getHeading();
        double temp = pwr_y * Math.cos(yaw_radians) + pwr_x * Math.sin(yaw_radians);
        pwr_x = -pwr_y * Math.sin(yaw_radians) + pwr_x * Math.cos(yaw_radians);
        pwr_y = temp;

        /* At this point, Joystick X/Y (strafe/forward) vectors have been */
        /* rotated by the gyro angle, and can be sent to drive system */
        moveDrivetrain(pwr_x, pwr_y);
    }

    // Regular Movement
    public void moveDrivetrain(double pwrx, double pwry) {
        double gear_pwr = gearMode.getMultiplier();
        BL.setPower(gear_pwr*(-pwrx-pwry));
        FR.setPower(gear_pwr*(-pwrx-pwry));

        FL.setPower(gear_pwr*(pwrx-pwry));
        BR.setPower(gear_pwr*(pwrx-pwry));
    }

    // Turning
    public void turnDrivetrain(double pwr) {
        double gear_pwr = gearMode.getMultiplier();
        BL.setPower(gear_pwr*pwr);
        FR.setPower(gear_pwr*-pwr);

        FL.setPower(0);
        BR.setPower(0);
    }

    // Stopping Drivetrain
    public void stopDrivetrain() {
        BL.setPower(0);
        FL.setPower(0);
        FR.setPower(0);
        BR.setPower(0);
    }

    // Intake Functions
    public void forwardIntake() {
        roller.setPower(0.5);
    }

    public void backwardIntake() {
        roller.setPower(-0.5);
    }

    public void stopIntake() {
        roller.setPower(0);
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

        telemetry.addData("Robot Positon in Road Runner", drive.getPoseEstimate());
        telemetry.addData("Robot Position in April Tag", robotPos);
        telemetry.addData("Difference", drive.getPoseEstimate().minus(robotPos));
        telemetry.update();

        drive.setPoseEstimate(robotPos);
    }

    // Rotating Carousel to Next Position
    public void spinCarousel() {
        double cur_pos = carousel.getPosition();
        if (cur_pos == ServoPos.CAROUSEL_POS_1.getPos()) {
            carousel.setPosition(ServoPos.CAROUSEL_POS_2.getPos());
        }
        else if (cur_pos == ServoPos.CAROUSEL_POS_2.getPos()) {
            carousel.setPosition(ServoPos.CAROUSEL_POS_3.getPos());
        }
        else if (cur_pos == ServoPos.CAROUSEL_POS_3.getPos()) {
            carousel.setPosition(ServoPos.CAROUSEL_POS_1.getPos());
        }
    }

    // Spinning Carousel to Given Position
    public void spinCarousel(double new_pos) {
        carousel.setPosition(new_pos);
    }

    // Spinning Carousel for Specific Ball Color
    public void findGreenBall() throws InterruptedException {
        findBall(HueValues.GREEN_MIN, HueValues.GREEN_MAX);
    }
    public void findPurpleBall() throws InterruptedException {
        findBall(HueValues.PURPLE_MIN, HueValues.PURPLE_MAX);
    }

    public void findBall(HueValues min, HueValues max) throws InterruptedException {
        for (int i = 0; i < 3; i++) {
            float[] hsvValues = colorSensor.hsv();
            if (hsvValues[0] >= min.getHue()
                    && hsvValues[0] <= max.getHue()) {
                return;
            }
            spinCarousel();
            Thread.sleep((long) Timings.CAROUSEL_SPIN_TIME.getMilliseconds());
        }
    }

    public void startLauncher() throws InterruptedException {
        launcher.setPower(1);
        while (launcher.getVelocity() < Velocity.LAUNCHER_VELOCITY.getVelocity()) {}

        lift.setPosition(ServoPos.LIFT_OUT_POS.getPos());
        Thread.sleep((long) Timings.LAUNCHER_SHOOT_TIME.getMilliseconds());
    }

    public void stopLauncher() {
        lift.setPosition(ServoPos.LIFT_IN_POS.getPos());
        launcher.setPower(0);
    }
}