package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TestLauncher", group = "DYNABYTES")
public class TestLauncher extends LinearOpMode {
    DcMotor OM;
    @Override
    public void runOpMode() {
        // Initializing
        OM = hardwareMap.get(DcMotor.class, "OM");
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0) {
                OM.setPower(OM.getPower() + (0.1 * gamepad1.left_stick_y));
            }

            telemetry.addData("Launcher Power", OM.getPower());
            telemetry.update();
        }
    }
}
