package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.*;

@TeleOp(name="Game", group="FTC2025")
public class Game extends LinearOpMode {
    Robot auto;
    Robot teleop;

    @Override
    public void runOpMode() {
        auto = new AutoShootMotif();
        teleop = new Controller();

        waitForStart();

        // Starting Autonomous
        auto.configure();
        auto.game = this;
        auto.run();

        // Starting TeleOp
        teleop.configure();
        teleop.drive.setPoseEstimate(auto.drive.getPoseEstimate()); // Transferring Auto Position Data

        teleop.game = this;
        teleop.run();
    }
}
