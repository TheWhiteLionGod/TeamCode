package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.auto.*;

@Disabled
@TeleOp(name="Game", group="FTC2025")
public abstract class Game extends LinearOpMode {
    Robot auto;
    Robot teleop;

    @Override
    public void runOpMode() {
        if (auto == null || teleop == null) {
            throw new NullPointerException("Auto or TeleOp Mode Not Defined");
        }
        waitForStart();

        // Starting Autonomous
        auto.configure();
        auto.game = this;

        double start_time = getRuntime();
        Thread autoThread = new Thread(auto::run);
        autoThread.start();

        // Making Sure Auto Runs for Only 30 Seconds
        while (autoThread.isAlive() && getRuntime() - start_time < 30) {}
        if (autoThread.isAlive()) { autoThread.interrupt(); }

        // Starting TeleOp
        teleop.configure();
        teleop.drive.setPoseEstimate(auto.drive.getPoseEstimate()); // Transferring Auto Position Data

        teleop.game = this;
        teleop.run();
    }
}
