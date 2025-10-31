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
            throw new NullPointerException("Auto or TeleOp Mode is Null");
        }
        waitForStart();

        // Starting Autonomous
        auto.game = this;
        auto.configure();

        double start_time = getRuntime();
        Thread autoThread = new Thread(auto::run);
        autoThread.start();

        // Making Sure Auto Runs for Only 30 Seconds
        while (autoThread.isAlive() && getRuntime() - start_time < 30) {}
        // Force Stop as Interrupt Exceptions are Caught
        if (autoThread.isAlive()) { autoThread.stop(); }

        // Starting TeleOp
        teleop.configure();
        teleop.odometry.setPoseEstimate(auto.odometry.getPoseEstimate()); // Transferring Auto Position Data

        teleop.game = this;
        teleop.run();
    }
}
