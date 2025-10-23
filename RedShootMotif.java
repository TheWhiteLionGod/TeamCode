package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoShootMotif;

@TeleOp(name="RedShootMotif", group="FTC2025")
public class RedShootMotif extends Game {
    public RedShootMotif() {
        auto = new AutoShootMotif();
        teleop = new Controller();

        auto.alliance = Alliance.RED;
        teleop.alliance = Alliance.RED;
    }
}
