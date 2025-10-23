package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.AutoShootMotif;

@TeleOp(name="BlueShootMotif", group="FTC2025")
public class BlueShootMotif extends Game {
    public BlueShootMotif() {
        auto = new AutoShootMotif();
        teleop = new Controller();

        auto.alliance = Alliance.BLUE;
        teleop.alliance = Alliance.BLUE;
    }
}
