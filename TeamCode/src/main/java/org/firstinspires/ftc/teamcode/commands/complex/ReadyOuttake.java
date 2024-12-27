package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawNeutral;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeDown;

public class ReadyOuttake extends ParallelRaceGroup {
    public ReadyOuttake() {
        super(
                new OuttakeDown(),
                new ClawNeutral(),
                new OpenClaw()
        );
    }
}
