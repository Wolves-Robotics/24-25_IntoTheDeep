package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawDown;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;

public class PlaceSample extends ParallelRaceGroup {
    public PlaceSample() {
        super(
                new OpenClaw(),
                new WaitCommand(100),
                new ReadyOuttake()
        );
    }
}
