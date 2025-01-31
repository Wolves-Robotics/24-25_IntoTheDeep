package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;

public class PlaceSample extends SequentialCommandGroup {
    public PlaceSample() {
        super(
                new OpenClaw(),
                new WaitCommand(125),
                new ReadyOuttake()
        );
    }
}
