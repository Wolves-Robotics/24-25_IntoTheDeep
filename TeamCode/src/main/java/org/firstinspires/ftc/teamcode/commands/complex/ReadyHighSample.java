package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawSample;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeHighSample;

public class ReadyHighSample extends ParallelRaceGroup {
    public ReadyHighSample() {
        super(
                new OuttakeHighSample(),
                new ClawSample()
        );
    }
}
