package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawSample;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeHighSample;

public class ReadySample extends ParallelRaceGroup {
    public ReadySample() {
        super(
                new OuttakeHighSample(),
                new ClawSample()
        );
    }
}
