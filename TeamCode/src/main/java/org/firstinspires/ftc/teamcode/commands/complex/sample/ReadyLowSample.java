package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawSample;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeLowSample;

public class ReadyLowSample extends ParallelRaceGroup {
    public ReadyLowSample() {
        super(
                new OuttakeLowSample(),
                new ClawSample()
        );
    }
}
