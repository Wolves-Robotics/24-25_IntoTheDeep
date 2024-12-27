package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.intake.BucketDown;
import org.firstinspires.ftc.teamcode.commands.intake.DoorClose;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpForward;

public class IntakeDown extends ParallelRaceGroup {
    public IntakeDown() {
        super(
                new ReadyOuttake(),
                new DoorClose(),
                new BucketDown(),
                new SlurpForward()
        );
    }
}
