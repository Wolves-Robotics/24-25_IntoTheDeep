package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawSpecimenPlace;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeHighSpecimen;

public class ReadySpecimen extends ParallelRaceGroup {
    public ReadySpecimen() {
        super(
                new CloseClaw(),
                new ClawSpecimenPlace(),
                new OuttakeHighSpecimen()
        );
    }
}
