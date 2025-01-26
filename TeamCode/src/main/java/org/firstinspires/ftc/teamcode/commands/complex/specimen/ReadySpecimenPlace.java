package org.firstinspires.ftc.teamcode.commands.complex.specimen;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawSpecimenPlace;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeHighSpecimen;

public class ReadySpecimenPlace extends ParallelRaceGroup {
    public ReadySpecimenPlace() {
        super(
                new CloseClaw(),
                new ClawSpecimenPlace(),
                new OuttakeHighSpecimen()
        );
    }
}
