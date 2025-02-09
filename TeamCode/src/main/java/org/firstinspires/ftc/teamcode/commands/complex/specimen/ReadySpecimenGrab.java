package org.firstinspires.ftc.teamcode.commands.complex.specimen;

import com.arcrobotics.ftclib.command.ParallelRaceGroup;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawSpecimenGrab;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeDown;

public class ReadySpecimenGrab extends ParallelRaceGroup {
    public ReadySpecimenGrab() {
        super(
                new OpenClaw(),
                new ClawSpecimenGrab(),
                new OuttakeDown()
        );
    }
}
