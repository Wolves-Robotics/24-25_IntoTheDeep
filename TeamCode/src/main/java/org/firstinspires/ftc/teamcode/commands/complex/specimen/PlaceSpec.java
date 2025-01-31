package org.firstinspires.ftc.teamcode.commands.complex.specimen;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.outtake.ClawSpecimenGrab;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeDown;

public class PlaceSpec extends SequentialCommandGroup {
    public PlaceSpec() {
        super(
                new OuttakeDown(),
                new WaitCommand(600),
                new ReadySpecimenGrab()
        );
    }
}
