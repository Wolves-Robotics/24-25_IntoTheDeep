package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ClawSpecimenGrab extends InstantCommand {
    public ClawSpecimenGrab() {
        super(
                () -> OuttakeSubsystem.getInstance().clawSpecimenGrab()
        );
    }
}
