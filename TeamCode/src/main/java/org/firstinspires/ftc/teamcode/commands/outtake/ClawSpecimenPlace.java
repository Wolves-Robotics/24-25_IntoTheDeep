package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ClawSpecimenPlace extends InstantCommand {
    public ClawSpecimenPlace() {
        super(
                () -> OuttakeSubsystem.getInstance().clawSpecimenPlace()
        );
    }
}
