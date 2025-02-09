package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class CloseClaw extends InstantCommand {
    public CloseClaw() {
        super(
                () -> OuttakeSubsystem.getInstance().clawClose()
        );
    }
}
