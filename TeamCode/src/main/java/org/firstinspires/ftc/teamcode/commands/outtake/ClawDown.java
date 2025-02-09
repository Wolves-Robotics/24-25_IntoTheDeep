package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ClawDown extends InstantCommand {
    public ClawDown() {
        super(
                () -> OuttakeSubsystem.getInstance().clawDown()
        );
    }
}
