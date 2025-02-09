package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ClawSample extends InstantCommand {
    public ClawSample() {
        super(
                () -> OuttakeSubsystem.getInstance().clawSample()
        );
    }
}
