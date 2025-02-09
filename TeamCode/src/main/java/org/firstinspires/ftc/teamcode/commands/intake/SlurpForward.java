package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SlurpForward extends InstantCommand {
    public SlurpForward() {
        super(
                () -> IntakeSubsystem.getInstance().slurpForward()
        );
    }
}
