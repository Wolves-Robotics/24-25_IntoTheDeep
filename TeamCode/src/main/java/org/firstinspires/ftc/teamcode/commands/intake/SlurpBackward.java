package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SlurpBackward extends InstantCommand {
    public SlurpBackward() {
        super(
                () -> IntakeSubsystem.getInstance().slurpBackward()
        );
    }
}
