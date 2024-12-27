package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SlurpStop extends InstantCommand {
    public SlurpStop() {
        super(
                () -> IntakeSubsystem.getInstance().slurpStop()
        );
    }
}
