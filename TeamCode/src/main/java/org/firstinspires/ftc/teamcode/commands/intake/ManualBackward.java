package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ManualBackward extends InstantCommand {
    public ManualBackward() {
        super(
                () -> IntakeSubsystem.getInstance().manualBackward()
        );
    }
}
