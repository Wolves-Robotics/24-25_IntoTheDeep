package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ManualForward extends InstantCommand {
    public ManualForward() {
        super(
                () -> IntakeSubsystem.getInstance().manualForward()
        );
    }
}
