package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class ManualStop extends InstantCommand {
    public ManualStop() {
        super(
                () -> IntakeSubsystem.getInstance().stopManual()
        );
    }
}
