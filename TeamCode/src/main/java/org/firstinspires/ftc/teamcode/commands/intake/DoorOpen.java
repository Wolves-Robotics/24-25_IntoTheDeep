package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class DoorOpen extends InstantCommand {
    public DoorOpen() {
        super(
                () -> IntakeSubsystem.getInstance().doorOpen()
        );
    }
}
