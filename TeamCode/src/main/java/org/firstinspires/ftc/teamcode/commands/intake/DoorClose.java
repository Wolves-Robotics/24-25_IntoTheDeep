package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class DoorClose extends InstantCommand {
    public DoorClose() {
        super(
                () -> IntakeSubsystem.getInstance().doorClose()
        );
    }
}
