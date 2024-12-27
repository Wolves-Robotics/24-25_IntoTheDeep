package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class SetIntakeTarget extends InstantCommand{
    public SetIntakeTarget(int target) {
        super(
                () -> IntakeSubsystem.getInstance().setTarget(target)
        );
    }
}
