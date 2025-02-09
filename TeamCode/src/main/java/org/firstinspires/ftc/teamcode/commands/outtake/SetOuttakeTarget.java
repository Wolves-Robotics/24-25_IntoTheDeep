package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class SetOuttakeTarget extends InstantCommand {
    public SetOuttakeTarget(int target) {
        super(
                () -> OuttakeSubsystem.getInstance().setTarget(target)
        );
    }
}
