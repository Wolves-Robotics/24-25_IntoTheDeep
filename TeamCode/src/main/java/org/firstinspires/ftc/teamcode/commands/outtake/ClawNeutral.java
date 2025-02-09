package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class ClawNeutral extends InstantCommand {
    public ClawNeutral() {
        super(
                () -> OuttakeSubsystem.getInstance().clawNeutral()
        );
    }
}
