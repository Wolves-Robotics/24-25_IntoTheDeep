package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class OuttakeHighSample extends InstantCommand {
    public OuttakeHighSample() {
        super(
                () -> OuttakeSubsystem.getInstance().setTarget(760)
        );
    }
}
