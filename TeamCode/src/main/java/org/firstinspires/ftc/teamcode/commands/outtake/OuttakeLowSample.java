package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class OuttakeLowSample extends InstantCommand {
    public OuttakeLowSample() {
        super(
                () -> OuttakeSubsystem.getInstance().setTarget(1300)
        );
    }
}
