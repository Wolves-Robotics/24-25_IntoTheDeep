package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

public class OpenClaw extends InstantCommand {
    public OpenClaw() {
        super(
                () -> OuttakeSubsystem.getInstance().clawOpen()
        );
    }
}
