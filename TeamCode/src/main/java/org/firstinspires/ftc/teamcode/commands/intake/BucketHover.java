package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class BucketHover extends InstantCommand {
    public BucketHover() {
        super(
                () -> IntakeSubsystem.getInstance().bucketHover()
        );
    }
}
