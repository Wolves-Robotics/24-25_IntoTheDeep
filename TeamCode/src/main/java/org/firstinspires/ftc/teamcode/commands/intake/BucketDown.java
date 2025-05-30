package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class BucketDown extends InstantCommand {
    public BucketDown() {
        super(
                () -> IntakeSubsystem.getInstance().bucketDown()
        );
    }
}
