package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class BucketUp extends InstantCommand {
    public BucketUp() {
        super(
                () -> IntakeSubsystem.getInstance().bucketUp()
        );
    }
}
