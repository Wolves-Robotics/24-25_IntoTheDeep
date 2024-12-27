package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class BucketNeutral extends InstantCommand {
    public BucketNeutral() {
        super(
                () -> IntakeSubsystem.getInstance().bucketNeutral()
        );
    }
}
