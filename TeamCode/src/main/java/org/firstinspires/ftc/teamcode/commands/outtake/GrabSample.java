package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.OuttakesSubsystem;

public class GrabSample extends SequentialCommandGroup {
    public GrabSample() {
        super(
                new InstantCommand(() -> OuttakesSubsystem.getInstance().clawDown()),
                new WaitCommand(75),
                new InstantCommand(() -> OuttakesSubsystem.getInstance().clawClose())
        );
    }
}