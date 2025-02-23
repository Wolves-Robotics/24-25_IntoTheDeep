package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.intake.BucketNeutral;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawDown;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;

public class GrabSample extends SequentialCommandGroup {
    public GrabSample() {
        super(
                new BucketNeutral(),
                new ClawDown(),
                new WaitCommand(75),
                new CloseClaw()
        );
    }
}
