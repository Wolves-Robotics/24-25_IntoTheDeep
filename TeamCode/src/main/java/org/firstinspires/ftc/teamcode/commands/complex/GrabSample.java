package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.intake.BucketNeutral;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawDown;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;

public class GrabSample extends SequentialCommandGroup {
    public GrabSample() {
        super(
                new ParallelCommandGroup(
                        new BucketNeutral(),
                        new ClawDown()
                ).withTimeout(50),
                new CloseClaw()
        );
    }
}
