package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.intake.BucketNeutral;
import org.firstinspires.ftc.teamcode.commands.intake.BucketUp;
import org.firstinspires.ftc.teamcode.commands.intake.DoorOpen;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpStop;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawDown;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawSample;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeHighSample;

public class FullTransfer extends SequentialCommandGroup {
    public FullTransfer() {
        super(
                new BucketUp(),
                new SetIntakeTarget(0),
                new WaitCommand(450),
                new DoorOpen(),
                new WaitCommand(600),
                new SlurpStop(),

                new ParallelCommandGroup(
                        new BucketNeutral(),
                        new ClawDown()
                ),
                new WaitCommand(100),
                new CloseClaw(),
                new WaitCommand(50),

                new OuttakeHighSample(),
                new ClawSample()
        );
    }
}
