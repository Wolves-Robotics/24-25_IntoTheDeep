package org.firstinspires.ftc.teamcode.commands.complex;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.commands.intake.BucketUp;
import org.firstinspires.ftc.teamcode.commands.intake.DoorOpen;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpStop;

public class IntakeRetract extends SequentialCommandGroup {
    public IntakeRetract() {
        super(
                new BucketUp(),
                new SetIntakeTarget(0).withTimeout(50),
                new DoorOpen().withTimeout(200),
                new SlurpStop()
        );
    }
}
