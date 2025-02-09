package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.intake.BucketNeutral;
import org.firstinspires.ftc.teamcode.commands.intake.BucketUp;
import org.firstinspires.ftc.teamcode.commands.intake.DoorOpen;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpStop;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawDown;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;

public class IntakeRetractAndGrab extends SequentialCommandGroup {
    public IntakeRetractAndGrab() {
        super(
                new BucketUp()
                ,new SetIntakeTarget(0)
                ,new WaitCommand(500)
                ,new SlurpStop()
                ,new WaitCommand(50)
                ,new DoorOpen()
                ,new WaitCommand(800)
                ,new BucketNeutral()
                ,new ClawDown()
                ,new WaitCommand(100)
                ,new CloseClaw()
        );
    }
}
