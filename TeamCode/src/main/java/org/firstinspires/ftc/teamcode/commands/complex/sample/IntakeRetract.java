package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.intake.BucketUp;
import org.firstinspires.ftc.teamcode.commands.intake.DoorOpen;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpStop;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeRetract extends SequentialCommandGroup {

    public IntakeRetract() {
        super(
                new BucketUp()
                ,new SetIntakeTarget(0)
                ,new WaitCommand(500)
                ,new DoorOpen()
                ,new InstantCommand(() -> IntakeSubsystem.getInstance().setIntakeSpeed())
                ,new WaitCommand(100)
                ,new SlurpStop()
        );
    }
}
