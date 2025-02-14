package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class IntakeRetract extends CommandBase {
    private final IntakeSubsystem intakeSubsystem;


    public IntakeRetract() {
        intakeSubsystem = IntakeSubsystem.getInstance();
    }

    @Override
    public void initialize() {
        intakeSubsystem.setTarget(0);
    }

    @Override
    public void execute() {

    }
}
