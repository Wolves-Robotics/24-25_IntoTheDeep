package org.firstinspires.ftc.teamcode.teleop.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.OuttakeSubsystem;

public class intakeZero extends CommandBase {
    private IntakeSubsystem intakeSubsystem;

    public intakeZero(IntakeSubsystem _intakeSubsystem) {
        intakeSubsystem = _intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.targetZero();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.getPos() < 50;
    }
}
