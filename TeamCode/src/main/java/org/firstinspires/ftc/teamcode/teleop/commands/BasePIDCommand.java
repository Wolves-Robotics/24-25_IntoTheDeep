package org.firstinspires.ftc.teamcode.teleop.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.hardware.subsystems.BaseSubsystem;

public class BasePIDCommand extends CommandBase {
    BaseSubsystem subsystem;
    public BasePIDCommand(BaseSubsystem _subsystem) {
        subsystem = _subsystem;
    }
}
