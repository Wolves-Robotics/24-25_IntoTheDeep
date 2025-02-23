package org.firstinspires.ftc.teamcode.commands.driveSubsystem;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class Drive extends InstantCommand {
    public Drive(double x, double y, double rot, boolean fieldCentric) {
        super(
                () -> DriveSubsystem.getInstance().drive(x, y, rot, fieldCentric)
        );
    }
}
