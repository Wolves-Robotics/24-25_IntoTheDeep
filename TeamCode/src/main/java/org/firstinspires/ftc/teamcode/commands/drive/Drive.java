package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class Drive extends InstantCommand {
    public Drive(double x, double y , double rot, boolean robotCentric) {
        super(
                () -> DriveSubsystem.getInstance().drive(x, y, rot, robotCentric)
        );
    }
    public Drive(double x1, double x2, double y , double rot, boolean robotCentric) {
        super(
                () -> DriveSubsystem.getInstance().drive(x1, x2, y, rot, robotCentric)
        );
    }
}
