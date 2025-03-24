package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.function.BooleanSupplier;

public class FollowPath extends CommandBase {
    private DriveSubsystem driveSubsystem;
    private PathChain path;
    private boolean holdEnd;

    private BooleanSupplier override;

    public FollowPath(PathChain _path, boolean _holdEnd) {
        driveSubsystem = DriveSubsystem.getInstance();
        path = _path;
        holdEnd = _holdEnd;
    }

    public FollowPath(PathChain _path, boolean _holdEnd, BooleanSupplier _override) {
        driveSubsystem = DriveSubsystem.getInstance();
        path = _path;
        holdEnd = _holdEnd;

        override = _override;
    }

    public FollowPath(Path _path, boolean _holdEnd) {
        this(new PathChain(_path), _holdEnd);
    }

    @Override
    public void initialize() {
        driveSubsystem.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return (override != null && override.getAsBoolean()) ||
                (override == null && driveSubsystem.isNotBusy());
    }
}
