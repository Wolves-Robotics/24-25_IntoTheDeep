package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;

public class FollowPath extends CommandBase {
    private final Follower follower;
    private final Path path;
    private final boolean holdEnd;

    public FollowPath(Follower _follower, Path _path, boolean _holdEnd) {
        follower = _follower;
        path = _path;
        holdEnd = _holdEnd;
    }

    @Override
    public void initialize() {
        follower.followPath(path, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
