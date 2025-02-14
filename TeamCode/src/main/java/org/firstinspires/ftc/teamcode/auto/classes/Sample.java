package org.firstinspires.ftc.teamcode.auto.classes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.commands.drive.FollowPath;
import org.firstinspires.ftc.teamcode.commands.outtake.setOuttake;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Color;
import org.firstinspires.ftc.teamcode.utils.collections.outtake.OuttakeArmState;


public class Sample extends BaseAuto {
    private Pose
            startPose, initBucketPose,
            sample1Pose, sample1BucketPose,
            sample2Pose, sample2BucketPose,
            sample3Pose, sample3BucketPose,
            parkControlPose, parkPose;

    private Path
            scoreSample1Path,
            getSample2Path, scoreSample2Path,
            getSample3Path, scoreSample3Path,
            getSample4Path, scoreSample4Path,
            parkPath;

    private final Follower follower;

    protected Sample(Color _color) {
        super(_color);

        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(RobotHardware.getInstance().getHardwareMap());

        startPose = new Pose(100, 100, 0);
        initBucketPose = new Pose(83.25, 106, Math.toRadians(45));

        follower.setStartingPose(startPose);

        scoreSample1Path = new Path(new BezierLine(
                new Point(startPose),
                new Point(initBucketPose)
        ));
        scoreSample1Path.setLinearHeadingInterpolation(startPose.getHeading(), initBucketPose.getHeading());

        CommandScheduler.getInstance().schedule(
                new RunCommand(follower::update),

                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new setOuttake(OuttakeArmState.sampleScoring, 1940, false),
                                new FollowPath(follower, scoreSample1Path, true)
                        ),
                        new InstantCommand(() -> OuttakeSubsystem.getInstance().setClawOpen(true))
                ));
    }

    @Override
    public void update() {

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
