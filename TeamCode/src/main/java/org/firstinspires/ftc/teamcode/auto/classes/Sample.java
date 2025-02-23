package org.firstinspires.ftc.teamcode.auto.classes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.collections.Color;
import org.firstinspires.ftc.teamcode.commands.complex.sample.GrabSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetract;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyHighSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyOuttake;
import org.firstinspires.ftc.teamcode.commands.drive.FollowPath;
import org.firstinspires.ftc.teamcode.commands.intake.BucketHover;
import org.firstinspires.ftc.teamcode.commands.intake.DoorClose;
import org.firstinspires.ftc.teamcode.commands.complex.sample.GetSample;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;

public class Sample extends BaseAuto {
    private Pose
            startPose, sample1BucketPose,
            sample2Pose, sample2BucketPose,
            sample3Pose, sample3BucketPose,
            sample4Pose, sample4BucketPose,
            submersible1Control1, submersible1Pose,
            sample5BucketControl1, sample5BucketPose,
            parkControlPose, parkPose;

    private Path
            scoreSample1Path,
            getSample2Path, scoreSample2Path,
            getSample3Path, scoreSample3Path,
            getSample4Path, scoreSample4Path,
            submersible1Path, scoreSample5Path,
            parkPath;

    protected Sample(Color _color) {
        super(_color);

        startPose = new Pose(7, 109, Math.toRadians(-90));
        sample1BucketPose = new Pose(16.7, 132.8, Math.toRadians(-20));

        sample2Pose = new Pose(25.9, 128.8, Math.toRadians(-19));
        sample2BucketPose = new Pose(16.7, 132.8, Math.toRadians(-20));

        sample3Pose = new Pose(25.6, 135.2, Math.toRadians(-7));
        sample3BucketPose = new Pose(17.7, 133.8, Math.toRadians(-20));

        sample4Pose = new Pose(25.7, 133, Math.toRadians(22));
        sample4BucketPose = new Pose(17.7, 133.8, Math.toRadians(-20));

        submersible1Control1 = new Pose(63.8, 114.7);
        submersible1Pose = new Pose(60.7, 96.1, Math.toRadians(-90));

        sample5BucketControl1 = new Pose(63.8, 114.7);
        sample5BucketPose = new Pose(17.7, 133.8, Math.toRadians(-20));

        parkControlPose = new Pose(60, 170);
        parkPose = new Pose(114.7, 150, Math.toRadians(180));

        driveSubsystem.setFollower(startPose);


        scoreSample1Path = new Path(new BezierLine(
                new Point(startPose),
                new Point(sample1BucketPose)
        ));
        scoreSample1Path.setLinearHeadingInterpolation(startPose.getHeading(), sample1BucketPose.getHeading());


        getSample2Path = new Path(new BezierLine(
                new Point(sample1BucketPose),
                new Point(sample2Pose)
        ));
        getSample2Path.setLinearHeadingInterpolation(sample1BucketPose.getHeading(), sample2Pose.getHeading());

        scoreSample2Path = new Path(new BezierLine(
                new Point(sample2Pose),
                new Point(sample2BucketPose)
        ));
        scoreSample2Path.setLinearHeadingInterpolation(sample2Pose.getHeading(), sample2BucketPose.getHeading());


        getSample3Path = new Path(new BezierLine(
                new Point(sample2BucketPose),
                new Point(sample3Pose)
        ));
        getSample3Path.setLinearHeadingInterpolation(sample2BucketPose.getHeading(), sample3Pose.getHeading());

        scoreSample3Path = new Path(new BezierLine(
                new Point(sample3Pose),
                new Point(sample3BucketPose)
        ));
        scoreSample3Path.setLinearHeadingInterpolation(sample3Pose.getHeading(), sample3BucketPose.getHeading());


        getSample4Path = new Path(new BezierLine(
                new Point(sample3BucketPose),
                new Point(sample4Pose)
        ));
        getSample4Path.setLinearHeadingInterpolation(sample3BucketPose.getHeading(), sample4Pose.getHeading());

        scoreSample4Path = new Path(new BezierLine(
                new Point(sample4Pose),
                new Point(sample4BucketPose)
        ));
        scoreSample4Path.setLinearHeadingInterpolation(sample4Pose.getHeading(), sample4BucketPose.getHeading());

        submersible1Path = new Path(new BezierCurve(
                sample4BucketPose,
                submersible1Control1,
                submersible1Pose
        ));
        submersible1Path.setLinearHeadingInterpolation(sample4BucketPose.getHeading(), submersible1Pose.getHeading());

        scoreSample5Path = new Path(new BezierCurve(
                submersible1Pose,
                sample5BucketControl1,
                sample5BucketPose
        ));
        scoreSample5Path.setLinearHeadingInterpolation(submersible1Pose.getHeading(), sample5BucketPose.getHeading());

        parkPath = new Path(new BezierCurve(
                new Point(sample5BucketPose),
                new Point(parkControlPose),
                new Point(parkPose)
        ));
        parkPath.setLinearHeadingInterpolation(sample5BucketPose.getHeading(), parkPose.getHeading());

        schedule(new SequentialCommandGroup(
                new ReadyHighSample(),
                new WaitCommand(500),
                new FollowPath(scoreSample1Path, true),
                new OpenClaw(),
                new WaitCommand(50),

                grab(getSample2Path),

                score(scoreSample2Path),

                grab(getSample3Path),

                score(scoreSample3Path),

                grab(getSample4Path),

                score(scoreSample4Path),

                new ReadyOuttake(),
                new DoorClose(),
                new BucketHover(),
                new FollowPath(submersible1Path, true),
                new GetSample(2),
                new IntakeRetract(),
                new WaitCommand(600),
                new GrabSample(),
                new WaitCommand(120),

                new ReadyHighSample(),
                new FollowPath(scoreSample5Path, true),
                new OpenClaw(),
                new WaitCommand(150),
                new FollowPath(parkPath, true)
        ));

        start();
    }

    private SequentialCommandGroup grab(Path path) {
        return new SequentialCommandGroup(
                new ReadyOuttake(),
                new BucketHover(),
                new FollowPath(path, true),
                new GetSample(2),
                new IntakeRetract()
        );
    }

    private SequentialCommandGroup score(Path path) {
        return new SequentialCommandGroup(
                new WaitCommand(600),
                new GrabSample(),
                new DoorClose(),
                new WaitCommand(120),
                new ReadyHighSample(),
                new WaitCommand(550),
                new FollowPath(path, true),
                new OpenClaw(),
                new WaitCommand(150)
        );
    }

    @Override
    public void loop() {
//        switch (sampleEnum) {
//            case scoreSample1:
//                scoreSample(scoreSample1Path, SampleEnum.getSample2);
//                break;
//
//
//            case getSample2:
//                getSample(getSample2Path, SampleEnum.scoreSample2);
//                break;
//
//
//            case scoreSample2:
//                scoreSample(scoreSample2Path, SampleEnum.getSample3);
//                break;
//
//
//            case getSample3:
//                getSample(getSample3Path, SampleEnum.scoreSample3);
//                break;
//
//
//            case scoreSample3:
//                scoreSample(scoreSample3Path, SampleEnum.getSample4);
//                break;
////
//            case getSample4:
//                getSample(getSample4Path, SampleEnum.scoreSample4);
//                break;
//
//            case scoreSample4:
//                scoreSample(scoreSample4Path, SampleEnum.park);
//                break;
//
//            case park:
//                caseThingie(
//                        () ->driveSubsystem.followPath(parkPath, true),
//                        () -> driveSubsystem.atParametricEnd() || elapsedTime.seconds() > 2.5,
//                        () -> {sampleEnum = SampleEnum.done;
//                        schedule(new SetOuttakeTarget(450));
//                        schedule(new ClawSample());}
//                );
//                break;
//
//            case done:
//                break;
//        }

//        RobotHardware.getInstance().updateLeds(Names.intakeColor);
    }

//    private void getSample(Path path, SampleEnum next) {
//        switch (grabSamp) {
//            case start:
//                caseThingie(
//                        () -> {driveSubsystem.followPath(path, true);
//                            schedule(new IntakeDown());},
//                        () -> driveSubsystem.atParametricEnd(),
//                        () -> grabSamp = GrabSamp.forward
//                );
//                break;
//
//            case forward:
//                caseThingie(
//                        () -> schedule(new SetIntakeTarget(350)),
//                        () -> elapsedTime.seconds() > 0.7 || RobotHardware.getInstance().isYellow(Names.intakeColor),
//                        () -> grabSamp = GrabSamp.retract
//                );
//                break;
//
//            case retract:
//                caseThingie(
//                        () -> schedule(new IntakeRetractAndGrab()),
//                        () -> elapsedTime.seconds() > 1.9,
//                        () -> {grabSamp = GrabSamp.start;
//                            sampleEnum = next;}
//                );
//                break;
//        }
//    }
//
//    private void scoreSample(Path path, SampleEnum next) {
//        switch (scoreSamp) {
//            case start:
//                caseThingie(
//                        () -> schedule(new ReadyHighSample()),
//                        () -> outtakeSubsystem.getPos() > 600,
//                        () -> scoreSamp = ScoreSamp.move
//                );
////                if (elapsedTime.seconds() > 0.25 && RobotHardware.getInstance().isYellow(Names.transferColor)) {
////                    scoreSamp = ScoreSamp.recorrect;
////                }
//                break;
//
//            case move:
//                caseThingie(
//                        () -> driveSubsystem.followPath(path, true),
//                        () -> driveSubsystem.atParametricEnd(),
//                        () -> scoreSamp = ScoreSamp.place
//                );
//                break;
//
//            case place:
//                caseThingie(
//                        () -> {schedule(new PlaceSample());
//                            scoreSamp = ScoreSamp.start;
//                            sampleEnum = next;},
//                        () -> true,
//                        () -> {}
//                );
//                break;
//        }
//    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
