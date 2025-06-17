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
import org.firstinspires.ftc.teamcode.commands.complex.sample.AHHHHHHHHHHH;
import org.firstinspires.ftc.teamcode.commands.complex.sample.GrabSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetract;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyHighSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyLowSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyOuttake;
import org.firstinspires.ftc.teamcode.commands.complex.sample.TransferDistance;
import org.firstinspires.ftc.teamcode.commands.drive.FollowPath;
import org.firstinspires.ftc.teamcode.commands.intake.BucketHover;
import org.firstinspires.ftc.teamcode.commands.intake.DoorClose;
import org.firstinspires.ftc.teamcode.commands.complex.sample.GetSample;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawSample;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeLowSample;
import org.firstinspires.ftc.teamcode.commands.outtake.SetOuttakeTarget;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.utils.Names;

import java.util.function.BooleanSupplier;

public class Sample extends BaseAuto {
    private Pose
            startPose, sample1BucketPose,
            sample2Pose, sample2BucketPose,
            sample3Pose, sample3BucketPose,
            sample4Pose, sample4BucketPose,
            submersible1Control1, submersible1Pose,
            sample5BucketControl1, sample5BucketPose,
            submersible2Control1, submersible2Pose,
            sample6BucketControl1, sample6BucketPose,
            submersible3Control1, submersible3Pose,
            sample7BucketControl1, sample7BucketPose,
            parkControlPose, parkPose;

    private Path
            scoreSample1Path,
            getSample2Path, scoreSample2Path,
            getSample3Path, scoreSample3Path,
            getSample4Path, scoreSample4Path,
            submersible1Path, scoreSample5Path,
            submersible2Path, scoreSample6Path,
            submersible3Path, scoreSample7Path,
            parkPath;

    protected Sample(Color _color) {
        super(_color);

        startPose = new Pose(7, 109, Math.toRadians(-90));
        sample1BucketPose = new Pose(16.7, 131, Math.toRadians(-20));

        sample2Pose = new Pose(26, 128.2, Math.toRadians(-19));
        sample2BucketPose = new Pose(17.2, 132.8, Math.toRadians(-30));

        sample3Pose = new Pose(24, 131.5, Math.toRadians(-4));
        sample3BucketPose = new Pose(18, 133.8, Math.toRadians(-25));

        sample4Pose = new Pose(25.7, 132.7, Math.toRadians(20));
        sample4BucketPose = new Pose(17.7, 133.8, Math.toRadians(-20));

        submersible1Control1 = new Pose(63.8, 114.7);
        submersible1Pose = new Pose(60.7, 97, Math.toRadians(-90));

        sample5BucketControl1 = new Pose(63.8, 114.7);
        sample5BucketPose = new Pose(17.7, 133.8, Math.toRadians(-20));

        submersible2Control1 = new Pose(63.8, 114.7);
        submersible2Pose = new Pose(66, 97, Math.toRadians(-90));

        sample6BucketControl1 = new Pose(63.8, 114.7);
        sample6BucketPose = new Pose(17.7, 133.8, Math.toRadians(-20));

        submersible3Control1 = new Pose(63.8, 114.7);
        submersible3Pose = new Pose(72, 97, Math.toRadians(-90));

        sample7BucketControl1 = new Pose(63.8, 114.7);
        sample7BucketPose = new Pose(17.7, 133.8, Math.toRadians(-20));

        parkControlPose = new Pose(80, 130);
        parkPose = new Pose(60.7, 95, Math.toRadians(90));

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
        scoreSample2Path.setZeroPowerAccelerationMultiplier(3.7);


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

        submersible2Path = new Path(new BezierCurve(
                sample5BucketPose,
                submersible2Control1,
                submersible2Pose
        ));
        submersible2Path.setLinearHeadingInterpolation(sample4BucketPose.getHeading(), submersible2Pose.getHeading());

        scoreSample6Path = new Path(new BezierCurve(
                submersible2Pose,
                sample6BucketControl1,
                sample6BucketPose
        ));
        scoreSample6Path.setLinearHeadingInterpolation(submersible2Pose.getHeading(), sample6BucketPose.getHeading());

        submersible3Path = new Path(new BezierCurve(
                sample6BucketPose,
                submersible3Control1,
                submersible3Pose
        ));
        submersible3Path.setLinearHeadingInterpolation(sample6BucketPose.getHeading(), submersible3Pose.getHeading());

        scoreSample7Path = new Path(new BezierCurve(
                submersible3Pose,
                sample7BucketControl1,
                sample7BucketPose
        ));
        scoreSample7Path.setLinearHeadingInterpolation(submersible3Pose.getHeading(), sample7BucketPose.getHeading());

        parkPath = new Path(new BezierCurve(
                new Point(sample5BucketPose),
                new Point(parkControlPose),
                new Point(parkPose)
        ));
        parkPath.setLinearHeadingInterpolation(sample7BucketPose.getHeading(), parkPose.getHeading());

        schedule(new SequentialCommandGroup(
                new ReadyHighSample(),
                new WaitCommand(150),
                new FollowPath(scoreSample1Path, true, () -> DriveSubsystem.getInstance().getYPos() > 129.5),
                new OpenClaw(),

                grab(getSample2Path, () -> DriveSubsystem.getInstance().getXPos() > 25),

                score(scoreSample2Path, 200, () -> DriveSubsystem.getInstance().getXPos() < 17.5),

                grab(getSample3Path, () -> DriveSubsystem.getInstance().getXPos() > 23),

                score(scoreSample3Path, 200, () -> DriveSubsystem.getInstance().getXPos() < 18),

                grab(getSample4Path, () -> DriveSubsystem.getInstance().getXPos() > 24),

                score(scoreSample4Path, 200, () -> DriveSubsystem.getInstance().getXPos() < 18.5),

                new ReadyOuttake(),
                new DoorClose(),
                new BucketHover(),
                new FollowPath(submersible1Path, true),
                new GetSample(2, color),
                new FollowPath(scoreSample5Path, true, () -> DriveSubsystem.getInstance().getXPos() < 18.5)
                        .alongWith(new IntakeRetract()
                                .andThen(new IntakeRetract())
                                .andThen(new TransferDistance())
                                .andThen(new GrabSample())
                                .andThen(new WaitCommand(120))
                                .andThen(new ReadyHighSample())
                        ),

                new OpenClaw(),
                new WaitCommand(150),
                new SetOuttakeTarget(200),
                new ClawSample(),

                new ReadyOuttake(),
                new DoorClose(),
                new BucketHover(),
                new FollowPath(submersible2Path, true),
                new GetSample(2, color),
                new FollowPath(scoreSample6Path, true, () -> DriveSubsystem.getInstance().getXPos() < 18)
                        .alongWith(new IntakeRetract()
                                .andThen(new IntakeRetract())
                                .andThen(new TransferDistance())
                                .andThen(new GrabSample())
                                .andThen(new WaitCommand(120))
                                .andThen(new ReadyHighSample())
                        ),

                new OpenClaw(),
                new WaitCommand(150),
                new SetOuttakeTarget(200),
                new ClawSample(),

                new ReadyOuttake(),
                new DoorClose(),
                new BucketHover(),
                new FollowPath(submersible3Path, true),
                new GetSample(2, color),
                new FollowPath(scoreSample7Path, true, () -> DriveSubsystem.getInstance().getXPos() < 18)
                        .alongWith(new IntakeRetract()
                                .andThen(new IntakeRetract())
                                .andThen(new TransferDistance())
                                .andThen(new GrabSample())
                                .andThen(new WaitCommand(120))
                                .andThen(new ReadyHighSample())
                        ),

                new OpenClaw(),
                new WaitCommand(150),
                new SetOuttakeTarget(200),
                new ClawSample(),

                new FollowPath(parkPath, true)
        ));

        start();
    }

    private SequentialCommandGroup grab(Path path) {
        return new SequentialCommandGroup(
                new BucketHover(),
                new FollowPath(path, true),
                new ReadyOuttake(),
                new GetSample(1.25, color),
                new IntakeRetract()
        );
    }

    private SequentialCommandGroup grab(Path path, BooleanSupplier override) {
        return new SequentialCommandGroup(
                new BucketHover(),
                new FollowPath(path, true, override),
                new ReadyOuttake(),
                new GetSample(1.25, color),
                new IntakeRetract()
        );
    }

    private SequentialCommandGroup score(Path path) {
        return new SequentialCommandGroup(
                new TransferDistance(),
                new GrabSample(),
                new DoorClose(),
                new WaitCommand(120),
                new ReadyHighSample(),
                new WaitCommand(700),
                new FollowPath(path, true),
                new OpenClaw(),
                new WaitCommand(150)
        );
    }

    private SequentialCommandGroup score(Path path, int timing, BooleanSupplier override) {
        return new SequentialCommandGroup(
                new TransferDistance(),
                new GrabSample(),
                new DoorClose(),
                new WaitCommand(120),
                new ReadyHighSample(),
                new WaitCommand(timing),
                new FollowPath(path, true, override),
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
        telemetry.addData("ditance", robotHardware.getDistance(Names.transferColor));
    }
}
