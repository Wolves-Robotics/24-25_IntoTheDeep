package org.firstinspires.ftc.teamcode.auto.classes;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.collections.Color;
import org.firstinspires.ftc.teamcode.auto.collections.sample.GrabSamp;
import org.firstinspires.ftc.teamcode.auto.collections.sample.SampleEnum;
import org.firstinspires.ftc.teamcode.auto.collections.sample.ScoreSamp;
import org.firstinspires.ftc.teamcode.commands.complex.sample.GrabSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetract;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetractAndGrab;
import org.firstinspires.ftc.teamcode.commands.complex.sample.PlaceSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyHighSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyOuttake;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawSample;
import org.firstinspires.ftc.teamcode.commands.outtake.SetOuttakeTarget;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class Sample extends BaseAuto {
    private SampleEnum sampleEnum;
    private ScoreSamp scoreSamp;
    private GrabSamp grabSamp;

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

    public Sample(Color color) {
        super(color);

        sampleEnum = SampleEnum.scoreSample1;
        scoreSamp = ScoreSamp.start;
        grabSamp = GrabSamp.start;


        startPose = new Pose(100, 100, 0);
        initBucketPose = new Pose(83.25, 106, Math.toRadians(45));

        sample1Pose = new Pose(79.25, 119, Math.toRadians(60));
        sample1BucketPose = new Pose(80, 109.5, Math.toRadians(45));

        sample2Pose = new Pose(86, 118, Math.toRadians(107));
        sample2BucketPose = new Pose(82, 109, Math.toRadians(45));

        sample3Pose = new Pose(77, 117.5, Math.toRadians(102));
        sample3BucketPose = new Pose(82, 104, Math.toRadians(32));

        parkControlPose = new Pose(60, 170);
        parkPose = new Pose(114.7, 150, Math.toRadians(180));


        driveSubsystem.setFollower(robotHardware.getHardwareMap(), startPose);


        scoreSample1Path = new Path(new BezierLine(
                new Point(startPose),
                new Point(initBucketPose)
        ));
        scoreSample1Path.setLinearHeadingInterpolation(startPose.getHeading(), initBucketPose.getHeading());


        getSample2Path = new Path(new BezierLine(
                new Point(initBucketPose),
                new Point(sample1Pose)
        ));
        getSample2Path.setLinearHeadingInterpolation(initBucketPose.getHeading(), sample1Pose.getHeading());

        scoreSample2Path = new Path(new BezierLine(
                new Point(sample1Pose),
                new Point(sample1BucketPose)
        ));
        scoreSample2Path.setLinearHeadingInterpolation(sample1Pose.getHeading(), sample1BucketPose.getHeading());


        getSample3Path = new Path(new BezierLine(
                new Point(sample1BucketPose),
                new Point(sample2Pose)
        ));
        getSample3Path.setLinearHeadingInterpolation(sample1BucketPose.getHeading(), sample2Pose.getHeading());

        scoreSample3Path = new Path(new BezierLine(
                new Point(sample2Pose),
                new Point(sample2BucketPose)
        ));
        scoreSample3Path.setLinearHeadingInterpolation(sample2Pose.getHeading(), sample2BucketPose.getHeading());


        getSample4Path = new Path(new BezierLine(
                new Point(sample2BucketPose),
                new Point(sample3Pose)
        ));
        getSample4Path.setLinearHeadingInterpolation(sample2BucketPose.getHeading(), sample3Pose.getHeading());

        scoreSample4Path = new Path(new BezierLine(
                new Point(sample3Pose),
                new Point(sample3BucketPose)
        ));
        scoreSample4Path.setLinearHeadingInterpolation(sample3Pose.getHeading(), sample3BucketPose.getHeading());


        parkPath = new Path(new BezierCurve(
                new Point(sample3BucketPose),
                new Point(parkControlPose),
                new Point(parkPose)
        ));
        parkPath.setLinearHeadingInterpolation(sample3BucketPose.getHeading(), parkPose.getHeading());

        start();
    }

    @Override
    public void loop() {
        switch (sampleEnum) {
            case scoreSample1:
                scoreSample(scoreSample1Path, SampleEnum.getSample2);
                break;


            case getSample2:
                getSample(getSample2Path, SampleEnum.scoreSample2);
                break;


            case scoreSample2:
                scoreSample(scoreSample2Path, SampleEnum.getSample3);
                break;


            case getSample3:
                getSample(getSample3Path, SampleEnum.scoreSample3);
                break;


            case scoreSample3:
                scoreSample(scoreSample3Path, SampleEnum.getSample4);
                break;
//
            case getSample4:
                getSample(getSample4Path, SampleEnum.scoreSample4);
                break;

            case scoreSample4:
                scoreSample(scoreSample4Path, SampleEnum.park);
                break;

            case park:
                caseThingie(
                        () ->driveSubsystem.followPath(parkPath, true),
                        () -> driveSubsystem.atParametricEnd(),
                        () -> {sampleEnum = SampleEnum.done;
                        schedule(new SetOuttakeTarget(450));
                        schedule(new ClawSample());}
                );
                break;

            case done:
                break;
        }

//        RobotHardware.getInstance().updateLeds(Names.intakeColor);
    }

    private void getSample(Path path, SampleEnum next) {
        switch (grabSamp) {
            case start:
                caseThingie(
                        () -> {driveSubsystem.followPath(path, true);
                            schedule(new IntakeDown());},
                        () -> driveSubsystem.atParametricEnd(),
                        () -> grabSamp = GrabSamp.forward
                );
                break;

            case forward:
                caseThingie(
                        () -> schedule(new SetIntakeTarget(350)),
                        () -> elapsedTime.seconds() > 0.7 || RobotHardware.getInstance().isYellow(Names.intakeColor),
                        () -> grabSamp = GrabSamp.retract
                );
                break;

            case retract:
                caseThingie(
                        () -> schedule(new IntakeRetractAndGrab()),
                        () -> elapsedTime.seconds() > 1.9,
                        () -> {grabSamp = GrabSamp.start;
                            sampleEnum = next;}
                );
                break;
        }
    }

    private void scoreSample(Path path, SampleEnum next) {
        switch (scoreSamp) {
            case start:
                caseThingie(
                        () -> schedule(new ReadyHighSample()),
                        () -> outtakeSubsystem.getPos() > 600,
                        () -> scoreSamp = ScoreSamp.move
                );
//                if (elapsedTime.seconds() > 0.25 && RobotHardware.getInstance().isYellow(Names.transferColor)) {
//                    scoreSamp = ScoreSamp.recorrect;
//                }
                break;

            case move:
                caseThingie(
                        () -> driveSubsystem.followPath(path, true),
                        () -> driveSubsystem.atParametricEnd(),
                        () -> scoreSamp = ScoreSamp.place
                );
                break;

            case place:
                caseThingie(
                        () -> {schedule(new PlaceSample());
                            scoreSamp = ScoreSamp.start;
                            sampleEnum = next;},
                        () -> true,
                        () -> {}
                );
                break;
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
