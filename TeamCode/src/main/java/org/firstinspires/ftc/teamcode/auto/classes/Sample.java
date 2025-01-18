package org.firstinspires.ftc.teamcode.auto.classes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.collections.Color;
import org.firstinspires.ftc.teamcode.auto.collections.GrabSamp;
import org.firstinspires.ftc.teamcode.auto.collections.SampleEnum;
import org.firstinspires.ftc.teamcode.auto.collections.ScoreSamp;
import org.firstinspires.ftc.teamcode.auto.pedro.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.commands.complex.FullTransfer;
import org.firstinspires.ftc.teamcode.commands.complex.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.complex.IntakeRetract;
import org.firstinspires.ftc.teamcode.commands.complex.PlaceSample;
import org.firstinspires.ftc.teamcode.commands.complex.ReadyHighSample;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.Names;

public class Sample extends BaseAuto {
    private SampleEnum sampleEnum;
    private ScoreSamp scoreSamp;
    private GrabSamp grabSamp;

    private Pose
            start,  initBucket,
            sample1, sample1Bucket,
            sample2, sample2Bucket,
            sample3, sample3Bucket,
            parkControlPoint, park;

    private Path
            initSamplePath,
            toSample1Path, sample1ToBucketPath,
            toSample2Path, sample2ToBucketPath,
            toSample3Path, sample3ToBucketPath,
            sampleParkPath;

    public Sample(Color color) {
        super(color);
        robotHardware.servoInit();
        driveSubsystem.setFollower(robotHardware.getHardwareMap(), new Pose());

        sampleEnum = SampleEnum.scoreInitSample;
        scoreSamp = ScoreSamp.start;
        grabSamp = GrabSamp.start;


        start = new Pose(100, 100, 0);
        initBucket = new Pose(85, 103, Math.toRadians(32));

        sample1 = new Pose(81, 120, Math.toRadians(60));
        sample1Bucket = new Pose(82, 104, Math.toRadians(32));

        sample2 = new Pose(85, 118, Math.toRadians(112));
        sample2Bucket = new Pose(82, 104, Math.toRadians(32));

        sample3 = new Pose(76, 117.5, Math.toRadians(109));
        sample3Bucket = new Pose(82, 104, Math.toRadians(32));

        parkControlPoint = new Pose(60, 170);
        park = new Pose(118, 150, Math.toRadians(180));


        initSamplePath = new Path(new BezierLine(
                new Point(start),
                new Point(initBucket)
        ));
        initSamplePath.setLinearHeadingInterpolation(start.getHeading(), initBucket.getHeading());


        toSample1Path = new Path(new BezierLine(
                new Point(initBucket),
                new Point(sample1)
        ));
        toSample1Path.setLinearHeadingInterpolation(initBucket.getHeading(), sample1.getHeading());

        sample1ToBucketPath = new Path(new BezierLine(
                new Point(sample1),
                new Point(sample1Bucket)
        ));
        sample1ToBucketPath.setLinearHeadingInterpolation(sample1.getHeading(), sample1Bucket.getHeading());


        toSample2Path = new Path(new BezierLine(
                new Point(sample1Bucket),
                new Point(sample2)
        ));
        toSample2Path.setLinearHeadingInterpolation(sample1Bucket.getHeading(), sample2.getHeading());

        sample2ToBucketPath = new Path(new BezierLine(
                new Point(sample2),
                new Point(sample2Bucket)
        ));
        sample2ToBucketPath.setLinearHeadingInterpolation(sample2.getHeading(), sample2Bucket.getHeading());


        toSample3Path = new Path(new BezierLine(
                new Point(sample2Bucket),
                new Point(sample3)
        ));
        toSample3Path.setLinearHeadingInterpolation(sample2Bucket.getHeading(), sample3.getHeading());

        sample3ToBucketPath = new Path(new BezierLine(
                new Point(sample3),
                new Point(sample3Bucket)
        ));
        sample3ToBucketPath.setLinearHeadingInterpolation(sample3.getHeading(), sample3Bucket.getHeading());


        sampleParkPath = new Path(new BezierCurve(
                new Point(sample3Bucket),
                new Point(parkControlPoint),
                new Point(park)
        ));
        sampleParkPath.setLinearHeadingInterpolation(sample3Bucket.getHeading(), park.getHeading());
    }

    @Override
    public void loop() {
        switch (sampleEnum) {
            case scoreInitSample:
                switch (scoreSamp) {
                    case start:
                        caseThingie(
                                () -> schedule(new ReadyHighSample()),
                                () -> outtakeSubsystem.getPos() > 600,
                                () -> scoreSamp = ScoreSamp.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> driveSubsystem.followPath(initSamplePath, true),
                                () -> driveSubsystem.atParametricEnd(),
                                () -> scoreSamp = ScoreSamp.place
                        );
                        break;
                    case place:
                        caseThingie(
                                () -> {schedule(new PlaceSample());
                                    scoreSamp = ScoreSamp.start;
                                    sampleEnum = SampleEnum.getSample1;},
                                () -> true,
                                () -> {}
                        );
                        break;
                }
                break;


            case getSample1:
                switch (grabSamp) {
                    case start:
                        caseThingie(
                                () -> {driveSubsystem.followPath(toSample1Path, true);
                                    schedule(new IntakeDown());},
                                () -> driveSubsystem.atParametricEnd(),
                                () -> grabSamp = GrabSamp.forward
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> schedule(new SetIntakeTarget(300)),
                                () -> elapsedTime.seconds() > 0.7 || robotHardware.isYellow(Names.intakeColor),
                                () -> grabSamp = GrabSamp.retract
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> schedule(new IntakeRetract()),
                                () -> elapsedTime.seconds() > 0.8,
                                () -> {sampleEnum = SampleEnum.scoreSample1;
                                    grabSamp = GrabSamp.start;}
                        );
                        break;
                }
                break;


            case scoreSample1:
                switch (scoreSamp) {
                    case start:
                        caseThingie(
                                () -> schedule(new ReadyHighSample()),
                                () -> outtakeSubsystem.getPos() > 600,
                                () -> scoreSamp = ScoreSamp.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> driveSubsystem.followPath(sample1ToBucketPath, true),
                                () -> driveSubsystem.atParametricEnd(),
                                () -> scoreSamp = ScoreSamp.place
                        );
                        break;
                    case place:
                        caseThingie(
                                () -> {schedule(new PlaceSample());
                                    scoreSamp = ScoreSamp.start;
                                    sampleEnum = SampleEnum.getSample2;},
                                () -> true,
                                () -> {}
                        );
                        break;
                }
                break;


            case getSample2:
                switch (grabSamp) {
                    case start:
                        caseThingie(
                                () -> {driveSubsystem.followPath(toSample2Path, true);
                                    schedule(new IntakeDown());},
                                () -> driveSubsystem.atParametricEnd(),
                                () -> grabSamp = GrabSamp.forward
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> schedule(new SetIntakeTarget(300)),
                                () -> elapsedTime.seconds() > 0.7 || robotHardware.isYellow(Names.intakeColor),
                                () -> grabSamp = GrabSamp.retract
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> schedule(new IntakeRetract()),
                                () -> OuttakeSubsystem.getInstance().getPos() > 600,
                                () -> {sampleEnum = SampleEnum.scoreSample2;
                                    grabSamp = GrabSamp.start;}
                        );
                        break;
                }
                break;


            case scoreSample2:
                switch (scoreSamp) {
                    case start:
                        caseThingie(
                                () -> schedule(new ReadyHighSample()),
                                () -> outtakeSubsystem.getPos() > 600,
                                () -> scoreSamp = ScoreSamp.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> driveSubsystem.followPath(sample2ToBucketPath, true),
                                () -> driveSubsystem.atParametricEnd(),
                                () -> scoreSamp = ScoreSamp.place
                        );
                        break;
                    case place:
                        caseThingie(
                                () -> {schedule(new PlaceSample());
                                    scoreSamp = ScoreSamp.start;
                                    sampleEnum = SampleEnum.getSample3;},
                                () -> true,
                                () -> {}
                        );
                        break;
                }
                break;
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
