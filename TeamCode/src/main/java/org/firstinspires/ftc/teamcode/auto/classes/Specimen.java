package org.firstinspires.ftc.teamcode.auto.classes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.collections.Color;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.JailSpec;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.ReadySpec;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.ScoreSpec;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.SpecEnum;
import org.firstinspires.ftc.teamcode.auto.pedro.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.PlaceSpec;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenGrab;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenPlace;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.utils.Names;

public class Specimen extends BaseAuto {
    private SpecEnum specEnum;
    private JailSpec jailSpec;
    private ReadySpec readySpec;
    private ScoreSpec scoreSpec;

    private Pose
            startPose, score1Pose,
            sample2Control1Pose, sample2Pose, jailSample2Pose,
            sample3Pose, jailSample3Pose,
            sample4Pose, jailSample4Pose;

    private PathChain
            scoreSpec1Path,
            getSample2Path, jailSample2Path,
            getSample3Path, jailSample3Path,
            getSample4Path, jailSample4Path;

    public Specimen(Color color) {
        super(color);

        specEnum = SpecEnum.score1;
        scoreSpec = ScoreSpec.move;
        jailSpec = JailSpec.move;
        readySpec = ReadySpec.move;

        startPose = new Pose(135, 80, 0);
        score1Pose = new Pose(108, 80, 0);

        sample2Control1Pose = new Pose(135, 80);
        sample2Pose = new Pose(106, 102, Math.toRadians(135));
        jailSample2Pose = new Pose(120, 102, Math.toRadians(60));

        sample3Pose = new Pose(107, 114, Math.toRadians(150));
        jailSample3Pose = new Pose(120, 113, Math.toRadians(60));

        sample4Pose = new Pose(107, 120, Math.toRadians(150));
        jailSample4Pose = new Pose(120, 105, Math.toRadians(60));


        driveSubsystem.setFollower(robotHardware.getHardwareMap(), startPose);


        scoreSpec1Path = new PathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(score1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), score1Pose.getHeading())
                .build();

        getSample2Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(score1Pose), new Point(sample2Control1Pose), new Point(sample2Pose)))
                .setLinearHeadingInterpolation(score1Pose.getHeading(), sample2Pose.getHeading())
                .build();

        jailSample2Path = new PathBuilder()
                .addPath(new BezierLine(new Point(sample2Pose), new Point(jailSample2Pose)))
                .setLinearHeadingInterpolation(sample2Pose.getHeading(), jailSample2Pose.getHeading())
                .build();

        getSample3Path = new PathBuilder()
                .addPath(new BezierLine(new Point(jailSample2Pose), new Point(sample3Pose)))
                .setLinearHeadingInterpolation(jailSample2Pose.getHeading(), sample3Pose.getHeading())
                .build();

        jailSample3Path = new PathBuilder()
                .addPath(new BezierLine(new Point(sample3Pose), new Point(jailSample3Pose)))
                .setLinearHeadingInterpolation(sample3Pose.getHeading(), jailSample3Pose.getHeading())
                .build();
        
        getSample4Path = new PathBuilder()
                .addPath(new BezierLine(new Point(jailSample3Pose), new Point(sample4Pose)))
                .setLinearHeadingInterpolation(jailSample3Pose.getHeading(), sample4Pose.getHeading())
                .build();

        jailSample4Path = new PathBuilder()
                .addPath(new BezierLine(new Point(sample4Pose), new Point(jailSample4Pose)))
                .setLinearHeadingInterpolation(sample4Pose.getHeading(), jailSample4Pose.getHeading())
                .build();

    }

    @Override
    public void loop() {
        switch (specEnum) {
            case score1:
                score(scoreSpec1Path, SpecEnum.jail2);
                break;

            case jail2:
                switch (jailSpec) {
                    case move:
                        caseThingie(
                                () -> {driveSubsystem.followPath(getSample2Path, true);
                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.75,
                                () -> {schedule(new SetIntakeTarget(350));
                                    jailSpec = JailSpec.extend;}
                        );
                        break;
                    case extend:
                        caseThingie(
                                () -> {},
                                () -> driveSubsystem.atParametricEnd(),
                                () -> {
                                    jailSpec = JailSpec.move;
                                    specEnum = SpecEnum.jail3;
                                }
                        );
                        break;
                }
                break;
        }
    }

    protected void score(PathChain path, SpecEnum next) {
        switch (scoreSpec) {
            case move:
                caseThingie(
                        () -> {
                            driveSubsystem.followPath(path, true);
                            schedule(new ReadySpecimenPlace());
                        },
                        () -> driveSubsystem.atParametricEnd(),
                        () -> scoreSpec = ScoreSpec.down
                );
                break;
            case down:
                caseThingie(
                        () -> schedule(new PlaceSpec()),
                        () -> elapsedTime.seconds() > 0.5,
                        () -> {
                            scoreSpec = ScoreSpec.move;
                            specEnum = next;
                        }
                );
                break;
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
