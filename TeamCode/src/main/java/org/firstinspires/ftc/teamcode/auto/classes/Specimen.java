package org.firstinspires.ftc.teamcode.auto.classes;

import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.collections.Color;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.JailSpec;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.ReadySpec;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.ScoreSpec;
import org.firstinspires.ftc.teamcode.auto.collections.specimen.SpecEnum;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetract;
import org.firstinspires.ftc.teamcode.commands.complex.sample.SlurpExpel;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.PlaceSpec;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenGrab;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenPlace;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class Specimen extends BaseAuto {
    private SpecEnum specEnum;
    private JailSpec jailSpec;
    private ReadySpec readySpec;
    private ScoreSpec scoreSpec;

    private Pose
            startPose, score1Pose,
            sample2Control1Pose, sample2Pose, jailSample2Pose,
            sample3Pose, jailSample3Pose,
            sample4Pose, jailSample4Pose,
            readySpec2Control1Pose, readySpec2Pose,
            scoreSpec2Control1Pose, scoreSpec2Pose,
            readySpec3Control1Pose, readySpec3Control2Pose, readySpec3Pose,
            scoreSpec3Control1Pose, scoreSpec3Pose,
            readySpec4Control1Pose, readySpec4Control2Pose, readySpec4Pose,
            scoreSpec4Control1Pose, scoreSpec4Pose,
            readySpec5Control1Pose, readySpec5Control2Pose, readySpec5Pose,
            scoreSpec5Control1Pose, scoreSpec5Pose,
            parkControl1Pose, parkPose;

    private PathChain
            scoreSpec1Path,
            getSample2Path, jailSample2Path,
            getSample3Path, jailSample3Path,
            getSample4Path, jailSample4Path,
            readySpec2Path, scoreSpec2Path,
            readySpec3Path, scoreSpec3Path,
            readySpec4Path, scoreSpec4Path,
            readySpec5Path, scoreSpec5Path,
            parkPath;

    public Specimen(Color color) {
        super(color);

        specEnum = SpecEnum.score1;
        scoreSpec = ScoreSpec.move;
        jailSpec = JailSpec.move;
        readySpec = ReadySpec.move;

        startPose = new Pose(135, 80, 0);
        score1Pose = new Pose(110.5, 81, 0);

        sample2Control1Pose = new Pose(125, 80);
        sample2Pose = new Pose(106, 100, Math.toRadians(155));
        jailSample2Pose = new Pose(120, 102, Math.toRadians(60));

        sample3Pose = new Pose(107, 106, Math.toRadians(100));
        jailSample3Pose = new Pose(116, 107, Math.toRadians(80));

        sample4Pose = new Pose(107, 114, Math.toRadians(100));
        jailSample4Pose = new Pose(116, 99, Math.toRadians(80));

        readySpec2Control1Pose = new Pose(95, 110);
        readySpec2Pose = new Pose(129, 113, Math.toRadians(180));

        scoreSpec2Control1Pose = new Pose(135, 70);
        scoreSpec2Pose = new Pose(110, 78);

        readySpec3Control1Pose = new Pose(130, 93);
        readySpec3Control2Pose = new Pose(77, 115);
        readySpec3Pose = new Pose(129, 113, Math.toRadians(180));

        scoreSpec3Control1Pose = new Pose(135, 67);
        scoreSpec3Pose = new Pose(110, 75);

        readySpec4Control1Pose = new Pose(130, 93);
        readySpec4Control2Pose = new Pose(77, 115);
        readySpec4Pose = new Pose(129, 113, Math.toRadians(180));

        scoreSpec4Control1Pose = new Pose(135, 64);
        scoreSpec4Pose = new Pose(110, 72);

        readySpec5Control1Pose = new Pose(130, 93);
        readySpec5Control2Pose = new Pose(77, 115);
        readySpec5Pose = new Pose(129, 113, Math.toRadians(180));

        scoreSpec5Control1Pose = new Pose(135, 61);
        scoreSpec5Pose = new Pose(110, 69);

        parkControl1Pose = new Pose(107, 74);
        parkPose = new Pose(133, 121, Math.toRadians(0));


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

        readySpec2Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(jailSample4Pose), new Point(readySpec2Control1Pose), new Point(readySpec2Pose)))
                .setLinearHeadingInterpolation(jailSample4Pose.getHeading(), readySpec2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        scoreSpec2Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(readySpec2Pose), new Point(scoreSpec2Control1Pose), new Point(scoreSpec2Pose)))
                .setLinearHeadingInterpolation(readySpec2Pose.getHeading(), scoreSpec2Pose.getHeading())
                .build();

        readySpec3Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpec2Pose), new Point(readySpec3Control1Pose), new Point(readySpec3Control2Pose), new Point(readySpec3Pose)))
                .setLinearHeadingInterpolation(scoreSpec2Pose.getHeading(), readySpec3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        scoreSpec3Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(readySpec3Pose), new Point(scoreSpec3Control1Pose), new Point(scoreSpec3Pose)))
                .setLinearHeadingInterpolation(readySpec3Pose.getHeading(), scoreSpec3Pose.getHeading())
                .build();

        readySpec4Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpec3Pose), new Point(readySpec4Control1Pose), new Point(readySpec4Control2Pose), new Point(readySpec4Pose)))
                .setLinearHeadingInterpolation(scoreSpec3Pose.getHeading(), readySpec4Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        scoreSpec4Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(readySpec4Pose), new Point(scoreSpec4Control1Pose), new Point(scoreSpec4Pose)))
                .setLinearHeadingInterpolation(readySpec4Pose.getHeading(), scoreSpec4Pose.getHeading())
                .build();

        readySpec5Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpec4Pose), new Point(readySpec5Control1Pose), new Point(readySpec5Control2Pose), new Point(readySpec5Pose)))
                .setLinearHeadingInterpolation(scoreSpec4Pose.getHeading(), readySpec5Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(8)
                .build();

        scoreSpec5Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(readySpec5Pose), new Point(scoreSpec5Control1Pose), new Point(scoreSpec5Pose)))
                .setLinearHeadingInterpolation(readySpec5Pose.getHeading(), scoreSpec5Pose.getHeading())
                .build();

        parkPath = new PathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpec5Pose), new Point(parkControl1Pose), new Point(parkPose)))
                .setLinearHeadingInterpolation(scoreSpec5Pose.getHeading(), parkPose.getHeading())
                .build();

        start();
    }

    @Override
    public void loop() {
        switch (specEnum) {
            case score1:
                score(scoreSpec1Path, SpecEnum.jail2);
                break;

            case jail2:
                jail(getSample2Path, jailSample2Path, SpecEnum.jail3);
                break;

            case jail3:
                jail(getSample3Path, jailSample3Path, SpecEnum.ready2);
                break;

            case jail4:
                jail(getSample4Path, jailSample4Path, SpecEnum.ready2);
                break;

            case ready2:
                ready(readySpec2Path, SpecEnum.score2);
                break;

            case score2:
                ready(scoreSpec2Path, SpecEnum.ready3);
                break;

            case ready3:
                ready(readySpec3Path, SpecEnum.score3);
                break;

            case score3:
                ready(scoreSpec3Path, SpecEnum.ready4);
                break;

            case ready4:
                ready(readySpec4Path, SpecEnum.score4);
                break;

            case score4:
                ready(scoreSpec4Path, SpecEnum.park);
                break;

            case ready5:
                ready(readySpec5Path, SpecEnum.score5);
                break;

            case score5:
                ready(scoreSpec5Path, SpecEnum.park);
                break;

            case park:
                caseThingie(
                        () -> driveSubsystem.followPath(parkPath, true),
                        () -> driveSubsystem.atParametricEnd(),
                        () -> specEnum = SpecEnum.done
                );
                break;

            case done:
                break;
        }
    }

    protected void ready(PathChain path, SpecEnum next) {
        switch (readySpec) {
            case move:
                caseThingie(
                        () -> {
                            driveSubsystem.followPath(path, true);
                            schedule(new IntakeRetract());
                            schedule(new ReadySpecimenGrab());
                        },
                        () -> driveSubsystem.atParametricEnd() ,
                        () -> readySpec = ReadySpec.grab
                );
                break;
            case grab:
                caseThingie(
                        () -> schedule(new CloseClaw()),
                        () -> elapsedTime.seconds() > 0.4,
                        () -> {
                            readySpec = ReadySpec.move;
                            specEnum = next;
                        }
                );
                break;
        }
    }

    protected void jail(PathChain path1, PathChain path2, SpecEnum next) {
        switch (jailSpec) {
            case move:
                caseThingie(
                        () -> {driveSubsystem.followPath(path1, true);
                            schedule(new SetIntakeTarget(350));},
                        () -> elapsedTime.seconds() > 1,
                        () -> {robotHardware.setServoPos(Names.intakePivot, 0.61);
                            robotHardware.setServoPos(Names.intakeArm, 0.77);
                            jailSpec = JailSpec.extend;}
                );
                break;
            case extend:
                caseThingie(
                        () -> {},
                        () -> driveSubsystem.atParametricEnd(),
                        () -> jailSpec = JailSpec.jail
                );
                break;

            case jail:
                caseThingie(
                        () -> driveSubsystem.followPath(path2, true),
                        () -> driveSubsystem.atParametricEnd(),
                        () -> {schedule(new SetIntakeTarget(0));
                            jailSpec = JailSpec.move;
                            specEnum = next;}
                );
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
                        () -> {schedule(new PlaceSpec());
                            scoreSpec = ScoreSpec.move;
                            specEnum = next;}
                );
                break;
        }
    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Spec enum",
                specEnum == SpecEnum.score1 ? "score 1" :
                specEnum == SpecEnum.jail2  ? "jail 2"  :
                specEnum == SpecEnum.jail3  ? "jail 3"  :
                specEnum == SpecEnum.jail4  ? "jail 4"  :
                specEnum == SpecEnum.ready2 ? "ready 2" :
                specEnum == SpecEnum.score2 ? "score 2" :
                specEnum == SpecEnum.ready3 ? "ready 3" :
                specEnum == SpecEnum.score3 ? "score 3" :
                specEnum == SpecEnum.ready4 ? "ready 4" :
                specEnum == SpecEnum.score4 ? "score 4" :
                specEnum == SpecEnum.ready5 ? "ready 5" :
                specEnum == SpecEnum.score5 ? "score 5" :
                specEnum == SpecEnum.park ? "park" : "done");

        telemetry.addData("jail enum",
                jailSpec == JailSpec.move ? "move" :
                jailSpec == JailSpec.intake ? "intake" : "jail");
    }
}
