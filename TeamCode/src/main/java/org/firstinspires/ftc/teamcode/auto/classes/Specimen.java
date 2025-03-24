package org.firstinspires.ftc.teamcode.auto.classes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.collections.Color;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenGrab;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenPlace;
import org.firstinspires.ftc.teamcode.commands.drive.FollowPath;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawNeutral;
import org.firstinspires.ftc.teamcode.commands.outtake.ClawSpecimenPlace;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeHighSpecimen;
import org.firstinspires.ftc.teamcode.commands.outtake.SetOuttakeTarget;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

public class Specimen extends BaseAuto{

    private Pose
            startPose, scoreSpec1Pose,
            farSamp1Control1, farSamp1Control2, farSamp1Control3, farSamp1Pose,
            jailSamp1Pose,
            farSamp2Control1, farSamp2Pose,
            jailSamp2Pose,
            farSamp3Control1, farSamp3Pose,
            jailSamp3ReadySpec2Pose,
            scoreSpec2Control1, scoreSpec2Control2, scoreSpec2Pose,
            readySpec3Control1, readySpec3Control2, readySpec3Pose,
            scoreSpec3Control1, scoreSpec3Control2, scoreSpec3Pose,
            readySpec4Control1, readySpec4Control2, readySpec4Pose,
            scoreSpec4Control1, scoreSpec4Control2, scoreSpec4Pose,
            readySpec5Control1, readySpec5Control2, readySpec5Pose,
            scoreSpec5Control1, scoreSpec5Control2, scoreSpec5Pose,
            parkControl1, parkPose;

    private PathChain
            scoreSpec1Path,
            farSamp1Path, jailSamp1Path,
            farSamp2Path, jailSamp2Path,
            farSamp3Path, jailSamp3ReadySpec2Path,
            scoreSpec2Path,
            readySpec3Path, scoreSpec3Path,
            readySpec4Path, scoreSpec4Path,
            readySpec5Path, scoreSpec5Path,
            parkPath;

    protected Specimen(Color _color) {
        super(_color);

        startPose = new Pose(8, 60, Math.toRadians(180));
        scoreSpec1Pose = new Pose(31.5, 59.5, Math.toRadians(180));

        farSamp1Control1 = new Pose(30.5, 18.8);
        farSamp1Control2 = new Pose(43, 40.5);
        farSamp1Control3 = new Pose(59.2, 36.7);
        farSamp1Pose = new Pose(58.5, 26, Math.toRadians(0));

        jailSamp1Pose = new Pose(16, 26, Math.toRadians(0));

        farSamp2Control1 = new Pose(67.5, 27);
        farSamp2Pose = new Pose(58.5, 16, Math.toRadians(0));

        jailSamp2Pose = new Pose(16, 16, Math.toRadians(0));

        farSamp3Control1 = new Pose(67.5, 17);
        farSamp3Pose = new Pose(58.5, 9.5, Math.toRadians(0));

        jailSamp3ReadySpec2Pose = new Pose(15, 9.5, Math.toRadians(0));

        scoreSpec2Control1 = new Pose(42, 28);
        scoreSpec2Control2 = new Pose(1, 67.7);
        scoreSpec2Pose = new Pose(30.5, 63, Math.toRadians(180));

        readySpec3Control1 = new Pose(8.7, 59.5);
        readySpec3Control2 = new Pose(70, 25.3);
        readySpec3Pose = new Pose(14.5, 30, Math.toRadians(0));

        scoreSpec3Control1 = new Pose(34.3, 37.6);
        scoreSpec3Control2 = new Pose(0.6, 68);
        scoreSpec3Pose = new Pose(30.5, 67, Math.toRadians(180));

        readySpec4Control1 = new Pose(8.7, 59.5);
        readySpec4Control2 = new Pose(70, 25.3);
        readySpec4Pose = new Pose(14.5, 30, Math.toRadians(0));

        scoreSpec4Control1 = new Pose(34.3, 37.6);
        scoreSpec4Control2 = new Pose(0.6, 68);
        scoreSpec4Pose = new Pose(30.5, 71, Math.toRadians(180));

//        readySpec5Control1 = new Pose(7.6, 59.5);
//        readySpec5Control2 = new Pose(70, 25.3);
//        readySpec5Pose = new Pose(14.5, 30, Math.toRadians(0));
//
//        scoreSpec5Control1 = new Pose(34.3, 37.6);
//        scoreSpec5Control2 = new Pose(0.6, 68);
//        scoreSpec5Pose = new Pose(30.5, 75);

        parkControl1 = new Pose(9.8, 69);
        parkPose = new Pose(12.5, 27.5, Math.toRadians(-90));

        driveSubsystem.setFollower(startPose);

        scoreSpec1Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(startPose), new Point(scoreSpec1Pose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), scoreSpec1Pose.getHeading())
                .build();

        farSamp1Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpec1Pose), new Point(farSamp1Control1), new Point(farSamp1Control2), new Point(farSamp1Control3), new Point(farSamp1Pose)))
                .setLinearHeadingInterpolation(scoreSpec1Pose.getHeading(), farSamp1Pose.getHeading())
                .build();

        jailSamp1Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(farSamp1Pose), new Point(jailSamp1Pose)))
                .setLinearHeadingInterpolation(farSamp1Pose.getHeading(), jailSamp1Pose.getHeading())
                .build();

        farSamp2Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(jailSamp1Pose), new Point(farSamp2Control1), new Point(farSamp2Pose)))
                .setLinearHeadingInterpolation(jailSamp1Pose.getHeading(), farSamp2Pose.getHeading())
                .build();

        jailSamp2Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(farSamp2Pose), new Point(jailSamp2Pose)))
                .setLinearHeadingInterpolation(farSamp2Pose.getHeading(), jailSamp2Pose.getHeading())
                .build();

        farSamp3Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(jailSamp2Pose), new Point(farSamp3Control1), new Point(farSamp3Pose)))
                .setLinearHeadingInterpolation(jailSamp2Pose.getHeading(), farSamp3Pose.getHeading())
                .build();

        jailSamp3ReadySpec2Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(farSamp3Pose), new Point(jailSamp3ReadySpec2Pose)))
                .setLinearHeadingInterpolation(farSamp3Pose.getHeading(), jailSamp3ReadySpec2Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        scoreSpec2Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(jailSamp3ReadySpec2Pose), new Point(scoreSpec2Control1), new Point(scoreSpec2Control2), new Point(scoreSpec2Pose)))
                .setLinearHeadingInterpolation(jailSamp3ReadySpec2Pose.getHeading(), scoreSpec2Pose.getHeading())
                .build();

        readySpec3Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpec2Pose), new Point(readySpec3Control1), new Point(readySpec3Control2), new Point(readySpec3Pose)))
                .setLinearHeadingInterpolation(scoreSpec2Pose.getHeading(), readySpec3Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        scoreSpec3Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(readySpec3Pose), new Point(scoreSpec3Control1), new Point(scoreSpec3Control2), new Point(scoreSpec3Pose)))
                .setLinearHeadingInterpolation(readySpec3Pose.getHeading(), scoreSpec3Pose.getHeading())
                .build();

        readySpec4Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(scoreSpec3Pose), new Point(readySpec4Control1), new Point(readySpec4Control2), new Point(readySpec4Pose)))
                .setLinearHeadingInterpolation(scoreSpec3Pose.getHeading(), readySpec4Pose.getHeading())
                .setZeroPowerAccelerationMultiplier(3.5)
                .build();

        scoreSpec4Path = new PathBuilder()
                .addPath(new BezierCurve(new Point(readySpec4Pose), new Point(scoreSpec4Control1), new Point(scoreSpec4Control2), new Point(scoreSpec4Pose)))
                .setLinearHeadingInterpolation(readySpec4Pose.getHeading(), scoreSpec4Pose.getHeading())
                .build();

//        readySpec5Path = new PathBuilder()
//                .addPath(new BezierCurve(new Point(scoreSpec4Pose), new Point(readySpec5Control1), new Point(readySpec5Control2), new Point(readySpec5Pose)))
//                .setLinearHeadingInterpolation(scoreSpec4Pose.getHeading(), readySpec5Pose.getHeading())
//                .build();
//
//        scoreSpec5Path = new PathBuilder()
//                .addPath(new BezierCurve(new Point(readySpec5Pose), new Point(scoreSpec5Control1), new Point(scoreSpec5Control2), new Point(scoreSpec5Pose)))
//                .setLinearHeadingInterpolation(readySpec5Pose.getHeading(), scoreSpec5Pose.getHeading())
//                .build();

        parkPath = new PathBuilder()
                .addPath(new BezierCurve(scoreSpec4Pose, parkControl1, parkPose))
                .setLinearHeadingInterpolation(scoreSpec4Pose.getHeading(), parkPose.getHeading())
                .build();

        schedule(new SequentialCommandGroup(
                new ReadySpecimenPlace(),
                new FollowPath(scoreSpec1Path, false),
                new SetOuttakeTarget(0),
                new WaitCommand(250),
                new OpenClaw(),
                new ClawNeutral(),

                new FollowPath(farSamp1Path, false, () -> DriveSubsystem.getInstance().getYPos() < 27),
                new FollowPath(jailSamp1Path, false, () -> DriveSubsystem.getInstance().getXPos() < 20),
                new FollowPath(farSamp2Path, false, () -> DriveSubsystem.getInstance().getYPos() < 17),
                new FollowPath(jailSamp2Path, false, () -> DriveSubsystem.getInstance().getXPos() < 20),
                new FollowPath(farSamp3Path, false, () -> DriveSubsystem.getInstance().getYPos() < 10.5),

                new ReadySpecimenGrab(),
                new FollowPath(jailSamp3ReadySpec2Path, true, () -> DriveSubsystem.getInstance().getXPos() < 15.4),
                new CloseClaw(),
                new WaitCommand(125),

                new OuttakeHighSpecimen(),
                new WaitCommand(50),
                new ClawSpecimenPlace(),
                new FollowPath(scoreSpec2Path, true),
                new SetOuttakeTarget(50),
                new WaitCommand(400),
                new OpenClaw(),

                new ReadySpecimenGrab(),
                new FollowPath(readySpec3Path, true, () -> DriveSubsystem.getInstance().getXPos() < 15.3),
                new CloseClaw(),
                new WaitCommand(125),

                new OuttakeHighSpecimen(),
                new WaitCommand(50),
                new ClawSpecimenPlace(),
                new FollowPath(scoreSpec3Path, true),
                new SetOuttakeTarget(0),
                new WaitCommand(400),
                new OpenClaw(),

                new ReadySpecimenGrab(),
                new FollowPath(readySpec4Path, true, () -> DriveSubsystem.getInstance().getXPos() < 15.3),
                new CloseClaw(),
                new WaitCommand(125),

                new OuttakeHighSpecimen(),
                new WaitCommand(50),
                new ClawSpecimenPlace(),
                new FollowPath(scoreSpec4Path, true),
                new SetOuttakeTarget(0),
                new WaitCommand(400),
                new OpenClaw(),

                new ReadySpecimenGrab(),
                new FollowPath(parkPath, true)
        ));

        start();
    }

    @Override
    public void loop() {

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
