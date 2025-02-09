package org.firstinspires.ftc.teamcode.auto.classes;

import static org.firstinspires.ftc.teamcode.utils.Constants.id;
import static org.firstinspires.ftc.teamcode.utils.Constants.ii;
import static org.firstinspires.ftc.teamcode.utils.Constants.ip;
import static org.firstinspires.ftc.teamcode.utils.Constants.od;
import static org.firstinspires.ftc.teamcode.utils.Constants.of;
import static org.firstinspires.ftc.teamcode.utils.Constants.oi;
import static org.firstinspires.ftc.teamcode.utils.Constants.op;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.auto.sample.GrabSamp;
import org.firstinspires.ftc.teamcode.collections.auto.sample.SampleEnum;
import org.firstinspires.ftc.teamcode.collections.auto.sample.ScoreSamp;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

@Autonomous
@Disabled
public class DepSample extends OpMode {
    private Follower follower;
    private SampleEnum sampleEnum;
    private ScoreSamp scoreSamp;
    private GrabSamp grabSamp;
    private RobotHardware robotHardware;
    private ElapsedTime elapsedTime;
    private boolean start = true;
    private Path
            initSamplePath,
            toSample1Path, sample1ToBucketPath,
            toSample2Path, sample2ToBucketPath,
            toSample3Path, sample3ToBucketPath,
            sampleParkPath;

    private PIDController intakePID, outtakePID;
    private double iTarget=0, oTarget=0;
    private int iArmPos, oArmPos;

    @Override
    public void init() {
        robotHardware = RobotHardware.getInstance();
        robotHardware.setServoPos(Names.intakeArm, 0.1);
        robotHardware.setServoPos(Names.intakePivot, 0.19);
        robotHardware.setServoPos(Names.claw, 0.3);
        robotHardware.setServoPos(Names.outtakeArm, 0.23);
        robotHardware.setServoPos(Names.outtakePivot, 0.4);
        robotHardware.setServoPos(Names.door, 0.7);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(100, 100, 0));
        follower.update();
        elapsedTime = new ElapsedTime();

        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);

        sampleEnum = SampleEnum.scoreSample1;
        scoreSamp = ScoreSamp.start;
        grabSamp = GrabSamp.start;
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            robotHardware.setServoPos(Names.claw, 0);
        }
    }

    @Override
    public void start() {
        initSamplePath = new Path(new BezierLine(
                new Point(100, 100, Point.CARTESIAN),
                new Point(85, 103, Point.CARTESIAN)));
        initSamplePath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32));

        toSample1Path = new Path(new BezierLine(
                new Point(85, 103, Point.CARTESIAN),
                new Point(81, 120, Point.CARTESIAN)
        ));
        toSample1Path.setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(60));

        sample1ToBucketPath = new Path(new BezierLine(
                new Point(81, 120, Point.CARTESIAN),
                new Point(82, 104, Point.CARTESIAN)
        ));
        sample1ToBucketPath.setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(32));

        toSample2Path = new Path(new BezierLine(
                new Point(82, 104, Point.CARTESIAN),
                new Point(85, 118, Point.CARTESIAN)
        ));
        toSample2Path.setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(112));

        sample2ToBucketPath = new Path(new BezierLine(
                new Point(85, 118, Point.CARTESIAN),
                new Point(82, 104, Point.CARTESIAN)
        ));
        sample2ToBucketPath.setLinearHeadingInterpolation(Math.toRadians(112), Math.toRadians(32));

        toSample3Path = new Path(new BezierLine(
                new Point(82, 104, Point.CARTESIAN),
                new Point(76, 117.5, Point.CARTESIAN)
        ));
        toSample3Path.setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(109));

        sample3ToBucketPath = new Path(new BezierLine(
                new Point(76, 117.5, Point.CARTESIAN),
                new Point(82, 104, Point.CARTESIAN)
        ));
        sample3ToBucketPath.setLinearHeadingInterpolation(Math.toRadians(109), Math.toRadians(32));

        sampleParkPath = new Path(new BezierCurve(
                new Point(82, 104, Point.CARTESIAN),
                new Point(60, 170, Point.CARTESIAN),
                new Point(118, 150, Point.CARTESIAN)
        ));
        sampleParkPath.setLinearHeadingInterpolation(Math.toRadians(32), Math.toRadians(180));
    }

    @Override
    public void loop() {
        follower.update();
        switch (sampleEnum) {
            case scoreSample1:
                switch (scoreSamp) {
                    case start:
                        caseThingie(
                                () -> oTarget = 1940,
                                () -> oArmPos > 600,
                                () -> scoreSamp = ScoreSamp.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> {
                                    follower.followPath(initSamplePath, true);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                },
                                () -> !follower.isBusy(),
                                () -> scoreSamp = ScoreSamp.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        scoreSamp = ScoreSamp.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.15);
                            robotHardware.setServoPos(Names.outtakePivot, 0.1);
                            robotHardware.setServoPos(Names.intakePivot, 0.22);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            oTarget = 0;
                            scoreSamp = ScoreSamp.start;
                            sampleEnum = SampleEnum.getSample2;
                        }
                        break;
                }
                break;




            case getSample2:
                telemetry.addData("time", elapsedTime.seconds());
                switch (grabSamp) {
                    case start:
                        caseThingie(
                                () -> {follower.followPath(toSample1Path, true);
                                    robotHardware.setServoPos(Names.intakePivot, 0.47);
                                    robotHardware.setServoPos(Names.intakeArm, 0.76);
                                    robotHardware.setMotorPower(Names.slurp, 1);},
                                () -> !follower.isBusy(),
                                () -> grabSamp = GrabSamp.forward
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> iTarget = 300,
                                () -> elapsedTime.seconds() > 0.7 || robotHardware.isYellow(Names.intakeColor),
                                () -> grabSamp = GrabSamp.retract
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> iTarget = 0,
                                () -> iArmPos < 25,
                                () -> {
                                    robotHardware.setServoPos(Names.intakePivot, 0.18);
                                    robotHardware.setServoPos(Names.intakeArm, 0.045);
                                    grabSamp = GrabSamp.door;}
                        );
                        break;
                    case door:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.275,
                                () -> {robotHardware.setServoPos(Names.door, 0.4);
                                    grabSamp = GrabSamp.ready;}
                        );
                        break;
                    case ready:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 1.5,
                                () -> {robotHardware.setMotorPower(Names.slurp, 0);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.05);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.15);
                                    robotHardware.setServoPos(Names.intakePivot, 0.22);
                                    robotHardware.setServoPos(Names.intakeArm, 0.3);
                                    robotHardware.setServoPos(Names.door, 0.7);
                                    grabSamp = GrabSamp.claw;
                                }
                        );
                        break;
                    case claw:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.4,
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    grabSamp = GrabSamp.pause;}
                        );
                        break;
                    case pause:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.1,
                                () -> {
                                    sampleEnum = SampleEnum.scoreSample2;
                                    grabSamp = GrabSamp.start;}
                        );
                        break;
                }
                break;




            case scoreSample2:
                switch (scoreSamp) {
                    case start:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.4);
                                    oTarget = 1940;},
                                () -> oArmPos > 600,
                                () -> scoreSamp = ScoreSamp.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> follower.followPath(sample1ToBucketPath, true),
                                () -> !follower.isBusy(),
                                () -> scoreSamp = ScoreSamp.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        scoreSamp = ScoreSamp.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.15);
                            robotHardware.setServoPos(Names.outtakePivot, 0.1);
                            robotHardware.setServoPos(Names.intakePivot, 0.22);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            oTarget = 0;
                            scoreSamp = ScoreSamp.start;
                            sampleEnum = SampleEnum.getSample3;
                        }
                        break;
                }
                break;


            case getSample3:
                telemetry.addData("time", elapsedTime.seconds());
                switch (grabSamp) {
                    case start:
                        caseThingie(
                                () -> {follower.followPath(toSample2Path, true);
                                    robotHardware.setServoPos(Names.intakePivot, 0.47);
                                    robotHardware.setServoPos(Names.intakeArm, 0.76);
                                    robotHardware.setMotorPower(Names.slurp, 1);},
                                () -> !follower.isBusy(),
                                () -> grabSamp = GrabSamp.forward
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> iTarget = 400,
                                () -> elapsedTime.seconds() > 1.2 || robotHardware.isYellow(Names.intakeColor),
                                () -> grabSamp = GrabSamp.retract
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> iTarget = 0,
                                () -> iArmPos < 25,
                                () -> {
                                    robotHardware.setServoPos(Names.intakePivot, 0.18);
                                    robotHardware.setServoPos(Names.intakeArm, 0.045);
                                    grabSamp = GrabSamp.door;}
                        );
                        break;
                    case door:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.275,
                                () -> {robotHardware.setServoPos(Names.door, 0.4);
                                    grabSamp = GrabSamp.ready;}
                        );
                        break;
                    case ready:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 1.5,
                                () -> {robotHardware.setMotorPower(Names.slurp, 0);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.05);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.15);
                                    robotHardware.setServoPos(Names.intakePivot, 0.22);
                                    robotHardware.setServoPos(Names.intakeArm, 0.3);
                                    robotHardware.setServoPos(Names.door, 0.7);
                                    grabSamp = GrabSamp.claw;
                                }
                        );
                        break;
                    case claw:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.4,
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    grabSamp = GrabSamp.pause;}
                        );
                        break;
                    case pause:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.1,
                                () -> {
                                    sampleEnum = SampleEnum.scoreSample3;
                                    grabSamp = GrabSamp.start;}
                        );
                        break;
                }
                break;


            case scoreSample3:
                switch (scoreSamp) {
                    case start:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.4);
                                    oTarget = 1940;},
                                () -> oArmPos > 600,
                                () -> scoreSamp = ScoreSamp.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> follower.followPath(sample2ToBucketPath, true),
                                () -> !follower.isBusy(),
                                () -> scoreSamp = ScoreSamp.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        scoreSamp = ScoreSamp.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.15);
                            robotHardware.setServoPos(Names.outtakePivot, 0.1);
                            robotHardware.setServoPos(Names.intakePivot, 0.22);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            oTarget = 0;
                            scoreSamp = ScoreSamp.start;
                            sampleEnum = SampleEnum.getSample4;
                        }
                        break;
                }
                break;


            case getSample4:
                telemetry.addData("time", elapsedTime.seconds());
                switch (grabSamp) {
                    case start:
                        caseThingie(
                                () -> {follower.followPath(toSample3Path, true);
                                    robotHardware.setServoPos(Names.intakePivot, 0.47);
                                    robotHardware.setServoPos(Names.intakeArm, 0.76);
                                    robotHardware.setMotorPower(Names.slurp, 1);},
                                () -> !follower.isBusy(),
                                () -> grabSamp = GrabSamp.forward
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> iTarget = 275,
                                () -> elapsedTime.seconds() > 1.2 || robotHardware.isYellow(Names.intakeColor),
                                () -> grabSamp = GrabSamp.retract
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> iTarget = 0,
                                () -> iArmPos < 25,
                                () -> {robotHardware.setServoPos(Names.intakePivot, 0.18);
                                    robotHardware.setServoPos(Names.intakeArm, 0.045);
                                    grabSamp = GrabSamp.door;}
                        );
                        break;
                    case door:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.275,
                                () -> {robotHardware.setServoPos(Names.door, 0.4);
                                    grabSamp = GrabSamp.ready;}
                        );
                        break;
                    case ready:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 1.5,
                                () -> {robotHardware.setMotorPower(Names.slurp, 0);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.05);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.15);
                                    robotHardware.setServoPos(Names.intakePivot, 0.22);
                                    robotHardware.setServoPos(Names.intakeArm, 0.3);
                                    robotHardware.setServoPos(Names.door, 0.7);
                                    grabSamp = GrabSamp.claw;
                                }
                        );
                        break;
                    case claw:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.4,
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    grabSamp = GrabSamp.pause;}
                        );
                        break;
                    case pause:
                        caseThingie(
                                () -> {},
                                () -> elapsedTime.seconds() > 0.1,
                                () -> {
                                    sampleEnum = SampleEnum.scoreSample4;
                                    grabSamp = GrabSamp.start;}
                        );
                        break;
                }
                break;


            case scoreSample4:
                switch (scoreSamp) {
                    case start:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.4);
                                    oTarget = 1940;},
                                () -> oArmPos > 600,
                                () -> scoreSamp = ScoreSamp.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> follower.followPath(sample3ToBucketPath, true),
                                () -> !follower.isBusy(),
                                () -> scoreSamp = ScoreSamp.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        scoreSamp = ScoreSamp.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.15);
                            robotHardware.setServoPos(Names.outtakePivot, 0.1);
                            robotHardware.setServoPos(Names.intakePivot, 0.22);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            oTarget = 500;
                            scoreSamp = ScoreSamp.start;
                            sampleEnum = SampleEnum.park;
                        }
                        break;
                }
                break;
            case park:
                caseThingie(
                        () -> follower.followPath(sampleParkPath, true),
                        () -> false,
                        () -> {});
                break;
        }

        iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));

        oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;
        double oPower = outtakePID.calculate(oArmPos, oTarget) + of;
        robotHardware.setMotorPower(Names.leftOuttake, oPower);
        robotHardware.setMotorPower(Names.rightOuttake, oPower);

        if (robotHardware.isRed(Names.intakeColor)) robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.RED);
        else if (robotHardware.isBlue(Names.intakeColor)) robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else if (robotHardware.isYellow(Names.intakeColor)) robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        else robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);

        telemetry.update();
    }

    private void caseThingie (Runnable startSup, BooleanSupplier endCond, Runnable endSup){
        if (start) {
            startSup.run();
            elapsedTime.reset();
            start = false;
        }
        if (endCond.getAsBoolean()) {
            start = true;
            endSup.run();
        }
    }
}