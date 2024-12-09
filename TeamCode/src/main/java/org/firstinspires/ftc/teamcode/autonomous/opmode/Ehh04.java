package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.autonomous.collections.PathEnum;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.hardware.Names;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

import java.util.function.BooleanSupplier;

@Autonomous
public class Ehh04 extends OpMode {
    private Follower follower;
    private PathEnum pathEnum;
    private score score;
    private grab grab;
    private RobotHardware robotHardware;
    private ElapsedTime elapsedTime;
    private boolean start = true;
    private Path
            firstSpecimenPath, initSamplePath,
            toSample1Path, sample1ToBucketPath,
            toSample2Path, sample2ToBucketPath,
            toSample3Path, sample3ToBucketPath,
            sampleParkPath;


    private PIDController intakePID, outtakePID;
    private final double ticksPerDeg = 760/180.;
    private double ip=0.014, ii=0.15, id=0.00081, op=0.015, oi=0, od=0.00022 , of=0.05;
    private double iTarget=0, oTarget=0;
    double ff = Math.cos(Math.toRadians(oTarget / ticksPerDeg)) * of;
    private int iArmPos, oArmPos;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.setServoPos(Names.intakeArm, 0.1);
        robotHardware.setServoPos(Names.intakePivot, 0.19);
        robotHardware.setServoPos(Names.claw, 0.3);
        robotHardware.setServoPos(Names.outtakeArm, 0.23);
        robotHardware.setServoPos(Names.outtakePivot, 0.4);
        robotHardware.setServoPos(Names.door, 0.7);
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7.256, 108.3, Math.toRadians(-90)));
        elapsedTime = new ElapsedTime();

        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);

        pathEnum = PathEnum.scoreInitSample;
        score = score.start;
        grab = grab.start;
    }

    @Override
    public void init_loop() {
        if(gamepad1.a){
            robotHardware.setServoPos(Names.claw, 0);
        }
    }

    @Override
    public void start() {
        initSamplePath = new Path(new BezierLine(new Point(7.256, 108.3, Point.CARTESIAN),
                new Point(14.674, 129.648, Point.CARTESIAN)));
        initSamplePath.setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-45));

        toSample1Path = new Path(new BezierLine(
                new Point(14.674, 129.648, Point.CARTESIAN),
                new Point(25, 120.9, Point.CARTESIAN)
        ));
        toSample1Path.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));

        sample1ToBucketPath = new Path(new BezierLine(
                new Point(25.000, 120.9, Point.CARTESIAN),
                new Point(14.8, 129.4, Point.CARTESIAN)
        ));
        sample1ToBucketPath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        toSample2Path = new Path(new BezierLine(
                new Point(14.674, 129.648, Point.CARTESIAN),
                new Point(25.000, 129.2, Point.CARTESIAN)
        ));
        toSample2Path.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));

        sample2ToBucketPath = new Path(new BezierLine(
                new Point(25.000, 129.2, Point.CARTESIAN),
                new Point(14.8, 129.4, Point.CARTESIAN)
        ));
        sample2ToBucketPath.setLinearHeadingInterpolation(0, Math.toRadians(-45));

        toSample3Path = new Path(new BezierLine(
                new Point(14.8, 129.4, Point.CARTESIAN),
                new Point(29.154, 129.987, Point.CARTESIAN)
        ));
        toSample3Path.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45));

        sample3ToBucketPath = new Path(new BezierCurve(
                new Point(29.154, 129.987, Point.CARTESIAN),
                new Point(14.8, 129.4, Point.CARTESIAN)
        ));
        sample3ToBucketPath.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(45));

        sampleParkPath = new Path(new BezierLine(
                new Point(14.8, 129.4, Point.CARTESIAN),
                new Point(60, 110, Point.CARTESIAN)
        ));
        sampleParkPath.setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-90));
    }

    @Override
    public void loop() {
        follower.update();
        switch (pathEnum) {
            case scoreInitSample:
                switch (score) {
                    case start:
                        caseThingie(
                                () -> oTarget = 1940,
                                () -> oArmPos > 1000,
                                () -> score = score.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> {follower.followPath(initSamplePath, true);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.45);},
                                () -> !follower.isBusy(),
                                () -> score = score.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        score = score.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.09);
                            robotHardware.setServoPos(Names.outtakePivot, 0.15);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            robotHardware.setServoPos(Names.intakePivot, 0.2);
                            oTarget = 0;
                            score = score.start;
                            pathEnum = PathEnum.getSample1;
                        }
                        break;
                }
                break;
            case getSample1:
                switch (grab) {
                    case start:
                        caseThingie(
                                () -> {follower.followPath(toSample1Path, true);
                                    robotHardware.setServoPos(Names.intakeArm, 0.89);
                                    robotHardware.setServoPos(Names.intakePivot, 0.6  );
                                    robotHardware.setMotorPower(Names.slurp, 1);},
                                () -> !follower.isBusy(),
                                () -> {grab = grab.forward;
                                    elapsedTime.reset();}
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> iTarget = 250 ,
                                () -> elapsedTime.seconds() > 1.5,
                                () -> {iTarget = 0;
                                    grab = grab.retract;}
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> iTarget = 0,
                                () -> iArmPos < 10,
                                () -> {robotHardware.setServoPos(Names.intakeArm, 0.11);
                                    robotHardware.setServoPos(Names.intakePivot, 0.2);
                                    robotHardware.setServoPos(Names.door, 0.45);
                                    grab = grab.ready;}
                        );
                        break;
                    case ready:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 1,
                                () -> {robotHardware.setMotorPower(Names.slurp, 0);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.05);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.15);
                                    robotHardware.setServoPos(Names.intakeArm, 0.35);
                                    robotHardware.setServoPos(Names.intakePivot, 0.2);
                                    robotHardware.setServoPos(Names.door, 0.7);
                                    grab = grab.claw;
                                }
                        );
                        break;
                    case claw:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 0.25,
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    grab = grab.pause;}
                        );
                        break;
                    case pause:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 0.1,
                                () -> {pathEnum = PathEnum.scoreSample1;
                                    grab = grab.start;}
                        );
                        break;
                }
                break;
            case scoreSample1:
                switch (score) {
                    case start:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.4);
                                    oTarget = 1940;},
                                () -> oArmPos > 1000,
                                () -> score = score.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> follower.followPath(sample1ToBucketPath, true),
                                () -> !follower.isBusy(),
                                () -> score = score.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        score = score.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.09);
                            robotHardware.setServoPos(Names.outtakePivot, 0.15);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            robotHardware.setServoPos(Names.intakePivot, 0.2);
                            oTarget = 0;
                            score = score.start;
                            pathEnum = PathEnum.getSample2;
                        }
                        break;
                }
                break;
            case getSample2:
                switch (grab) {
                    case start:
                        caseThingie(
                                () -> {follower.followPath(toSample2Path, true);
                                    robotHardware.setServoPos(Names.intakeArm, 0.89);
                                    robotHardware.setServoPos(Names.intakePivot, 0.6  );
                                    robotHardware.setMotorPower(Names.slurp, 1);},
                                () -> !follower.isBusy(),
                                () -> {grab = grab.forward;
                                    elapsedTime.reset();}
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> iTarget = 250 ,
                                () -> elapsedTime.seconds() > 1.5,
                                () -> {iTarget = 0;
                                    grab = grab.retract;}
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> iTarget = 0,
                                () -> iArmPos < 10,
                                () -> {robotHardware.setServoPos(Names.intakeArm, 0.11);
                                    robotHardware.setServoPos(Names.intakePivot, 0.2);
                                    robotHardware.setServoPos(Names.door, 0.45);
                                    grab = grab.ready;}
                        );
                        break;
                    case ready:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 1,
                                () -> {robotHardware.setMotorPower(Names.slurp, 0);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.05);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.15);
                                    robotHardware.setServoPos(Names.intakeArm, 0.35);
                                    robotHardware.setServoPos(Names.intakePivot, 0.2);
                                    robotHardware.setServoPos(Names.door, 0.7);
                                    grab = grab.claw;
                                }
                        );
                        break;
                    case claw:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 0.25,
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    grab = grab.pause;}
                        );
                        break;
                    case pause:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 0.1,
                                () -> {pathEnum = PathEnum.scoreSample2;
                                    grab = grab.start;}
                        );
                        break;
                }
                break;
            case scoreSample2:
                switch (score) {
                    case start:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.4);
                                    oTarget = 1940;},
                                () -> oArmPos > 1000,
                                () -> score = score.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> follower.followPath(sample2ToBucketPath, true),
                                () -> !follower.isBusy(),
                                () -> score = score.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        score = score.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.09);
                            robotHardware.setServoPos(Names.outtakePivot, 0.15);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            robotHardware.setServoPos(Names.intakePivot, 0.2);
                            oTarget = 0;
                            score = score.start;
                            pathEnum = PathEnum.getSample3;
                        }
                        break;
                }
                break;
            case getSample3:
                switch (grab) {
                    case start:
                        caseThingie(
                                () -> {follower.followPath(toSample3Path, true);
                                    robotHardware.setServoPos(Names.intakeArm, 0.89);
                                    robotHardware.setServoPos(Names.intakePivot, 0.6  );
                                    robotHardware.setMotorPower(Names.slurp, 1);},
                                () -> !follower.isBusy(),
                                () -> {grab = grab.forward;
                                    elapsedTime.reset();}
                        );
                        break;
                    case forward:
                        caseThingie(
                                () -> iTarget = 250 ,
                                () -> elapsedTime.seconds() > 1.5,
                                () -> {iTarget = 0;
                                    grab = grab.retract;}
                        );
                        break;
                    case retract:
                        caseThingie(
                                () -> iTarget = 0,
                                () -> iArmPos < 10,
                                () -> {robotHardware.setServoPos(Names.intakeArm, 0.11);
                                    robotHardware.setServoPos(Names.intakePivot, 0.2);
                                    robotHardware.setServoPos(Names.door, 0.45);
                                    grab = grab.ready;}
                        );
                        break;
                    case ready:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 1,
                                () -> {robotHardware.setMotorPower(Names.slurp, 0);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.05);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.15);
                                    robotHardware.setServoPos(Names.intakeArm, 0.35);
                                    robotHardware.setServoPos(Names.intakePivot, 0.2);
                                    robotHardware.setServoPos(Names.door, 0.7);
                                    grab = grab.claw;
                                }
                        );
                        break;
                    case claw:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 0.25,
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    grab = grab.pause;}
                        );
                        break;
                    case pause:
                        caseThingie(
                                () -> elapsedTime.reset(),
                                () -> elapsedTime.seconds() > 0.1,
                                () -> {pathEnum = PathEnum.scoreSample3;
                                    grab = grab.start;}
                        );
                        break;
                }
                break;
            case scoreSample3:
                switch (score) {
                    case start:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.4);
                                    oTarget = 1940;},
                                () -> oArmPos > 1000,
                                () -> score = score.move
                        );
                        break;
                    case move:
                        caseThingie(
                                () -> follower.followPath(sample3ToBucketPath, true),
                                () -> !follower.isBusy(),
                                () -> score = score.place
                        );
                        break;
                    case place:
                        robotHardware.setServoPos(Names.claw, 0.35);
                        elapsedTime.reset();
                        score = score.retract;
                        break;
                    case retract:
                        if (elapsedTime.seconds() > 0.1) {
                            robotHardware.setServoPos(Names.outtakeArm, 0.09);
                            robotHardware.setServoPos(Names.outtakePivot, 0.15);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            robotHardware.setServoPos(Names.intakePivot, 0.2);
                            oTarget = 0;
                            score = score.start;
                            pathEnum = PathEnum.park;
                        }
                        break;
                }
                break;
            case park:
                caseThingie(
                        () -> follower.followPath(sampleParkPath),
                        () -> false,
                        () -> {}
                );
                break;
        }

        iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));

        oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;
        double oPower = outtakePID.calculate(oArmPos, oTarget) + ff;
        robotHardware.setMotorPower(Names.leftOuttake, oPower);
        robotHardware.setMotorPower(Names.rightOuttake, oPower);
    }

    private void caseThingie(Runnable startSup, BooleanSupplier endCond, Runnable endSup) {
        if (start) {
            startSup.run();
            start = false;
        }
        if (endCond.getAsBoolean()) {
            start = true;
            endSup.run();
        }
    }
}