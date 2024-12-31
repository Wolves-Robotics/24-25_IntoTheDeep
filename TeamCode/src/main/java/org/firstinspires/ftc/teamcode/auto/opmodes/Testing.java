package org.firstinspires.ftc.teamcode.auto.opmodes;

import static org.firstinspires.ftc.teamcode.utils.Constants.id;
import static org.firstinspires.ftc.teamcode.utils.Constants.ii;
import static org.firstinspires.ftc.teamcode.utils.Constants.ip;
import static org.firstinspires.ftc.teamcode.utils.Constants.od;
import static org.firstinspires.ftc.teamcode.utils.Constants.of;
import static org.firstinspires.ftc.teamcode.utils.Constants.oi;
import static org.firstinspires.ftc.teamcode.utils.Constants.op;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.collections.PathEnum;
import org.firstinspires.ftc.teamcode.auto.pedro.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedro.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

@Autonomous
public class Testing extends OpMode {
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
    private double iTarget=0, oTarget=0;
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
        follower.setStartingPose(new Pose(100, 100, 0));
        follower.update();
        elapsedTime = new ElapsedTime();

        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);

        pathEnum = PathEnum.scoreInitSample;
        score = score.start;
        grab = grab.start;
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            robotHardware.setServoPos(Names.claw, 0);
        }
    }

    @Override
    public void start() {
        initSamplePath = new Path(
                new BezierLine(new Point(100, 100, Point.CARTESIAN),
                new Point(85, 103, Point.CARTESIAN)));
        initSamplePath.setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(32));

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
                                () -> {
                                    follower.followPath(initSamplePath, true);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.45);
                                },
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
                            robotHardware.setServoPos(Names.intakePivot, 0.22);
                            robotHardware.setServoPos(Names.intakeArm, 0.3);
                            oTarget = 0;
                            score = score.start;
                            pathEnum = PathEnum.getSample1;
                        }
                        break;
                }
                break;
        }

        iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));

        oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;
        double oPower = outtakePID.calculate(oArmPos, oTarget) + of;
        robotHardware.setMotorPower(Names.leftOuttake, oPower);
        robotHardware.setMotorPower(Names.rightOuttake, oPower);
    }

    private void caseThingie (Runnable startSup, BooleanSupplier endCond, Runnable endSup){
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

enum score {
    start, move, place, retract
}

enum grab {
    start, forward, retract, door, ready, claw, pause
}