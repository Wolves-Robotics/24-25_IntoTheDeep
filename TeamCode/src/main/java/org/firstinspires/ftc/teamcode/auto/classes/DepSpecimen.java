package org.firstinspires.ftc.teamcode.auto.classes;

import static org.firstinspires.ftc.teamcode.utils.Constants.id;
import static org.firstinspires.ftc.teamcode.utils.Constants.ii;
import static org.firstinspires.ftc.teamcode.utils.Constants.ip;
import static org.firstinspires.ftc.teamcode.utils.Constants.od;
import static org.firstinspires.ftc.teamcode.utils.Constants.of;
import static org.firstinspires.ftc.teamcode.utils.Constants.oi;
import static org.firstinspires.ftc.teamcode.utils.Constants.op;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.collections.JailSpec;
import org.firstinspires.ftc.teamcode.auto.collections.ParkBot;
import org.firstinspires.ftc.teamcode.auto.collections.ReadySpec;
import org.firstinspires.ftc.teamcode.auto.collections.ScoreSpec;
import org.firstinspires.ftc.teamcode.auto.collections.SpecEnum;
import org.firstinspires.ftc.teamcode.auto.pedro.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedro.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

@Autonomous
@Disabled
public class DepSpecimen extends OpMode {
    private Follower follower;
    private SpecEnum specEnum;
    private ScoreSpec scoreSpec;
    private JailSpec jailSpec;
    private ReadySpec readySpec;
    private ParkBot parkBot;
    private RobotHardware robotHardware;
    private ElapsedTime elapsedTime;
    private boolean start = true;
    private PathChain
            initSpecPath, jailSpec1Path,
            jailSpec2Path, jailSpec3Path,
            readySpec2Path, score2Path,
            readySpec3Path, score3Path,
            scoreToJail, scoreStrafe, park, score4, idle;


    private PIDController intakePID, outtakePID;
    private double iTarget=0, oTarget=0;
    private int iArmPos, oArmPos;
    private int count = 0;

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
        follower.setStartingPose(new Pose(135, 80, 0));
        follower.update();
        elapsedTime = new ElapsedTime();

        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);

        specEnum = SpecEnum.score1;
        scoreSpec = ScoreSpec.move;
        jailSpec = JailSpec.move;
        readySpec = ReadySpec.move;
        parkBot = ParkBot.move;
    }

    @Override
    public void init_loop() {
        if (gamepad1.a) {
            robotHardware.setServoPos(Names.claw, 0);
        }
    }

    @Override
    public void start() {
        initSpecPath = new PathBuilder()
                // Line 1
                .addPath(new BezierLine(
                                new Point(135, 80, Point.CARTESIAN),
                                new Point(108.1, 80, Point.CARTESIAN)
                ))
                .setConstantHeadingInterpolation(0).build();

        jailSpec1Path = new PathBuilder()
                // Line 2
                .addPath(new BezierCurve(
                                new Point(108, 80, Point.CARTESIAN),
                                new Point(135, 80, Point.CARTESIAN),
                                new Point(106, 102, Point.CARTESIAN)
                ))
                .setTangentHeadingInterpolation()
                // Line 3
                .addPath(new BezierCurve(
                        new Point(106, 102, Point.CARTESIAN),
                        new Point(113, 98, Point.CARTESIAN),
                        new Point(120, 102, Point.CARTESIAN)
                ))
                .setLinearHeadingInterpolation(Math.toRadians(143), Math.toRadians(60))
                .build();

        jailSpec2Path = new PathBuilder()
                // Line 4
                .addPath(new BezierCurve(
                        new Point(120, 102, Point.CARTESIAN),
                        new Point(106, 95, Point.CARTESIAN),
                        new Point(107, 114, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(165))
                // Line 5
                .addPath(new BezierCurve(
                        new Point(107, 114, Point.CARTESIAN),
                        new Point(113, 109, Point.CARTESIAN),
                        new Point(120, 113, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(165), Math.toRadians(60))
                .build();

        readySpec2Path = new PathBuilder()
                .addPath(
                        // Line 8
                        new BezierCurve(
                                new Point(133.407, 120.287, Point.CARTESIAN),
                                new Point(125.000, 113.250, Point.CARTESIAN),
                                new Point(114.5, 113.673, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(114.049, 113, Point.CARTESIAN),
                                new Point(129.700, 113, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        score2Path = new PathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(129.519, 114.479, Point.CARTESIAN),
                                new Point(135.000, 70.000, Point.CARTESIAN),
                                new Point(110.000, 78.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        readySpec3Path = new PathBuilder()
                .addPath(
                        // Line 11
                        new BezierCurve(
                                new Point(108.000, 78.000, Point.CARTESIAN),
                                new Point(130.000, 93.000, Point.CARTESIAN),
                                new Point(77.000, 115.000, Point.CARTESIAN),
                                new Point(129.700, 114.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(183))
                .build();
        score3Path = new PathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(130.019, 114.479, Point.CARTESIAN),
                                new Point(135.000, 80.000, Point.CARTESIAN),
                                new Point(110.000, 73.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        score4 = new PathBuilder()
                .addPath(
                        // Line 10
                        new BezierCurve(
                                new Point(130.019, 114.479, Point.CARTESIAN),
                                new Point(140.000, 60.000, Point.CARTESIAN),
                                new Point(110.000, 70.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .build();
        idle = new PathBuilder()
                .addPath(
                        // Line 10
                        new BezierLine(
                                new Point(129.697, 114.318, Point.CARTESIAN),
                                new Point(110.500, 81.894, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .addPath(
                        // Line 11
                        new BezierLine(
                                new Point(110.500, 81.894, Point.CARTESIAN),
                                new Point(108, 82.055 + 14, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        scoreStrafe = new PathBuilder()
                .addPath(
                        // Line 12
                        new BezierLine(
                                new Point(108.565, 81.087, Point.CARTESIAN),
                                new Point(107.919, 74.635, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        park = new PathBuilder()
                .addPath(
                        // Line 13
                        new BezierLine(
                                new Point(107.919, 74.635, Point.CARTESIAN),
                                new Point(133.407, 121.739, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    @Override
    public void loop() {
        follower.update();

        switch (specEnum) {
            case score1:
                switch (scoreSpec) {
                    case move:
                        caseThingie(
                                () -> {follower.followPath(initSpecPath, true);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.76);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.35);
                                    oTarget = 1350;},
                                () -> follower.atParametricEnd(),
                                () -> scoreSpec = ScoreSpec.down
                        );
                        break;
                    case down:
                        caseThingie(
                                () -> {oTarget = 0;
                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.75,
                                () -> {robotHardware.setServoPos(Names.claw, 0.35);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.74);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.29);
                                    scoreSpec = ScoreSpec.move;
                                    specEnum = SpecEnum.jail1;}
                        );
                        break;
                }
                break;

            case jail1:
                switch (jailSpec) {
                    case move:
                        caseThingie(
                                () -> {follower.followPath(jailSpec1Path, true);
                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.75,
                                () -> {iTarget = 350;
                                    robotHardware.setServoPos(Names.intakePivot, 0.61);
                                    robotHardware.setServoPos(Names.intakeArm, 0.77);
                                    jailSpec = JailSpec.extend;}
                        );
                        break;
                    case extend:
                        caseThingie(
                                () -> {},
                                () -> follower.atParametricEnd(),
                                () -> {
                                    jailSpec = JailSpec.move;
                                    specEnum = SpecEnum.jail2;
                                }
                        );
                        break;
                }
                break;

            case jail2:
                switch (jailSpec) {
                    case move:
                        caseThingie(
                                () -> {
                                    follower.followPath(jailSpec2Path, true);
                                    elapsedTime.reset();
                                    iTarget = 0;
                                },
                                () -> elapsedTime.seconds() > 0.9,
                                () -> {
                                    iTarget = 350;
                                    jailSpec = JailSpec.extend;
                                }
                        );
                        break;
                    case extend:
                        caseThingie(
                                () -> {
                                },
                                () -> follower.atParametricEnd(),
                                () -> {
                                    jailSpec = JailSpec.move;
                                    specEnum = SpecEnum.ready2;
                                }
                        );
                        break;
                }
                break;

            case ready2:
                switch (readySpec) {
                    case move:
                        caseThingie(
                                () -> {
                                    follower.followPath(readySpec2Path, true);
                                    iTarget = 0;
                                    robotHardware.setServoPos(Names.intakePivot, 0.22);
                                    robotHardware.setServoPos(Names.intakeArm, 0.045);
                                    elapsedTime.reset();
                                },
                                () -> elapsedTime.seconds() > 2.8,
                                () -> readySpec = ReadySpec.grab
                        );
                        break;
                    case grab:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.4,
                                () -> {
                                    robotHardware.setServoPos(Names.outtakeArm, 0.76);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.35);
                                    readySpec = ReadySpec.move;
                                    specEnum = SpecEnum.score2;
                                }
                        );
                        break;
                }
                break;

            case score2:
                switch (scoreSpec){
                    case move:
                        caseThingie(
                                () ->{
                                    oTarget = 1350;
                                    follower.followPath(score2Path, true);
                                    },
                                () -> follower.atParametricEnd(),
                                () -> scoreSpec = ScoreSpec.down
                        );
                        break;
                    case down:
                        caseThingie(
                                () -> {oTarget = 0;
                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.75,
                                () -> {

                                    robotHardware.setServoPos(Names.claw, 0.35);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.74);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.29);
                                    scoreSpec = ScoreSpec.move;
                                    specEnum = SpecEnum.ready3;

                                }
                        );
                        break;
                }
                break;

            case ready3:
                switch (readySpec) {
                    case move:
                        caseThingie(
                                () -> {
                                    follower.followPath(readySpec3Path, true);
                                    elapsedTime.reset();
                                },
                                () -> elapsedTime.seconds() > 2.8,
                                () -> readySpec = ReadySpec.grab
                        );
                        break;
                    case grab:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.4,
                                () -> {
                                    robotHardware.setServoPos(Names.outtakeArm, 0.76);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.35);
                                    readySpec = ReadySpec.move;
                                    specEnum = SpecEnum.score3;
                                }
                        );
                        break;
                }
                break;

            case score3:
                switch (scoreSpec){
                    case move:
                        caseThingie(
                                () ->{
                                    oTarget = 1350;
                                    follower.followPath(score3Path, true);
                                },
                                () -> follower.atParametricEnd(),
                                () -> scoreSpec = ScoreSpec.down
                        );
                        break;
                    case down:
                        caseThingie(
                                () -> {oTarget = 0;

                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.75,
                                () -> {
                                    robotHardware.setServoPos(Names.claw, 0.35);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.74);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.29);
                                    scoreSpec = ScoreSpec.move;
                                    specEnum = SpecEnum.ready4;

                                }
                        );
                        break;
                }
                break;
            case ready4:
                switch (readySpec) {
                    case move:
                        caseThingie(
                                () -> {
                                    follower.followPath(readySpec3Path, true);
                                    elapsedTime.reset();
                                },
                                () -> elapsedTime.seconds() > 2.8,
                                () -> readySpec = ReadySpec.grab
                        );
                        break;
                    case grab:
                        caseThingie(
                                () -> {robotHardware.setServoPos(Names.claw, 0);
                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.4,
                                () -> {
                                    robotHardware.setServoPos(Names.outtakeArm, 0.76);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.35);
                                    readySpec = ReadySpec.move;
                                    specEnum = SpecEnum.score4;
                                }
                        );
                        break;
                }
                break;

            case score4:
                switch (scoreSpec){
                    case move:
                        caseThingie(
                                () ->{
                                    oTarget = 1350;
                                    follower.followPath(score4, true);
                                },
                                () -> follower.atParametricEnd(),
                                () -> scoreSpec = ScoreSpec.down
                        );
                        break;
                    case down:
                        caseThingie(
                                () -> {oTarget = 0;

                                    elapsedTime.reset();},
                                () -> elapsedTime.seconds() > 0.75,
                                () -> {
                                    robotHardware.setServoPos(Names.claw, 0.35);
                                    robotHardware.setServoPos(Names.outtakeArm, 0.74);
                                    robotHardware.setServoPos(Names.outtakePivot, 0.29);
                                    scoreSpec = ScoreSpec.move;
                                    specEnum = SpecEnum.park;

                                }
                        );
                        break;
                }
                break;
            case park:
                caseThingie(

                        () -> follower.followPath(park),
                        () -> follower.atParametricEnd(),
                        () -> {}
                );

        }

        iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));

        oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;
        double oPower = outtakePID.calculate(oArmPos, oTarget) + of;
        robotHardware.setMotorPower(Names.leftOuttake, oPower);
        robotHardware.setMotorPower(Names.rightOuttake, oPower);

        telemetry.update();
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
