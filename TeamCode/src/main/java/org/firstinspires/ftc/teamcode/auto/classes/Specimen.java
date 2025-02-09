package org.firstinspires.ftc.teamcode.auto.classes;

import static org.firstinspires.ftc.teamcode.utils.Constants.id;
import static org.firstinspires.ftc.teamcode.utils.Constants.ii;
import static org.firstinspires.ftc.teamcode.utils.Constants.ip;
import static org.firstinspires.ftc.teamcode.utils.Constants.od;
import static org.firstinspires.ftc.teamcode.utils.Constants.of;
import static org.firstinspires.ftc.teamcode.utils.Constants.oi;
import static org.firstinspires.ftc.teamcode.utils.Constants.op;

import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.collections.Color;
import org.firstinspires.ftc.teamcode.collections.auto.specimen.JailSpec;
import org.firstinspires.ftc.teamcode.collections.auto.specimen.ReadySpec;
import org.firstinspires.ftc.teamcode.collections.auto.specimen.ScoreSpec;
import org.firstinspires.ftc.teamcode.collections.auto.specimen.SpecEnum;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class Specimen extends BaseAuto {
    private SpecEnum specEnum;
    private ScoreSpec scoreSpec;
    private JailSpec jailSpec;
    private ReadySpec readySpec;

    private PathChain
            initSpecPath, jailSpec1Path,
            jailSpec2Path, jailSpec3Path,
            readySpec2Path, score2Path,
            readySpec3Path, score3Path,
            scoreToJail, scoreStrafe, park, score4;


    private PIDController intakePID, outtakePID;
    private double iTarget=0, oTarget=0;
    private int iArmPos, oArmPos;

    public Specimen(Color color) {
        super(color);

        robotHardware.setServoPos(Names.intakeArm, 0.1);
        robotHardware.setServoPos(Names.intakePivot, 0.19);
        robotHardware.setServoPos(Names.claw, 0.3);
        robotHardware.setServoPos(Names.outtakeArm, 0.23);
        robotHardware.setServoPos(Names.outtakePivot, 0.4);
        robotHardware.setServoPos(Names.door, 0.7);

        driveSubsystem.setFollower(robotHardware.getHardwareMap(), new Pose(135, 80, 0));


        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);

        specEnum = SpecEnum.score1;
        scoreSpec = ScoreSpec.move;
        jailSpec = JailSpec.move;
        readySpec = ReadySpec.move;
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
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(177))
                .addPath(
                        // Line 9
                        new BezierLine(
                                new Point(114.049, 113, Point.CARTESIAN),
                                new Point(128, 113, Point.CARTESIAN)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(177))
                .setZeroPowerAccelerationMultiplier(2)
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
        switch (specEnum) {
            case score1:
                switch (scoreSpec) {
                    case move:
                        caseThingie(
                                () -> {driveSubsystem.followPath(initSpecPath, true);
                                    RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.6);
                                    RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.55);
                                    oTarget = 1350;},
                                () -> driveSubsystem.atParametricEnd(),
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
                                    specEnum = SpecEnum.jail2;}
                        );
                        break;
                }
                break;

            case jail2:
                switch (jailSpec) {
                    case move:
                        caseThingie(
                                () -> {driveSubsystem.followPath(jailSpec1Path, true);
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
                                () -> driveSubsystem.atParametricEnd(),
                                () -> {
                                    jailSpec = JailSpec.move;
                                    specEnum = SpecEnum.jail3;
                                }
                        );
                        break;
                }
                break;

            case jail3:
                switch (jailSpec) {
                    case move:
                        caseThingie(
                                () -> {
                                    driveSubsystem.followPath(jailSpec2Path, true);
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
                                () -> driveSubsystem.atParametricEnd(),
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
                                    driveSubsystem.followPath(readySpec2Path, true);
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
                                    RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.6);
                                    RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.55);
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
                                    driveSubsystem.followPath(score2Path, true);
                                },
                                () -> driveSubsystem.atParametricEnd(),
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
                                    driveSubsystem.followPath(readySpec3Path, true);
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
                                    RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.6);
                                    RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.55);
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
                                    driveSubsystem.followPath(score3Path, true);
                                },
                                () -> driveSubsystem.atParametricEnd(),
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
                                    driveSubsystem.followPath(readySpec3Path, true);
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
                                    RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.6);
                                    RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.55);
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
                                    driveSubsystem.followPath(score4, true);
                                },
                                () -> driveSubsystem.atParametricEnd(),
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
                        () -> driveSubsystem.followPath(park),
                        () -> driveSubsystem.atParametricEnd(),
                        () -> {}
                );

        }

        iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));

        oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;
        double oPower = outtakePID.calculate(oArmPos, oTarget) + of;
        robotHardware.setMotorPower(Names.leftOuttake, oPower);
        robotHardware.setMotorPower(Names.rightOuttake, oPower);

    }

    @Override
    public void updateTelemetry(Telemetry telemetry) {

    }
}
