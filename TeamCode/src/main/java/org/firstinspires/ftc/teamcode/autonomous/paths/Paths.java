package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.collections.Color;
import org.firstinspires.ftc.teamcode.autonomous.collections.PathEnum;
import org.firstinspires.ftc.teamcode.autonomous.collections.ScoreSpecimen;
import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.autonomous.selection.AutoSelection;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.OuttakeSubsystem;

import java.util.function.BooleanSupplier;

public class Paths extends Thread {
    private RobotHardware robotHardware;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSubsystem outtakeSubsystem;
    private PathGen pathGen;
    private PathEnum pathEnum;
    private ScoreSpecimen scoreSpecimen;
    private Color color;
    private StartPos startPos;
    private double timeOffset;
    public Follower follower;
    private boolean start = true;
    private boolean running = false;

    public Paths(HardwareMap hardwareMap,
                 RobotHardware _robotHardware,
                 IntakeSubsystem _intakeSubsystem,
                 OuttakeSubsystem _outtakeSubsystem) {
        robotHardware = _robotHardware;
        intakeSubsystem = _intakeSubsystem;
        outtakeSubsystem = _outtakeSubsystem;
        pathGen = new PathGen();
        pathEnum = PathEnum.scoreInitSample;
        scoreSpecimen = ScoreSpecimen.forward;
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(7.256, 111.910, Math.toRadians(-90)));
    }

    @Override
    public void run() {
        pathGen.generate(startPos);
        while (!Thread.currentThread().isInterrupted()) {
            follower.update();
            switch (pathEnum) {
                case scoreInitSample:
                    switch (scoreSpecimen) {
                        case forward:
                            caseThingie(
                                    () -> follower.followPath(pathGen.getInitSamplePath(), true),
                                    () -> follower.atParametricEnd(),
                                    () -> pathEnum = PathEnum.getSample1);
                            break;
                        case place:
                            break;
                    }
                    break;
                case getSample1:
//                    caseThingie(
//                            () -> follower.followPath(pathGen.getToSample1Path()),
//                            () -> follower.atParametricEnd(),
//                            () -> pathEnum = PathEnum.scoreSample1);
                    break;
                case scoreSample1:
//                    caseThingie(
//                            () -> follower.followPath(pathGen.getSample1ToBucketPath()),
//                            () -> follower.atParametricEnd(),
//                            () -> pathEnum = PathEnum.getSample2
//                    );
                    break;
                case getSample2:
                    break;
                case getSample3:
                    break;
                case park:
                    break;
            }
            intakeSubsystem.updatePID();
            outtakeSubsystem.updatePID();
        }
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

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
    }

    public void setVars(AutoSelection selection) {
        color = selection.getColor();
        startPos = selection.getStartPos();
        timeOffset = selection.getTimeOffset();
    }
}
