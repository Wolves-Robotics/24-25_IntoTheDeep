package org.firstinspires.ftc.teamcode.autonomous.paths;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.autonomous.collections.Color;
import org.firstinspires.ftc.teamcode.autonomous.collections.PathEnum;
import org.firstinspires.ftc.teamcode.autonomous.collections.ScoreSpecimen;
import org.firstinspires.ftc.teamcode.autonomous.collections.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.autonomous.selection.AutoSelection;
import org.firstinspires.ftc.teamcode.hardware.Names;
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
    private Follower follower;
    private boolean start = true;

    public Paths(HardwareMap hardwareMap,
                 RobotHardware _robotHardware,
                 IntakeSubsystem _intakeSubsystem,
                 OuttakeSubsystem _outtakeSubsystem) {
        robotHardware = _robotHardware;
        intakeSubsystem = _intakeSubsystem;
        outtakeSubsystem = _outtakeSubsystem;
        pathGen = new PathGen();
        pathEnum = PathEnum.park;
        scoreSpecimen = ScoreSpecimen.forward;
        follower = new Follower(hardwareMap);
    }

    @Override
    public void run() {
        pathGen.generate(startPos);
        while (!Thread.currentThread().isInterrupted()) {
            follower.update();
            switch (pathEnum) {
                case scoreSpecimen:
                    switch (scoreSpecimen) {
                        case forward:
                            caseThingie(
                                    () -> {
                                        outtakeSubsystem.setTarget(300);
                                        follower.followPath(pathGen.getFirstSpecimenPath());
                                    },
                                    () -> follower.atParametricEnd(),
                                    () -> scoreSpecimen = ScoreSpecimen.place
                            );
                            break;
                        case place:
                            caseThingie(
                                    () -> outtakeSubsystem.setTarget(250),
                                    () -> robotHardware.getMotorPos(Names.leftOuttake) < 300,
                                    () -> {
                                        outtakeSubsystem.setTarget(100);
                                        robotHardware.setServoPos(Names.claw, 0);
                                        pathEnum = PathEnum.getSample1;
                                    }
                            );
                            break;
                    }
                    break;
                case scoreSample:
                    break;
                case getSample1:
                    break;
                case getSample2:
                    break;
                case getSample3:
                    break;
                case park:
                    break;
            }
        }
        intakeSubsystem.updatePID();
        outtakeSubsystem.updatePID();
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

    }

    public void setVars(AutoSelection selection) {
        color = selection.getColor();
        startPos = selection.getStartPos();
        timeOffset = selection.getTimeOffset();
    }
}
