package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.autonomous.selection.AutoSelection;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.OuttakeSubsystem;

@Autonomous
public class BaseAuto extends OpMode {
    private RobotHardware robotHardware;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSubsystem outtakeSubsystem;
    private MultipleTelemetry multipleTelemetry;
    private AutoSelection autoSelection;
    private Paths paths;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        intakeSubsystem = new IntakeSubsystem(robotHardware, multipleTelemetry);
        outtakeSubsystem = new OuttakeSubsystem(robotHardware, multipleTelemetry);
        autoSelection = new AutoSelection(gamepad1);
        autoSelection.start();
        paths = new Paths(hardwareMap, robotHardware, intakeSubsystem, outtakeSubsystem);
    }

    @Override
    public void init_loop() {
        autoSelection.updateTelem(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        paths.setVars(autoSelection);
        autoSelection.interrupt();
        paths.start();
    }

    @Override
    public void loop() {
        paths.updateTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        paths.interrupt();
    }
}
