package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.OuttakeSubsystem;

abstract public class BaseTeleop extends CommandOpMode {
    protected MultipleTelemetry multiTelem;
    private RobotHardware robotHardware;
    protected GamepadEx gamepadEx1, gamepadEx2;
    protected DriveSubsystem driveSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected OuttakeSubsystem outtakeSubsystem;

    abstract protected void setDefaultDrive();

    abstract protected void setCommands();

    @Override
    public void initialize() {
        multiTelem = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.setDaemon(true);
        robotHardware.start();

        driveSubsystem = new DriveSubsystem(robotHardware, multiTelem);
        intakeSubsystem = new IntakeSubsystem(robotHardware, multiTelem);
        outtakeSubsystem = new OuttakeSubsystem(robotHardware, multiTelem);

        CommandScheduler.getInstance().reset();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        setDefaultDrive();
        setCommands();
    }

    @Override
    public void run() {
        
    }
}
