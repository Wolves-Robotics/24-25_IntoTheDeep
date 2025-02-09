package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.collections.Color;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

abstract public class BaseTeleOp extends Thread {
    protected RobotHardware robotHardware;
    protected DriveSubsystem driveSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected OuttakeSubsystem outtakeSubsystem;

    protected Color color;

    protected BaseTeleOp(Color _color) {
        CommandScheduler.getInstance().reset();

        robotHardware = RobotHardware.getInstance();
        driveSubsystem = DriveSubsystem.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        outtakeSubsystem = OuttakeSubsystem.getInstance();

        color = _color;
    }

    @Override
    final public void run() {
        while (Thread.currentThread().isInterrupted()) {
            loop();
            CommandScheduler.getInstance().run();
        }
    }

    abstract protected void loop();

    abstract public void updateTelemetry(Telemetry telemetry);

    protected void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }
}
