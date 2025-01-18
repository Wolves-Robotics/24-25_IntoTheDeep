package org.firstinspires.ftc.teamcode.auto.classes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.collections.Color;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

abstract public class BaseAuto extends Thread{
    protected CommandScheduler commandScheduler;
    protected RobotHardware robotHardware;
    protected DriveSubsystem driveSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected OuttakeSubsystem outtakeSubsystem;

    protected Color color;

    protected ElapsedTime elapsedTime;
    protected boolean start;

    protected BaseAuto(Color _color) {
        CommandScheduler.getInstance().reset();
        commandScheduler = CommandScheduler.getInstance();

        robotHardware = RobotHardware.getInstance();
        driveSubsystem = DriveSubsystem.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        outtakeSubsystem = OuttakeSubsystem.getInstance();

        color = _color;

        elapsedTime = new ElapsedTime();
        start = true;

        start();
    }

    @Override
    final public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            loop();
            commandScheduler.run();
        }
    }

    abstract public void loop();

    abstract public void updateTelemetry(Telemetry telemetry);

    protected void schedule(Command command) {
        commandScheduler.schedule(command);
    }

    protected void caseThingie (Runnable startSup, BooleanSupplier endCond, Runnable endSup){
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
