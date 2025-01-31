package org.firstinspires.ftc.teamcode.auto.classes;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.localization.PoseUpdater;
import com.pedropathing.util.DashboardPoseTracker;
import com.pedropathing.util.Drawing;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.collections.Color;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

abstract public class BaseAuto extends Thread{

    protected RobotHardware robotHardware;
    protected DriveSubsystem driveSubsystem;
    protected IntakeSubsystem intakeSubsystem;
    protected OuttakeSubsystem outtakeSubsystem;

    protected MultipleTelemetry telemetryA;

    protected Color color;

    protected ElapsedTime elapsedTime;
    protected boolean start;

    protected BaseAuto(Color _color) {
        CommandScheduler.getInstance().reset();

        robotHardware = RobotHardware.getInstance();
        driveSubsystem = DriveSubsystem.getInstance();
        intakeSubsystem = IntakeSubsystem.getInstance();
        outtakeSubsystem = OuttakeSubsystem.getInstance();

        color = _color;

        elapsedTime = new ElapsedTime();
        start = true;
    }

    @Override
    final public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            loop();
            CommandScheduler.getInstance().run();
        }
    }

    abstract public void loop();

    abstract public void updateTelemetry(Telemetry telemetry);

    protected void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
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
