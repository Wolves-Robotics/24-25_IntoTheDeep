package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.OuttakeSubsystem;

@TeleOp
public class PIDshit extends CommandOpMode {
    private MultipleTelemetry multipleTelemetry;
    private RobotHardware robotHardware;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSubsystem outtakeSubsystem;

    @Override
    public void initialize() {
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robotHardware = new RobotHardware(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(robotHardware, multipleTelemetry);
        outtakeSubsystem = new OuttakeSubsystem(robotHardware, multipleTelemetry);
    }

    @Override
    public void run() {

    }
}
