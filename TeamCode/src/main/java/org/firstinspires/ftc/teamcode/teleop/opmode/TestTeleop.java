package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.teleop.subsystems.OuttakeSubsystem;

@TeleOp
public class TestTeleop extends CommandOpMode {
    protected MultipleTelemetry multiTelem;
    private RobotHardware robotHardware;
    private GamepadEx gamepadEx1, gamepadEx2;
    private DriveSubsystem driveSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSubsystem outtakeSubsystem;

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

        driveSubsystem.setDefaultCommand(driveSubsystem.driveCommand(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                gamepadEx1.getRightX(),
                false));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(intakeSubsystem.toZero());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT)
                .whenPressed(intakeSubsystem.increaseTarget());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(intakeSubsystem.decreaseTarget());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(outtakeSubsystem.toZero());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(outtakeSubsystem.bucket1());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(outtakeSubsystem.bucket2());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(outtakeSubsystem.lowChamber());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(outtakeSubsystem.highChamber());
    }

    @Override
    public void run() {
        multiTelem.update();
    }
}
