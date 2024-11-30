package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.teleop.commands.intakeZero;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.OuttakeSubsystem;

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

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(intakeSubsystem.setDown());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SequentialCommandGroup(
                        new intakeZero(intakeSubsystem),
                        intakeSubsystem.setPlace().withTimeout(200),
                        intakeSubsystem.openDoor().withTimeout(250),
                        intakeSubsystem.closeDoor()
                ));

        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(outtakeSubsystem.grab());

//        gamepadEx2.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed();
    }

    @Override
    public void run() {
        schedule(driveSubsystem.driveCommand(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                gamepadEx1.getRightX(),
                true));

        multiTelem.update();
    }
}
