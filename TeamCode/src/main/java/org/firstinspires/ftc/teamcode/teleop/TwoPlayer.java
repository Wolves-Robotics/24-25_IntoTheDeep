package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.complex.FullTransfer;
import org.firstinspires.ftc.teamcode.commands.complex.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.complex.IntakeRetract;
import org.firstinspires.ftc.teamcode.commands.complex.ReadyOuttake;
import org.firstinspires.ftc.teamcode.commands.drive.Drive;
import org.firstinspires.ftc.teamcode.commands.intake.BucketDown;
import org.firstinspires.ftc.teamcode.commands.intake.BucketUp;
import org.firstinspires.ftc.teamcode.commands.intake.ManualBackward;
import org.firstinspires.ftc.teamcode.commands.intake.ManualForward;
import org.firstinspires.ftc.teamcode.commands.intake.SetIntakeTarget;
import org.firstinspires.ftc.teamcode.commands.intake.StopManual;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OuttakeHighSample;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
public class TwoPlayer extends CommandOpMode {
    private RobotHardware robotHardware;
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSubsystem outtakeSubsystem;
    private DriveSubsystem driveSubsystem;

    private GamepadEx gamepadEx1, gamepadEx2;

    private boolean start = true;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();

        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.start();
        intakeSubsystem = IntakeSubsystem.getInstance(robotHardware);
        outtakeSubsystem = OuttakeSubsystem.getInstance(robotHardware);
        driveSubsystem = DriveSubsystem.getInstance(robotHardware);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);


        gamepadEx1.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new IntakeDown()
        );
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new FullTransfer()
        );
        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new ReadyOuttake()
        );
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X).whenPressed(
                new OpenClaw()
        );

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(
                new ManualForward()
        ).whenReleased(
                new StopManual()
        );
        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new ManualBackward()
        ).whenReleased(
                new StopManual()
        );
    }

    @Override
    public void run() {
        if (start) {
            robotHardware.startPids();
            start = false;
        }
        CommandScheduler.getInstance().run();
        schedule(new Drive(
                gamepad1.right_trigger,
                gamepad1.left_trigger,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                true));
    }
}
