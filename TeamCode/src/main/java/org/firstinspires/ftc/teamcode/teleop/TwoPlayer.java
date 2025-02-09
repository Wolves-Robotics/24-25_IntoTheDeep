package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.complex.sample.GrabSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeDown;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetract;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyHighSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.ReadyLowSample;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenGrab;
import org.firstinspires.ftc.teamcode.commands.complex.specimen.ReadySpecimenPlace;
import org.firstinspires.ftc.teamcode.commands.drive.Drive;
import org.firstinspires.ftc.teamcode.commands.intake.ManualBackward;
import org.firstinspires.ftc.teamcode.commands.intake.ManualForward;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpBackward;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpForward;
import org.firstinspires.ftc.teamcode.commands.intake.ManualStop;
import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
@Disabled
public class TwoPlayer extends OpMode {

    private GamepadEx gamepadEx1, gamepadEx2;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();

        RobotHardware.reset(hardwareMap);

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(() -> OuttakeSubsystem.getInstance().resetTimer())
                .whenReleased(OuttakeSubsystem.getInstance().getSeconds() < 0.3 ? new ReadySpecimenGrab() : new ReadySpecimenPlace());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new ManualBackward())
                .whenReleased(new ManualStop());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ManualForward())
                .whenReleased(new ManualStop());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new IntakeDown());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new IntakeRetract());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(new SlurpBackward())
                .whenReleased(new SlurpForward());


        gamepadEx2.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(new GrabSample());
        gamepadEx2.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(new ReadyHighSample());
        gamepadEx2.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(new ReadyLowSample());
        gamepadEx2.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(new ReadySpecimenPlace());

        gamepadEx2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(new OpenClaw())
                .whenReleased(new CloseClaw());
    }

    @Override
    public void start() {
        RobotHardware.getInstance().startPids();
    }

    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
        CommandScheduler.getInstance().schedule(new Drive(
                gamepad1.right_trigger,
                gamepad1.left_trigger,
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                true));
    }

    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
