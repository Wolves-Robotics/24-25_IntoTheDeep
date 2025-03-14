package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.driveSubsystem.Drive;
import org.firstinspires.ftc.teamcode.commands.outtake.GrabSample;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakesSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.intake.IntakeStatusState;

@TeleOp
public class TempKota extends OpMode {
    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);

        CommandScheduler.getInstance().reset();

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(RobotHardware.getInstance()::resetImuYaw);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(IntakeSubsystem.getInstance()::manualOut)
                .whenReleased(IntakeSubsystem.getInstance()::manualStop);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(IntakeSubsystem.getInstance()::manualIn)
                .whenReleased(IntakeSubsystem.getInstance()::manualStop);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(IntakeSubsystem.getInstance()::nextState);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> IntakeSubsystem.getInstance().changeState(IntakeStatusState.Done));

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(new GrabSample());

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> {
                    OuttakesSubsystem.getInstance().clawNeutral();
                    OuttakesSubsystem.getInstance().clawOpen();
                    OuttakesSubsystem.getInstance().setTarget(0);
                });

        gamepadEx1.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(() -> {
                    OuttakesSubsystem.getInstance().clawSample();
                    OuttakesSubsystem.getInstance().setTarget(740);
                });

        gamepadEx1.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(OuttakesSubsystem.getInstance()::switchClaw);
    }

    @Override
    public void start() {
        RobotHardware.getInstance().teleOpServoInit();
        RobotHardware.getInstance().startPDFL();
    }

    @Override
    public void loop() {
        if (gamepad1.left_trigger > 0.7) {
            OuttakesSubsystem.getInstance().clawSample();
            OuttakesSubsystem.getInstance().setTarget(300);
        }

        new Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true).schedule();
        RobotHardware.getInstance().update();
        CommandScheduler.getInstance().run();
        telemetry.addData("Rot", Math.toDegrees(RobotHardware.getInstance().getHeading()));
        telemetry.update();
    }
}
