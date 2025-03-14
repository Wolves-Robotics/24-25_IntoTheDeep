package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.driveSubsystem.Drive;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Names;

@TeleOp
public class SoloLogan extends OpMode {

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);

        CommandScheduler.getInstance().reset();

        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(RobotHardware.getInstance()::resetImuYaw);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(IntakeSubsystem.getInstance()::manualOut)
                .whenReleased(IntakeSubsystem.getInstance()::manualStop);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(IntakeSubsystem.getInstance()::manualIn)
                .whenReleased(IntakeSubsystem.getInstance()::manualStop);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(IntakeSubsystem.getInstance()::nextState);
    }

    @Override
    public void start() {
        RobotHardware.getInstance().teleOpServoInit();
        RobotHardware.getInstance().startPDFL();
    }

    @Override
    public void loop() {
        new Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true).schedule();
        RobotHardware.getInstance().update();
        CommandScheduler.getInstance().run();
        telemetry.addData("power", RobotHardware.getInstance().getMotorPower(Names.intakeExtendo));
        telemetry.addData("Intake state", IntakeSubsystem.getInstance().getStateString());
        telemetry.update();
    }
}
