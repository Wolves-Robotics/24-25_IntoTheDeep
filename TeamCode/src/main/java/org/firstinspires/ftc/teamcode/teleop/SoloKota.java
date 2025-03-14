package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.driveSubsystem.Drive;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

//@TeleOp
public class SoloKota extends OpMode {

    private boolean defaultB, thing1, thing2, isThing1;

    private GamepadEx gamepadEx1, gamepadEx2;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);

        defaultB = false;
        thing1 = false;
        thing2 = false;

        CommandScheduler.getInstance().reset();

        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        setDefaultThingies();
        setThing1();

    }

    private void setDefaultThingies() {
        gamepadEx1.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(() -> defaultB = true)
                .whenReleased(() -> defaultB = false);

        gamepadEx1.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(() -> {
                    CommandScheduler.getInstance().clearButtons();
                    setDefaultThingies();
                    isThing1 = !isThing1;
                    if (isThing1) {
                        setThing1();
                    } else {
                        setThing2();
                    }
                });
    }

    private void setThing1() {
        gamepadEx1.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(() -> thing1 = true)
                .whenReleased(() -> thing1 = false);
    }

    private void setThing2() {
        gamepadEx1.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(() -> thing2 = true)
                .whenReleased(() -> thing2 = false);
    }

    @Override
    public void start() {
        RobotHardware.getInstance().teleOpServoInit();
    }

    @Override
    public void loop() {
//        schedule(new Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true));
        telemetry.addData("default", defaultB);
        telemetry.addData("thing1", thing1);
        telemetry.addData("thing2", thing2);
        telemetry.update();
        CommandScheduler.getInstance().run();
    }

    private void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }
}
