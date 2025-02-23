package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.driveSubsystem.Drive;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
public class SoloKota extends OpMode {

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);

        CommandScheduler.getInstance().reset();
    }

    @Override
    public void loop() {
        schedule(new Drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, true));
        telemetry.addData("Robot heading", RobotHardware.getInstance().getHeading());
        CommandScheduler.getInstance().run();
    }

    private void schedule(Command command) {
        CommandScheduler.getInstance().schedule(command);
    }
}
