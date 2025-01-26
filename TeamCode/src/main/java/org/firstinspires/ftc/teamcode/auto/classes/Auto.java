package org.firstinspires.ftc.teamcode.auto.classes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commands.outtake.CloseClaw;
import org.firstinspires.ftc.teamcode.commands.outtake.OpenClaw;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

@Autonomous
public class Auto extends OpMode {
    private AutoSelection autoSelection;
    private BaseAuto auto;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        RobotHardware.getInstance().servoInit();
        autoSelection = new AutoSelection(gamepad1);
    }

    @Override
    public void init_loop() {
        autoSelection.updateTelemetry(telemetry);
        telemetry.update();

        if (gamepad1.a) {
            CommandScheduler.getInstance().schedule(new CloseClaw());
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.b) {
            CommandScheduler.getInstance().schedule(new OpenClaw());
        }
        CommandScheduler.getInstance().run();
    }

    @Override
    public void start() {
        autoSelection.interrupt();

        switch (autoSelection.getAuto()) {
            case fullSample:
                auto = new Sample(autoSelection.getColor());
                break;
            case specimen:
                auto = new Specimen(autoSelection.getColor());
                break;
        }
    }

    @Override
    public void loop() {
        auto.updateTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().interrupt();
        auto.interrupt();
    }
}
