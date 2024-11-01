package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.paths.Paths;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.autonomous.selection.AutoSelection;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Autonomous
public class BaseAuto extends OpMode {
    private RobotHardware robotHardware;
    private AutoSelection autoSelection;
    private Paths paths;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        autoSelection = new AutoSelection(gamepad1);
        autoSelection.start();
        paths = new Paths(hardwareMap);
    }

    @Override
    public void init_loop() {
        autoSelection.updateTelem(telemetry);
        telemetry.update();
    }

    @Override
    public void start() {
        paths.setVars(autoSelection);
        autoSelection.interrupt();
        paths.start();
    }

    @Override
    public void loop() {
        paths.updateTelemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        paths.interrupt();
    }
}
