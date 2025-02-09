package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.collections.teleop.TeleOps;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends OpMode {
    private TeleOpSelection teleOpSelection;

    private MultipleTelemetry telemetryA;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);

        teleOpSelection = new TeleOpSelection(gamepad1);

        telemetryA = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void init_loop() {
        teleOpSelection.updateTelemetry(telemetryA);
        telemetryA.update();
    }

    @Override
    public void start() {
        RobotHardware.getInstance().servoInit();
        RobotHardware.getInstance().startPids();
        teleOpSelection.interrupt();

        switch (teleOpSelection.getTeleOp()) {
            case LandonKota:
                break;

            case Kota:
                break;
        }
    }

    @Override
    public void loop() {
        telemetryA.update();
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().interrupt();
    }
}
