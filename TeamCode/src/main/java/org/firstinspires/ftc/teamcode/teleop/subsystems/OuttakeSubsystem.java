package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Names;

@Config
public class OuttakeSubsystem extends BaseSubsystem {
    PIDFController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    private boolean grab = false;

    public OuttakeSubsystem(RobotHardware _robotHardware, MultipleTelemetry _telemetry) {
        super(_robotHardware, _telemetry);
        controller = new PIDFController(p, i, d, f);
        register();
    }

    public InstantCommand grab() {
        return new InstantCommand(() -> {
            grab = !grab;
            if (grab) robotHardware.setServoPos(Names.claw ,0.2);
            else robotHardware.setServoPos(Names.claw, 0);
        });
    }

    @Override
    protected void runPeriotic() {
        controller.setPIDF(p, i, d, f);
        int armPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;
        double power = controller.calculate(armPos, target);

        robotHardware.setMotorPower(Names.leftOuttake, power);
        robotHardware.setMotorPower(Names.rightOuttake, power);
        telemetry.addData("Outtake pos", armPos);
        telemetry.addData("Outtake target", target);
    }

    @Override
    public void updateTelemetry() {
    }
}
