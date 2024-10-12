package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Names;

@Config
public class OuttakeSubsystem extends BaseSubsystem {
    PIDFController controller;
    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;

    public OuttakeSubsystem(RobotHardware _robotHardware, MultipleTelemetry _telemetry) {
        super(_robotHardware, _telemetry);
        controller = new PIDFController(p, i, d, f);
    }

    public InstantCommand toZero() {
        return new InstantCommand(() -> target=0);
    }

    public InstantCommand bucket1() {
        return new InstantCommand(() -> target=300);
    }

    public InstantCommand bucket2() {
        return new InstantCommand(() -> target=500);
    }

    public InstantCommand lowChamber() {
        return new InstantCommand(() -> target=200);
    }

    public InstantCommand highChamber() {
        return new InstantCommand(() -> target=400);
    }

    public InstantCommand openClaw() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.outtakeGrab, 1));
    }

    public InstantCommand closeClaw() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.outtakeGrab, 0));
    }

    public InstantCommand clawLeft() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.outtakeRotate, 0));
    }

    public InstantCommand clawMid() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.outtakeRotate, 0.5));
    }

    public InstantCommand clawRight() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.outtakeRotate, 1));
    }

    @Override
    protected void runPeriotic() {
        int armPos = (robotHardware.getMotorPos(Names.slideLeft) + robotHardware.getMotorPos(Names.slideRight)) / 2;
        double power = controller.calculate(armPos, target);

        robotHardware.setMotorPower(Names.slideLeft, power);
        robotHardware.setMotorPower(Names.slideRight, power);
        telemetry.addData("Outtake pos", armPos);
        telemetry.addData("Outtake target", target);
    }
}
