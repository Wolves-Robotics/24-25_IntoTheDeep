package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Names;

@Config
public class IntakeSubsystem extends BaseSubsystem {
    PIDController controller, secondaryController;
    private double p1 = 0, i1 = 0, d1 = 0, p2 = 0, i2 = 0, d2 = 0;
    private int target = 0, maxTarget = 4000;

    public IntakeSubsystem(RobotHardware _robotHardware, MultipleTelemetry _telemetry) {
        super(_robotHardware, _telemetry);
        controller = new PIDController(p1, i1, d1);
        secondaryController = new PIDController(p2, i2, d2);
    }

    public void targetZero() {
        target = 0;
    }

    public int getPos() {
        return robotHardware.getMotorPos(Names.intakeExtendo);
    }

    public InstantCommand setDown() {
        return new InstantCommand(() -> {
            robotHardware.setServoPos(Names.intakeArm, 0.7);
            robotHardware.setServoPos(Names.intakePivot, 0.3);
        });
    }

    public InstantCommand setUp() {
        return new InstantCommand(() -> {
            robotHardware.setServoPos(Names.intakeArm, 0.015);
            robotHardware.setServoPos(Names.intakePivot, 0);
        });
    }

    public InstantCommand setPlace() {
        return new InstantCommand(() -> {
            robotHardware.setServoPos(Names.intakeArm, 0.08);
            robotHardware.setServoPos(Names.intakePivot, 0);
        });
    }

    public InstantCommand openDoor() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.door, 0.4));
    }

    public InstantCommand closeDoor() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.door, 0));
    }

    @Override
    protected void runPeriotic() {
        controller.setPID(p1, i1, d1);
        secondaryController.setPID(p2, i2, d2);

        target = Math.min(target, maxTarget);
        int armPos = robotHardware.getMotorPos(Names.intakeExtendo);

        double power = armPos < maxTarget ?
                controller.calculate(armPos, target) :
                secondaryController.calculate(armPos, target);

        robotHardware.setMotorPower(Names.intakeExtendo, power);
        telemetry.addData("Intake pos", armPos);
        telemetry.addData("Intake target", target);
    }

    @Override
    public void updateTelemetry() {

    }
}
