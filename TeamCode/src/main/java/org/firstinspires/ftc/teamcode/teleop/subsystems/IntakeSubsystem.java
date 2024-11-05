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
    public static double p1 = 0, i1 = 0, d1 = 0, p2 = 0, i2 = 0, d2 = 0;
    public static int target = 0, maxTarget = 4000;

    public IntakeSubsystem(RobotHardware _robotHardware, MultipleTelemetry _telemetry) {
        super(_robotHardware, _telemetry);
        controller = new PIDController(p1, i1, d1);
        secondaryController = new PIDController(p2, i2, d2);
    }

    public InstantCommand toZero() {
        return new InstantCommand(() -> target=0);
    }

    public InstantCommand increaseTarget() {
        return new InstantCommand(() -> target=100/50);
    }

    public InstantCommand decreaseTarget() {
        return new InstantCommand(() -> target=100/50);
    }

    public InstantCommand runIntakeMotor() {
        return new InstantCommand(() -> robotHardware.setMotorPower(Names.slurp, 1));
    }

    public InstantCommand stopIntakeMotor() {
        return new InstantCommand(() -> robotHardware.setMotorPower(Names.slurp, 0));
    }

    public InstantCommand openColorServo() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.door, 1));
    }

    public InstantCommand closeColorServo() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.door, 0));
    }

    public InstantCommand lowerArmPitch() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.armPitch, 1));
    }

    public InstantCommand raiseArmPitch() {
        return new InstantCommand(() -> robotHardware.setServoPos(Names.armPitch, 0));
    }

    public SequentialCommandGroup resetIntake() {
        return new SequentialCommandGroup(new ParallelCommandGroup(raiseArmPitch(), stopIntakeMotor()), toZero());
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
}
