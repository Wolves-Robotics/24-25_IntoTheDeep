package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Names;
import org.firstinspires.ftc.teamcode.utils.collections.outtake.OuttakeArmState;

public class OuttakeSubsystem extends SubsystemBase {
    private static OuttakeSubsystem instance = null;
    private PIDController pidController;
    private int target;
    private boolean pidOn;
    private double power, pos;
    private OuttakeArmState outtakeArmState;

    public static void reset() {
        instance = new OuttakeSubsystem();
    }

    public static OuttakeSubsystem getInstance() {
        return instance;
    }

    private OuttakeSubsystem() {
        super();
        pidController = new PIDController(OP, OI, OD);

        target = 0;
        pidOn = false;
    }

    public void setTarget(int _target) {
        target = Range.clip(_target, OUTTAKE_MIN_TARGET, OUTTAKE_MAX_TARGET);
    }
    public int getTarget() {
        return target;
    }
    public double getPos() {
        return pos;
    }

    public void setClawOpen(boolean open) {
        if (open) RobotHardware.getInstance().setServoPos(Names.claw, CLAW_OPEN);
        else RobotHardware.getInstance().setServoPos(Names.claw, CLAW_CLOSE);
    }

    public void setHangOut(boolean out) {
        if (out) {
            RobotHardware.getInstance().setServoPos(Names.leftHang, 0.4);
            RobotHardware.getInstance().setServoPos(Names.rightHang, 0.4);
        } else {
            RobotHardware.getInstance().setServoPos(Names.leftHang, 0.1);
            RobotHardware.getInstance().setServoPos(Names.rightHang, 0.1);
        }
    }

    public void setArmState(OuttakeArmState _outtakeArmState) {
        outtakeArmState = _outtakeArmState;
        switch (outtakeArmState) {
            case autoStart:
                RobotHardware.getInstance().setServoPos(Names.outtakeArm, AUTO_START_ARM_POS);
                RobotHardware.getInstance().setServoPos(Names.outtakePivot, AUTO_START_PIVOT_POS);
                break;
            case readyToTransfer:
                RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.2);
                RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.1);
                break;
            case downToTransfer:
                RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.05);
                RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.15);
                break;
            case sampleScoring:
                RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.50);
                RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.38);
                break;
            case specimenGrabbing:
                RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.74);
                RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.29);
                break;
            case specimenScoring:
                RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.6);
                RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.55);
                break;
        }
    }

    public OuttakeArmState getOuttakeArmState() {
        return outtakeArmState;
    }

    public void startPid() {
        target = 0;
        pidOn = true;
    }
    public void stopPid() {
        pidOn = false;
    }

    @Override
    public void periodic() {
        if (pidOn) {
            pos = (RobotHardware.getInstance().getMotorPos(Names.leftOuttake) + RobotHardware.getInstance().getMotorPos(Names.rightOuttake)) / 2.;
            power = pidController.calculate(pos, target) + OF;
            RobotHardware.getInstance().setMotorPower(Names.leftOuttake, power);
            RobotHardware.getInstance().setMotorPower(Names.rightOuttake, power);
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Outtake target", target);
        telemetry.addData("Outtake pos", pos);
        telemetry.addData("Outtake power", power);
    }
}