package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.utilClasses.PIDF;

public class IntakeSubsystem extends Thread {
    private static IntakeSubsystem instance = null;
    private PIDController pidController;
    private PIDF pid;
    private int target;
    private boolean pidOn;

    public static void reset() {
        instance = new IntakeSubsystem();
    }

    public static IntakeSubsystem getInstance() {
        return instance;
    }

    private IntakeSubsystem() {
        pidController = new PIDController(Constants.ip, Constants.ii, Constants.id);

        target = 0;
        pidOn = false;

        start();
    }

    public void setTarget(int _target) {target = Math.max(Math.min(_target, Constants.intakeMaxTarget), Constants.intakeMinTarget);}
    public int getTarget() {return target;}

    public void slurpForward() {RobotHardware.getInstance().setMotorPower(Names.slurp, 1);}
    public void slurpBackward() {RobotHardware.getInstance().setMotorPower(Names.slurp, -0.5);}
    public void slurpStop() {RobotHardware.getInstance().setMotorPower(Names.slurp, 0);}

    public void bucketDown() {
        RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.48);
        RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.73);
    }
    public void bucketNeutral() {
        RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.22);
        RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.3);
    }
    public void bucketUp() {
        RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.2);
        RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.01);
    }
    public void bucketHover() {
        RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.45);
        RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.6);
    }

    public void doorClose() {
        RobotHardware.getInstance().setServoPos(Names.door, 0.76);
    }
    public void doorOpen() {
        RobotHardware.getInstance().setServoPos(Names.door, 0.5);
    }

    public void manualForward() {
        pidOn = false;
        RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, 1);
    }
    public void manualBackward() {
        pidOn = false;
        RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, -1);
    }
    public void stopManual() {
        pidOn = true;
        target = RobotHardware.getInstance().getMotorPos(Names.intakeExtendo);
    }

    public void startPid() {
        target = 0;
        pidOn = true;
    }

    public void updatePID() {
        if (pidOn) {
            double power = pidController.calculate(RobotHardware.getInstance().getMotorPos(Names.intakeExtendo), target);
            RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, power);
        }
    }

//    @Override
//    public void run() {
//        while (!currentThread().isInterrupted()) {
//            if (pidOn) {
//                double power = pidController.calculate(RobotHardware.getInstance().getMotorPos(Names.intakeExtendo), target);
//                RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, power);
//            }
//        }
//    }
}
