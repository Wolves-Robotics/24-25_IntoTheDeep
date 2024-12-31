package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.utilClasses.PIDF;

public class IntakeSubsystem extends Thread {
    private static IntakeSubsystem instance = null;
    private RobotHardware robotHardware;
    private PIDController pidController;
    private PIDF pid;
    private int target = 0;
    private boolean pidOn=false;

    public static IntakeSubsystem getInstance(RobotHardware robotHardware) {
        if (instance == null) {
            instance = new IntakeSubsystem(robotHardware);
        }
        return instance;
    }

    public static IntakeSubsystem getInstance() {
        return instance;
    }

    private IntakeSubsystem(RobotHardware _robotHardware) {
        robotHardware = _robotHardware;
        pidController = new PIDController(Constants.ip, Constants.ii, Constants.id);
        pid = new PIDF(
                Constants.ip,
                Constants.ii,
                Constants.id,
                () -> robotHardware.getMotorPos(Names.intakeExtendo),
                x -> robotHardware.setMotorPower(Names.intakeExtendo, x)
        );
//        pid.start();
        setDaemon(true);
        start();
    }

    public void setTarget(int _target) {target = Math.max(Math.min(_target, Constants.intakeMaxTarget), Constants.intakeMinTarget);}
    public int getTarget() {return target;}

    public void slurpForward() {robotHardware.setMotorPower(Names.slurp, 1);}
    public void slurpBackward() {robotHardware.setMotorPower(Names.slurp, -1);}
    public void slurpStop() {robotHardware.setMotorPower(Names.slurp, 0);}

    public void bucketDown() {
        robotHardware.setServoPos(Names.intakePivot, 0.45);
        robotHardware.setServoPos(Names.intakeArm, 0.73);
    }
    public void bucketNeutral() {
        robotHardware.setServoPos(Names.intakePivot, 0.22);
        robotHardware.setServoPos(Names.intakeArm, 0.3);
    }
    public void bucketUp() {
        robotHardware.setServoPos(Names.intakePivot, 0.22);
        robotHardware.setServoPos(Names.intakeArm, 0.05);
    }

    public void doorClose() {
        robotHardware.setServoPos(Names.door, 0.73);
    }
    public void doorOpen() {
        robotHardware.setServoPos(Names.door, 0.4);
    }

    public void manualForward() {
        pidOn = false;
        robotHardware.setMotorPower(Names.intakeExtendo, 1);
    }
    public void manualBackward() {
        pidOn = false;
        robotHardware.setMotorPower(Names.intakeExtendo, -1);
    }
    public void stopManual() {
        pidOn = true;
        target = robotHardware.getMotorPos(Names.intakeExtendo);
    }

    public void startPid() {
        target = 0;
        pidOn = true;
    }
    public void stopPid() {
        pidOn = false;
    }


    @Override
    public void run() {
        while (!currentThread().isInterrupted()) {
            if (pidOn) {
                double power = pidController.calculate(robotHardware.getMotorPos(Names.intakeExtendo), target);
                robotHardware.setMotorPower(Names.intakeExtendo, power);
            }
        }
    }
}
