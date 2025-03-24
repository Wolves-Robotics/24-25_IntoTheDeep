package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.collections.teleop.IntakeStatusState;
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
    private IntakeStatusState intakeStatusState;
    private int index;
    private ElapsedTime elapsedTime;

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


        index = 0;
        elapsedTime = new ElapsedTime();

        intakeStatusState = IntakeStatusState.Done;

        start();
    }

    public void changeState(IntakeStatusState _intakeStatusState) {
        intakeStatusState = _intakeStatusState;

        index = 0;

        elapsedTime.reset();

        switch (intakeStatusState) {
            case IntakeHover:
                RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.55);
                RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.45);
                break;
            case Intaking:
                RobotHardware.getInstance().setMotorPower(Names.slurp, 1);
                RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.68);
                RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.48);
                break;
            case TransferHover:
                RobotHardware.getInstance().setMotorPower(Names.slurp, 0.5);
                RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.55);
                RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.45);
                target = 0;
                break;
            case Transfering:
                RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.2);
                RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.01);
                break;
            case Done:
                RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.25);
                RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.22);
                RobotHardware.getInstance().setServoPos(Names.door, 0.76);
                RobotHardware.getInstance().setMotorPower(Names.slurp, 0);
                target = 0;
                break;
        }
    }

    public void nextState() {
        switch (intakeStatusState) {
            case IntakeHover:
                changeState(IntakeStatusState.Intaking);
                break;
            case Intaking:
                changeState(IntakeStatusState.TransferHover);
                break;
            case TransferHover:
                changeState(IntakeStatusState.Transfering);
                break;
            case Transfering:
                changeState(IntakeStatusState.Done);
                break;
            case Done:
                changeState(IntakeStatusState.IntakeHover);
                break;
        }
    }


    public void setTarget(int _target) {target = Math.max(Math.min(_target, Constants.intakeMaxTarget), Constants.intakeMinTarget);}
    public int getTarget() {return target;}

    public void slurpForward() {RobotHardware.getInstance().setMotorPower(Names.slurp, 1);}
    public void slurpBackward() {RobotHardware.getInstance().setMotorPower(Names.slurp, -0.5);}
    public void slurpStop() {RobotHardware.getInstance().setMotorPower(Names.slurp, 0);}

    public void bucketDown() {
        RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.68);
        RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.48);
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
        RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.55);
        RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.45);
    }

    public void doorClose() {
        RobotHardware.getInstance().setServoPos(Names.door, 0.76);
    }
    public void doorOpen() {
        RobotHardware.getInstance().setServoPos(Names.door, 0.4);
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

    public void updateIntake() {
        if (intakeStatusState == IntakeStatusState.Transfering) {
            if (index == 0 && elapsedTime.milliseconds() > 600) {
                RobotHardware.getInstance().setServoPos(Names.door, 0.5);
                elapsedTime.reset();
                index = 1;
            } else if (index == 1 && elapsedTime.milliseconds() > 600) {
                nextState();
            }
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
