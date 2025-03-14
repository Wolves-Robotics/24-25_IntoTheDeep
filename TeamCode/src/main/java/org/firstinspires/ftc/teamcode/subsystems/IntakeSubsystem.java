package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Names;
import org.firstinspires.ftc.teamcode.utils.collections.intake.IntakeStatusState;
import org.firstinspires.ftc.teamcode.utils.utilClasses.PDFL;

public class IntakeSubsystem extends SubsystemBase {
    private static IntakeSubsystem instance;

    private IntakeStatusState intakeStatusState;
    private PDFL pdfl;
    private boolean pdflOn, manualIn;

    private int index;
    private ElapsedTime elapsedTime;

    public static void reset() {
        instance = new IntakeSubsystem();
    }

    public static IntakeSubsystem getInstance() {
        return instance;
    }

    private IntakeSubsystem() {
        pdfl = new PDFL(
                () -> RobotHardware.getInstance().getMotorPos(Names.intakeExtendo),
                (p) -> RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, p));
        pdfl.setCoefficients(IP, ID, IF, IL, I_DEAD);

        intakeStatusState = IntakeStatusState.Done;

        index = 0;
        elapsedTime = new ElapsedTime();

        pdflOn = false;
        manualIn = true;
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
                pdfl.setTarget(0);
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
                pdfl.setTarget(0);
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

    public void reverseIntake() {
        RobotHardware.getInstance().setMotorPower(Names.slurp, -0.5);
    }

    public void forwardIntake() {
        RobotHardware.getInstance().setMotorPower(Names.slurp, 1);
    }

    private void updateIntake() {
        switch (intakeStatusState) {
            case IntakeHover:
                break;
            case Intaking:
                if (index == 0) {
//                    if (RobotHardware.getInstance().isYellow(Names.intakeColor)) {
//                        nextState();
//                        RobotHardware.getInstance().setMotorPower(Names.slurp, 0.5);
//                    }
//                    else if (RobotHardware.getInstance().isRed(Names.intakeColor) || RobotHardware.getInstance().isBlue(Names.intakeColor)) {
//                        index = 1;
//                        RobotHardware.getInstance().setMotorPower(Names.slurp, -0.5);
//                        pdfl.setTarget(50);
//                        elapsedTime.reset();
//                    }
                }
//                else if (index == 1) {
//                    if (elapsedTime.milliseconds() > 500) {
//                        index = 0;
//                        RobotHardware.getInstance().setMotorPower(Names.slurp, 1);
//                    }
//                }
                break;
            case TransferHover:
                break;
            case Transfering:
                if (index == 0) {
                    if (elapsedTime.milliseconds() > 600) {
                        RobotHardware.getInstance().setServoPos(Names.door, 0.5);
                        elapsedTime.reset();
                        index = 1;
                    }
                } else
                if (index == 1) {
                 if (elapsedTime.milliseconds() > 600) {
                     nextState();
                 }
                }
                break;
            case Done:
                break;
        }
    }

    public void updatePDFL() {
        if (pdflOn) {
            pdfl.update();
        } else {
            if (manualIn && pdfl.getPos() < 0) {
                RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, 0);
            } else if (manualIn) {
                RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, -1);
            } else if (!manualIn && pdfl.getPos() > INTAKE_MAX_TARGET) {
                RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, 0);
            } else if (!manualIn) {
                RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, 1);
            }
        }
    }

    public void startPid() {
        pdflOn = true;
    }

    public void manualIn() {
        pdflOn = false;
        manualIn = true;
    }

    public void manualOut() {
        pdflOn = false;
        manualIn = false;
    }

    public void manualStop() {
        pdfl.setTarget(pdfl.getPos());
        pdflOn = true;
    }

    public String getStateString() {
        switch (intakeStatusState) {
            case Done:
                return "Done";
            case IntakeHover:
                return "Intake Hover";
            case Intaking:
                return "Intaking";
            case TransferHover:
                return "Transfer Hover";
            case Transfering:
                return "Transfering";
            default:
                return "Something fucked up";
        }
    }

    public void update() {
        updatePDFL();
        updateIntake();
    }
}
