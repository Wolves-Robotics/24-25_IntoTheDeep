package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.utils.Constants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Names;
import org.firstinspires.ftc.teamcode.utils.utilClasses.PDFL;

public class OuttakeSubsystem extends SubsystemBase {
    private static OuttakeSubsystem instance;

    private PDFL pdfl;
    private boolean pdflOn;

    private int index;
    private ElapsedTime elapsedTime;
    private boolean clawOpen;

    public static void reset() {
        instance = new OuttakeSubsystem();
    }

    public static OuttakeSubsystem getInstance() {
        return instance;
    }

    private OuttakeSubsystem() {
        pdfl = new PDFL(
                () -> (RobotHardware.getInstance().getMotorPos(Names.leftOuttake) + RobotHardware.getInstance().getMotorPos(Names.rightOuttake)) / 2,
                (p) -> {RobotHardware.getInstance().setMotorPower(Names.leftOuttake, p); RobotHardware.getInstance().setMotorPower(Names.rightOuttake, p);});
        pdfl.setCoefficients(OP, OD, OF, OL, O_DEAD);
        pdflOn = false;

        index = 0;
        elapsedTime = new ElapsedTime();

        clawOpen = true;
    }

    public void setTarget(int target) {pdfl.setTarget(target);}

    public void switchClaw() {
        if (clawOpen) {
            clawClose();
        } else {
            clawOpen();
        }
    }
    public void clawOpen() {
        RobotHardware.getInstance().setServoPos(Names.claw, 0.35);
        clawOpen = true;
    }
    public void clawClose() {
        RobotHardware.getInstance().setServoPos(Names.claw, 0);
        clawOpen = false;
    }

    public void clawDown() {
        RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.057);
        RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.15);
    }
    public void clawNeutral() {
        RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.2);
        RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.1);
    }
    public void clawSample() {
        RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.45);
        RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.4);
    }
    public void clawSpecimenGrab() {
        RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.74);
        RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.29);
    }
    public void clawSpecimenPlace() {
        RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.6);
        RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.55);
    }

    private void updateOuttake() {

    }

    private void updatePDFL() {
        if (pdflOn) {
            pdfl.update();
        }
    }

    public void update() {
        updateOuttake();
        updatePDFL();
    }

    public void startPDFL() {
        pdflOn = true;
    }
}
