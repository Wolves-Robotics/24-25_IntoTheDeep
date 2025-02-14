package org.firstinspires.ftc.teamcode.commands.outtake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.collections.outtake.OuttakeArmState;

public class setOuttake extends CommandBase {
    private final OuttakeSubsystem outtakeSubsystem;
    private final OuttakeArmState outtakeArmState;
    private final int target;
    private final boolean clawOpen;

    private final ElapsedTime elapsedTime;

    private double index;

    public setOuttake(OuttakeArmState _outtakeArmState, int _target, boolean _clawOpen) {
        outtakeSubsystem = OuttakeSubsystem.getInstance();
        outtakeArmState = _outtakeArmState;
        target = _target;
        clawOpen =_clawOpen;

        elapsedTime = new ElapsedTime();

        addRequirements(outtakeSubsystem);
    }

    @Override
    public void initialize() {
        // used when the outtake is ready to score, and is openning the claw to score
        if ((outtakeSubsystem.getOuttakeArmState() == OuttakeArmState.sampleScoring || outtakeSubsystem.getOuttakeArmState() == OuttakeArmState.specimenScoring)
            && clawOpen) {
            outtakeSubsystem.setClawOpen(clawOpen);
            elapsedTime.reset();
            index = 1;
        // used when the outtake is ready to pick up sample from transfer bucket and is going to pick it up
        } else if ((outtakeSubsystem.getOuttakeArmState() == OuttakeArmState.readyToTransfer && outtakeArmState == OuttakeArmState.downToTransfer)
            && !clawOpen) {
            outtakeSubsystem.setArmState(outtakeArmState);
            outtakeSubsystem.setTarget(target);
            elapsedTime.reset();
            index = 2;
        }
        // default setting position, no special cases, will finish instantly
        else {
            outtakeSubsystem.setArmState(outtakeArmState);
            outtakeSubsystem.setTarget(target);
            outtakeSubsystem.setClawOpen(clawOpen);
            index = 0;
        }
    }

    @Override
    public void execute() {
        // depositing sample or spec
        // waits for claw to open then sets everything else
        if (index == 1) {
            if (elapsedTime.seconds() > 0.1) {
                outtakeSubsystem.setArmState(outtakeArmState);
                outtakeSubsystem.setTarget(target);
            }
        } else

        if (index == 2) {
            if (elapsedTime.seconds() > 0.15) {
                outtakeSubsystem.setClawOpen(clawOpen);
            }
        }
    }

    @Override
    public boolean isFinished() {
        //when index is zero, can finish instantly
        return (index == 0) ||
                // finishes when claw opens and others have been set
                (index == 1 && elapsedTime.seconds() > 0.1) ||
                (index == 2 && elapsedTime.seconds() > 0.15);
    }
}
