package org.firstinspires.ftc.teamcode.commands.complex;


import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class DelayAction extends CommandBase {
    private int delay;
    private Runnable action;
    private ElapsedTime elapsedTime;
    private boolean ran = false;

    public DelayAction(int _delay, Runnable _action) {
        delay = _delay;
        action =_action;
    }

    @Override
    public void initialize() {
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void execute() {
        if (elapsedTime.milliseconds() > delay) {
            action.run();
            ran = true;
        }
    }

    @Override
    public boolean isFinished() {
        return ran;
    }
}
