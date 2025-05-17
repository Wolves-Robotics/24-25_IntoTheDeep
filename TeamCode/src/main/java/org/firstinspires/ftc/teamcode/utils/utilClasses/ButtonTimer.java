package org.firstinspires.ftc.teamcode.utils.utilClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

public class ButtonTimer {
    private ElapsedTime elapsedTime;
    private int delay;

    public ButtonTimer(int _delay) {
        elapsedTime = new ElapsedTime();
        delay = _delay;
    }

    public void setDelay(int _delay) {
        delay = _delay;
    }

    public boolean isReady() {
        if (elapsedTime.milliseconds() > delay) {
            elapsedTime.reset();
            return true;
        }
        return false;
    }
}
