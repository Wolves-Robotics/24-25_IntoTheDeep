package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.BooleanSupplier;

public class AHHHHHHHHHHH extends CommandBase {

    private ElapsedTime elapsedTime;
    private BooleanSupplier override;
    private int delay;

    public AHHHHHHHHHHH(BooleanSupplier _override, int _delay) {
        override = _override;
        delay = _delay;

        elapsedTime = new ElapsedTime();
    }

    @Override
    public boolean isFinished() {
        return override.getAsBoolean() ||
                (elapsedTime.milliseconds() > delay);
    }
}
