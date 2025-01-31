package org.firstinspires.ftc.teamcode.commands.complex.sample;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.commands.intake.SlurpBackward;
import org.firstinspires.ftc.teamcode.commands.intake.SlurpForward;

public class SlurpExpel extends SequentialCommandGroup {
    public SlurpExpel() {
        super(
                new SlurpBackward(),
                new WaitCommand(300),
                new SlurpForward()
        );
    }
}
