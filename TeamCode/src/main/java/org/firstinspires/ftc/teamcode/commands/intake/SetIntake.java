package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class SetIntake extends CommandBase {
    private final RobotHardware robotHardware;

    public SetIntake() {
        robotHardware = RobotHardware.getInstance();

    }
}
