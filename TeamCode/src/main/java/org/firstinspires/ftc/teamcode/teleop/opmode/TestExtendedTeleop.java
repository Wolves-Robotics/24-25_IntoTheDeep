package org.firstinspires.ftc.teamcode.teleop.opmode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TestExtendedTeleop extends BaseTeleop{
    @Override
    protected void setDefaultDrive() {
        driveSubsystem.setDefaultCommand(driveSubsystem.driveCommand(
                gamepadEx1.getLeftX(),
                gamepadEx1.getLeftY(),
                gamepadEx1.getRightX(),
                false
        ));
    }

    @Override
    protected void setCommands() {

    }
}
