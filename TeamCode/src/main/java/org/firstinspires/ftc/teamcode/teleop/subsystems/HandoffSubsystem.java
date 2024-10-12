package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

public class HandoffSubsystem extends BaseSubsystem {
    private IntakeSubsystem intakeSubsystem;
    private OuttakeSubsystem outtakeSubsystem;

    public HandoffSubsystem(RobotHardware _robotHardware,
                            MultipleTelemetry _telemetry,
                            IntakeSubsystem _intakeSubsystem,
                            OuttakeSubsystem _outtakeSubsystem) {
        super(_robotHardware, _telemetry);
        intakeSubsystem = _intakeSubsystem;
        outtakeSubsystem = _outtakeSubsystem;
    }

    @Override
    protected void runPeriotic() {

    }
}
