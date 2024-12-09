package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

abstract public class BaseSubsystem extends SubsystemBase {
    RobotHardware robotHardware;
    MultipleTelemetry telemetry;

    public BaseSubsystem(RobotHardware _robotHardware, MultipleTelemetry _telemetry) {
        super();
        robotHardware = _robotHardware;
        telemetry = _telemetry;
    }

    abstract protected void runPeriotic();

    abstract public void updateTelemetry();

    @Override
    public void periodic() {
        runPeriotic();
    }
}
