package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

abstract public class BaseSubsystem extends SubsystemBase {
    RobotHardware robotHardware;
    MultipleTelemetry telemetry;

    public BaseSubsystem(RobotHardware _robotHardware, MultipleTelemetry _telemetry) {
        robotHardware = _robotHardware;
        telemetry = _telemetry;
    }

    abstract protected void runPeriotic();

    @Override
    public void periodic() {
        runPeriotic();
    }
}
