package org.firstinspires.ftc.teamcode.teleop.subsystems;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.hardware.Names;

import java.util.function.DoubleSupplier;

public class DriveSubsystem extends BaseSubsystem {

    public DriveSubsystem(RobotHardware _robotHardware, MultipleTelemetry _telemetry) {
        super(_robotHardware, _telemetry);
    }

    public InstantCommand driveCommand(double x, double y, double rot, boolean robotCentric) {
        return new InstantCommand(
                () -> drive(x, y, rot, robotCentric)
        );
    }

    public InstantCommand driveCommand(double x, double y, double rotLeft, double rotRight, boolean robotCentric) {
        return driveCommand(x, y, -rotLeft + rotRight, robotCentric);
    }

    private void drive(double x, double y, double rot, boolean robotCentric) {
        if (robotCentric) {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
            double frontLeftPower = (y + x + rot) / denominator;
            double backLeftPower = (y - x + rot) / denominator;
            double frontRightPower = (y - x - rot) / denominator;
            double backRightPower = (y + x - rot) / denominator;

            robotHardware.setMotorPower(Names.frontLeft, frontLeftPower);
            robotHardware.setMotorPower(Names.frontRight, frontRightPower);
            robotHardware.setMotorPower(Names.backLeft, backLeftPower);
            robotHardware.setMotorPower(Names.backRight, backRightPower);
        } else {
            double botHeading = robotHardware.getImuAngles().getYaw(AngleUnit.RADIANS);
            double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) *1.1;
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
            double frontLeftPower = (rotY + rotX + rot) / denominator;
            double backLeftPower = (rotY - rotX + rot) / denominator;
            double frontRightPower = (rotY - rotX - rot) / denominator;
            double backRightPower = (rotY + rotX - rot) / denominator;

            robotHardware.setMotorPower(Names.frontLeft, frontLeftPower);
            robotHardware.setMotorPower(Names.frontRight, frontRightPower);
            robotHardware.setMotorPower(Names.backLeft, backLeftPower);
            robotHardware.setMotorPower(Names.backRight, backRightPower);
        }
    }

    @Override
    protected void runPeriotic() {}

    @Override
    public void updateTelemetry() {

    }
}
