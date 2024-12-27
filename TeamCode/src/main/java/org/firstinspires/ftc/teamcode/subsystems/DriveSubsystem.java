package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class DriveSubsystem extends Thread{
    private static DriveSubsystem instance = null;
    private RobotHardware robotHardware;

    public static DriveSubsystem getInstance(RobotHardware robotHardware) {
        if (instance == null) {
            instance = new DriveSubsystem(robotHardware);
        }
        return instance;
    }

    public static DriveSubsystem getInstance() {
        return instance;
    }

    private DriveSubsystem(RobotHardware _robotHardware) {
        robotHardware = _robotHardware;
    }

    public void drive(double x, double y, double rot, boolean robotCentric) {
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

    public void drive(double x1, double x2, double y, double rot, boolean robotCentric) {
        drive(x1-x2, y, rot, robotCentric);
    }

    @Override
    public void run() {

    }
}
