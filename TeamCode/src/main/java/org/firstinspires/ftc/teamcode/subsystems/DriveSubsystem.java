package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.util.Constants;

import org.firstinspires.ftc.teamcode.auto.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Names;

public class DriveSubsystem extends SubsystemBase {
    private static DriveSubsystem instance = null;

    public static void reset() {
        instance = new DriveSubsystem();
    }

    public static DriveSubsystem getInstance() {
        return instance;
    }

    private DriveSubsystem() {
        super();

        Constants.setConstants(FConstants.class, LConstants.class);
    }

    public void drive(double x, double y, double rot, boolean fieldCentric) {
        y = y * -1;

        if (fieldCentric) {
            double botHeading = RobotHardware.getInstance().getHeading();

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
            RobotHardware.getInstance().setMotorPower(Names.frontLeft, (rotY + rotX + rot) / denominator);
            RobotHardware.getInstance().setMotorPower(Names.backLeft, (rotY - rotX + rot) / denominator);
            RobotHardware.getInstance().setMotorPower(Names.frontRight, (rotY - rotX - rot) / denominator);
            RobotHardware.getInstance().setMotorPower(Names.backRight, (rotY + rotX - rot) / denominator);
        } else {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
            RobotHardware.getInstance().setMotorPower(Names.frontLeft, (y + x + rot) / denominator);
            RobotHardware.getInstance().setMotorPower(Names.backLeft, (y - x + rot) / denominator);
            RobotHardware.getInstance().setMotorPower(Names.frontRight, (y - x - rot) / denominator);
            RobotHardware.getInstance().setMotorPower(Names.backRight, (y + x - rot) / denominator);
        }
    }
}
