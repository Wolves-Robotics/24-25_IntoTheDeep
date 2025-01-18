package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.pedro.follower.Follower;
import org.firstinspires.ftc.teamcode.auto.pedro.localization.Pose;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.auto.pedro.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class DriveSubsystem extends Thread{
    private static DriveSubsystem instance = null;
    private RobotHardware robotHardware;
    private Follower follower;

    public static void reset() {
        instance = new DriveSubsystem();
    }

    public static DriveSubsystem getInstance() {
        return instance;
    }

    private DriveSubsystem() {
        robotHardware = RobotHardware.getInstance();
        start();
    }

    public void setFollower(HardwareMap hardwareMap, Pose pose) {
        follower = new Follower(hardwareMap);
        follower.setStartingPose(pose);
    }

    public void followPath(Path path) {
        follower.followPath(path);
    }

    public void followPath(Path path, boolean holdEnd) {
        follower.followPath(path, holdEnd);
    }

    public void followPath(PathChain pathChain) {
        follower.followPath(pathChain);
    }

    public void followPath(PathChain pathChain, boolean holdEnd) {
        follower.followPath(pathChain, holdEnd);
    }

    public boolean atParametricEnd() {
        return follower.atParametricEnd();
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
        drive(x2-x1, y, rot, robotCentric);
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            if (follower != null) follower.update();
        }
    }
}
