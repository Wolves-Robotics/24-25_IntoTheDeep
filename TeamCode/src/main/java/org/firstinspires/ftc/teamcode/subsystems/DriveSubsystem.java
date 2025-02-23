package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.FConstants;
import org.firstinspires.ftc.teamcode.auto.pedro.constants.LConstants;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class DriveSubsystem extends Thread{
    private static DriveSubsystem instance = null;
    private Follower follower;

    public static void reset() {
        instance = new DriveSubsystem();
    }

    public static DriveSubsystem getInstance() {
        return instance;
    }

    private DriveSubsystem() {

        follower = null;

        start();
    }

    public void setFollower(Pose pose) {
        Constants.setConstants(FConstants.class, LConstants.class);
        follower = new Follower(RobotHardware.getInstance().getHardwareMap());
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

    public void telemetryDebug(Telemetry telemetry) {
        follower.telemetryDebug(telemetry);
    }

    public boolean atParametricEnd() {
        return !follower.isBusy();
    }

    public Follower getFollower() {
        return follower;
    }

    public double getYPos() {
        return follower.getPose().getY();
    }

    public double getXPos() {
        return follower.getPose().getX();
    }

    public double getHeadimg() {
        return follower.getPose().getHeading();
    }

    public void drive(double x, double y, double rot, boolean robotCentric) {
        if (robotCentric) {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
            double frontLeftPower = (y + x + rot) / denominator;
            double backLeftPower = (y - x + rot) / denominator;
            double frontRightPower = (y - x - rot) / denominator;
            double backRightPower = (y + x - rot) / denominator;

            RobotHardware.getInstance().setMotorPower(Names.frontLeft, frontLeftPower);
            RobotHardware.getInstance().setMotorPower(Names.frontRight, frontRightPower);
            RobotHardware.getInstance().setMotorPower(Names.backLeft, backLeftPower);
            RobotHardware.getInstance().setMotorPower(Names.backRight, backRightPower);
        } else { // Field centric
            double botHeading = RobotHardware.getInstance().getImuAngles().getYaw(AngleUnit.RADIANS);
            double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) *1.1;
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
            double frontLeftPower = (rotY + rotX + rot) / denominator;
            double backLeftPower = (rotY - rotX + rot) / denominator;
            double frontRightPower = (rotY - rotX - rot) / denominator;
            double backRightPower = (rotY + rotX - rot) / denominator;

            RobotHardware.getInstance().setMotorPower(Names.frontLeft, frontLeftPower);
            RobotHardware.getInstance().setMotorPower(Names.frontRight, frontRightPower);
            RobotHardware.getInstance().setMotorPower(Names.backLeft, backLeftPower);
            RobotHardware.getInstance().setMotorPower(Names.backRight, backRightPower);
        }
    }

    public void drive(double x1, double x2, double y, double rot, boolean robotCentric) {
        drive(x2-x1, y, rot, robotCentric);
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            if (follower != null) {
                follower.update();
                follower.drawOnDashBoard();
            }
        }
    }
}
