package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Names;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Autonomous
@Config
public class BasicAhhAuto extends OpMode {
    private RobotHardware robotHardware;
    private PIDController intakePID, outtakePID;
    private double ip=0.014, ii=0.15, id=0.00081, op=0.04, oi=0.015, od=0.0005, of=0.05;
    private double iTarget=0, oTarget=0;
    private ElapsedTime elapsedTime;
    private Auto auto;
    public static int startOuttake=300, endOuttake=0;
    public static double forwardTime=1., pauseTime=0.5, backwardTime=0.2, placeTime=0.2,
                            forwardMove=-0.7, backwardMove = 0.2;

    public enum Auto {
        Start,
        Forward,
        Pause,
        Backwards,
        Place,
        Done
    }

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.setDaemon(true);
        robotHardware.start();
        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);
        elapsedTime = new ElapsedTime();
        auto = Auto.Start;
        robotHardware.setServoPos(Names.door, 0);
        robotHardware.setServoPos(Names.intakeArm, 0.0);
        robotHardware.setServoPos(Names.intakePivot, 0);
        robotHardware.setServoPos(Names.outtakeArm, 0.4);
        robotHardware.setServoPos(Names.outtakePivot, 0.6);
        robotHardware.setServoPos(Names.clawPivot, 1);
        robotHardware.setServoPos(Names.claw, 0.2);
    }

    @Override
    public void start() {
        robotHardware.setServoPos(Names.door, 0);
        robotHardware.setServoPos(Names.intakeArm, 0.0);
        robotHardware.setServoPos(Names.intakePivot, 0);
        robotHardware.setServoPos(Names.outtakeArm, 0.4);
        robotHardware.setServoPos(Names.outtakePivot, 0.6);
        robotHardware.setServoPos(Names.clawPivot, 0.78);
        robotHardware.setServoPos(Names.claw, 0.2);
        oTarget = startOuttake;
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    @Override
    public void loop() {
        switch (auto) {
            case Start:
                move(0, forwardMove, 0);
                robotHardware.setServoPos(Names.intakePivot, 0);
                elapsedTime.reset();
                auto = Auto.Forward;
                break;
            case Forward:
                if (elapsedTime.seconds() > forwardTime) {
                    move(0, 0, 0);
                    elapsedTime.reset();
                    auto = Auto.Pause;
                }
                break;
            case Pause:
                if (elapsedTime.seconds() > pauseTime) {
                    move(0, backwardMove, 0);
                    elapsedTime.reset();
                    auto = Auto.Backwards;
                }
                break;
            case Backwards:
                if (elapsedTime.seconds() > backwardTime) {
                    move(0, 0, 0);
                    oTarget = endOuttake;
                    elapsedTime.reset();
                    auto = Auto.Place;
                }
                break;
            case Place:
                if (elapsedTime.seconds() > placeTime) {
                    robotHardware.setServoPos(Names.claw, 0);
                    oTarget = 50;
                    robotHardware.servoInit();
                    robotHardware.setServoPos(Names.intakeArm, 0.6);
                    auto = Auto.Done;
                }
                break;
            case Done:
                break;
            default:
                break;
        }

        int iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        int oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;

        robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));
        double oPower = outtakePID.calculate(oArmPos, (int) oTarget);
        robotHardware.setMotorPower(Names.leftOuttake, oPower);
        robotHardware.setMotorPower(Names.rightOuttake, oPower);
    }

    private void move(double x, double y, double rot) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
        double frontLeftPower = (y + x + rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        robotHardware.setMotorPower(Names.frontLeft, frontLeftPower);
        robotHardware.setMotorPower(Names.frontRight, frontRightPower);
        robotHardware.setMotorPower(Names.backLeft, backLeftPower);
        robotHardware.setMotorPower(Names.backRight, backRightPower);
    }
}
