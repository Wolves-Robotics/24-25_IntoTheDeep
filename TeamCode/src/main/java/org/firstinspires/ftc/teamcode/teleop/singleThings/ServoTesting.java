package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.Names;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
@TeleOp
public class ServoTesting extends OpMode {
    public static double p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0, p8=0, p9=0, p10=0, p11=0;
    private RobotHardware robotHardware;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
    }

    @Override
    public void loop() {
        robotHardware.setMotorPower(Names.frontLeft, p1);
        robotHardware.setMotorPower(Names.backLeft, p2);
        robotHardware.setMotorPower(Names.frontRight, p3);
        robotHardware.setMotorPower(Names.backRight, p4);
        robotHardware.setServoPos(Names.door, p5);
        robotHardware.setServoPos(Names.intakePivot, p6);
        robotHardware.setServoPos(Names.intakeArm, p7);
        robotHardware.setServoPos(Names.outtakeArm, p8);
        robotHardware.setServoPos(Names.outtakePivot, p9);
        robotHardware.setServoPos(Names.clawPivot, p10);
        robotHardware.setServoPos(Names.claw, p11);
    }
}