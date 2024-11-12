package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.Names;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@Config
@TeleOp
public class ServoTesting extends OpMode {
    public static double p1=0, p2=0, p3=0, p4=0.05, p5=0, p6=0.5, p7=0;
    private RobotHardware robotHardware;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.setDaemon(true);
        robotHardware.start();
    }

    @Override
    public void loop() {
        robotHardware.setServoPos(Names.intakeArm, p1);
        robotHardware.setServoPos(Names.intakePivot, p2);
        robotHardware.setServoPos(Names.door, p3);
        robotHardware.setServoPos(Names.outtakeArm, p4);
        robotHardware.setServoPos(Names.outtakePivot, p5);
        robotHardware.setServoPos(Names.clawPivot, p6);
        robotHardware.setServoPos(Names.claw, p7);
    }
}