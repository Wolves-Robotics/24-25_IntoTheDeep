package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Config
@TeleOp
public class ServoTesting extends OpMode {
    ServoImplEx intakePivot, intakeArm, outtakeArm, outtakePivot;
    public static double p1 = 0, p2 = 0, p3 = 0, p4 = 0;
    public static boolean b1 = false, b2 = false, b3 = false, b4 = false;

    @Override
    public void init() {
        intakePivot = hardwareMap.get(ServoImplEx.class, "intakePivot");
        intakeArm = hardwareMap.get(ServoImplEx.class, "intakeArm");
        outtakeArm = hardwareMap.get(ServoImplEx.class, "outtakeArm");
        outtakePivot = hardwareMap.get(ServoImplEx.class, "outtakePivot");
    }

    @Override
    public void loop() {
        intakePivot.setPosition(p1);
        intakePivot.setDirection(b1? Servo.Direction.REVERSE: Servo.Direction.FORWARD);
        intakeArm.setPosition(p2);
        intakeArm.setDirection(b2? Servo.Direction.REVERSE: Servo.Direction.FORWARD);
        outtakeArm.setPosition(p3);
        outtakeArm.setDirection(b3? Servo.Direction.REVERSE: Servo.Direction.FORWARD);
        outtakePivot.setPosition(p4);
        outtakePivot.setDirection(b4? Servo.Direction.REVERSE: Servo.Direction.FORWARD);
    }
}
