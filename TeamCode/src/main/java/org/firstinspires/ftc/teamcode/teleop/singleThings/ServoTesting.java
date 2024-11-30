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
    public static double p1=0, p2=0, p3=0, p4=0.05, p5=0, p6=0.5, p7=0;
    private RobotHardware robotHardware;
    private Servo IDK1 = null;
    private Servo IDK2 = null;
    private Servo IDK3 = null;
    private Servo IDK4 = null;
    private Servo outtakeArm = null;
    private Servo intakeArm = null;
    private DcMotor suck = null;

    @Override
    public void init() {
        IDK1 = hardwareMap.get(Servo.class, "IDK1");
        IDK2 = hardwareMap.get(Servo.class, "IDK2");
        IDK3 = hardwareMap.get(Servo.class, "IDK3");
        IDK4 = hardwareMap.get(Servo.class, "IDK4");
        outtakeArm = hardwareMap.get(Servo.class, "outtakeArm");
        intakeArm = hardwareMap.get(Servo.class, "intakeArm");
        suck = hardwareMap.get(DcMotor.class, "slurp");
    }

    @Override
    public void loop() {
        IDK1.setPosition(p1);
        IDK2.setPosition(p2);
        IDK3.setPosition(p3);
        IDK4.setPosition(p4);
        outtakeArm.setPosition(p5);
        intakeArm.setPosition(p6);
        suck.setPower(p7);

    }
}