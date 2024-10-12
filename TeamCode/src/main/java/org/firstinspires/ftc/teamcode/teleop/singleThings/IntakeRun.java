package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class IntakeRun extends OpMode {
    DcMotor motor;
    Servo servo, servo2;

    public static double servoRot = 0.5, servoPos = 0.25;

    @Override
    public void init() {
//        motor = hardwareMap.dcMotor.get("intake");
        servo = hardwareMap.servo.get("clawPivot");
        servo.setPosition(0.5);
        servo2 = hardwareMap.servo.get("claw");
        servo2.setPosition(0);
    }

    @Override
    public void loop() {
//        if (gamepad1.a) motor.setPower(1);
//        else if (gamepad1.b) motor.setPower(-1);
//        else motor.setPower(0);
        if (gamepad1.a) servo.setPosition(0.5);
        if (gamepad1.b) servo.setPosition(1);
        if (gamepad1.x) servo.setPosition(0);
        if (gamepad1.y) servo2.setPosition(servoPos);
        else servo2.setPosition(0);
    }
}
