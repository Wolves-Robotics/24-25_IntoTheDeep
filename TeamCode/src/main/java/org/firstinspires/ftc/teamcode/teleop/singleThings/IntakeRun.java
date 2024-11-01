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
    Servo clawRot, claw, door;

    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get("intake");
//        clawRot = hardwareMap.servo.get("clawPivot");
//        clawRot.setPosition(0.5);
//        claw = hardwareMap.servo.get("claw");
//        claw.setPosition(0);
        door = hardwareMap.servo.get("door");
        door.setPosition(0.07);
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
//        if (gamepad1.a) clawRot.setPosition(0.5);
//        if (gamepad1.b) clawRot.setPosition(1);
//        if (gamepad1.x) clawRot.setPosition(0);
//        if (gamepad1.y) claw.setPosition(0.25);
//        else claw.setPosition(0);
        if (gamepad1.a) {
            door.setPosition(0.4);
        }
        if (gamepad1.b) {
            door.setPosition(0.07);
        }
    }
}
