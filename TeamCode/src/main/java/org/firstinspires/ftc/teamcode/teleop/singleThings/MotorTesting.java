package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Names;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class MotorTesting extends OpMode {
    private RobotHardware robotHardware;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.setDaemon(true);
        robotHardware.start();
    }

    @Override
    public void loop() {
//        double x = gamepad1.left_stick_x;
//        double y = -gamepad1.left_stick_y;
//        double rot = gamepad1.right_stick_x;
//
//        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
//        double frontLeftPower = (y + x + rot) / denominator;
//        double backLeftPower = (y - x + rot) / denominator;
//        double frontRightPower = (y - x - rot) / denominator;
//        double backRightPower = (y + x - rot) / denominator;
//
//        robotHardware.setMotorPower(Names.frontLeft, frontLeftPower);
//        robotHardware.setMotorPower(Names.frontRight, frontRightPower);
//        robotHardware.setMotorPower(Names.backLeft, backLeftPower);
//        robotHardware.setMotorPower(Names.backRight, backRightPower);
//
//        telemetry.addData("front left", frontLeftPower);
//        telemetry.addData("front right", frontRightPower);
//        telemetry.addData("back left", backLeftPower);
//        telemetry.addData("back right", backRightPower);
//        telemetry.update();

        if (gamepad1.a) robotHardware.setMotorPower(Names.slideLeft, 0.25);
        else robotHardware.setMotorPower(Names.slideLeft, 0);
        if (gamepad1.b) robotHardware.setMotorPower(Names.rightOuttake, 0.25);
        else robotHardware.setMotorPower(Names.rightOuttake, 0);
        if (gamepad1.x) robotHardware.setMotorPower(Names.intakeExtendo, 0.25);
        else robotHardware.setMotorPower(Names.intakeExtendo, 0);
        if (gamepad1.y) robotHardware.setMotorPower(Names.slurp, 0.25);
        else robotHardware.setMotorPower(Names.slurp, 0);
    }
}
