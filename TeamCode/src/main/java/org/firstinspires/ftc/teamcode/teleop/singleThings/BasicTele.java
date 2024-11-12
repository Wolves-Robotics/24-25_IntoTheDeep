package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Names;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class BasicTele extends OpMode {
    private RobotHardware robotHardware;
    private ElapsedTime clawTime, doorTime;
    private boolean grab=false, door=true;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.setDaemon(true);
        robotHardware.start();
        clawTime = new ElapsedTime();
        doorTime = new ElapsedTime();
    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            robotHardware.setServoPos(Names.intakeArm, 0.73);
            robotHardware.setServoPos(Names.intakePivot, 0.3);
            robotHardware.setServoPos(Names.outtakeArm, 0.02);
            robotHardware.setServoPos(Names.outtakePivot, 0);
        }
        if (gamepad1.b){
            robotHardware.setServoPos(Names.intakeArm, 0.015);
            robotHardware.setServoPos(Names.intakePivot, 0);
            robotHardware.setServoPos(Names.outtakeArm, 0.02);
            robotHardware.setServoPos(Names.outtakePivot, 0);
        }
        if (gamepad1.a) robotHardware.setMotorPower(Names.intakeExtendo, -1);
        else if (gamepad1.y) robotHardware.setMotorPower(Names.intakeExtendo, 1);
        else robotHardware.setMotorPower(Names.intakeExtendo, 0);

        if(gamepad1.right_bumper && doorTime.seconds() > 0.25){
            doorTime.reset();
            door = !door;
            if (door) robotHardware.setServoPos(Names.door ,0);
            else robotHardware.setServoPos(Names.door, 0.4);
        }
        if(gamepad1.left_bumper){
            robotHardware.setServoPos(Names.intakeArm, 0.08);
            robotHardware.setServoPos(Names.intakePivot, 0);
            robotHardware.setServoPos(Names.outtakeArm, 0.02);
            robotHardware.setServoPos(Names.outtakePivot, 0);
        }
        if(gamepad2.left_bumper && clawTime.seconds() > 0.25) {
            clawTime.reset();
            grab = !grab;
            if (grab) robotHardware.setServoPos(Names.claw ,0.2);
            else robotHardware.setServoPos(Names.claw, 0);
        }
        if(gamepad2.left_stick_button) {
            robotHardware.setServoPos(Names.outtakeArm, 0.0);
            robotHardware.setServoPos(Names.outtakePivot, 0.04);
        }
        if(gamepad2.right_stick_button) {
            robotHardware.setServoPos(Names.outtakeArm, 0.35);
            robotHardware.setServoPos(Names.outtakePivot, 0.45);
        }

        if(gamepad2.b) {
            robotHardware.setMotorPower(Names.leftOuttake, 0);
            robotHardware.setMotorPower(Names.rightOuttake, 0);
        }
        if(gamepad2.x) {
            robotHardware.setMotorPower(Names.leftOuttake, 0.25);
            robotHardware.setMotorPower(Names.rightOuttake, 0.25);
        }
        if(gamepad2.a) {
            robotHardware.setMotorPower(Names.leftOuttake, -0.25);
            robotHardware.setMotorPower(Names.rightOuttake, -0.25);
        }
        if(gamepad2.y) {
            robotHardware.setMotorPower(Names.leftOuttake, 1);
            robotHardware.setMotorPower(Names.rightOuttake, 1);
        }

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rot = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rot), 1);
        double frontLeftPower = (y + x + rot) / denominator;
        double backLeftPower = (y - x + rot) / denominator;
        double frontRightPower = (y - x - rot) / denominator;
        double backRightPower = (y + x - rot) / denominator;

        robotHardware.setMotorPower(Names.frontLeft, frontLeftPower);
        robotHardware.setMotorPower(Names.frontRight, frontRightPower);
        robotHardware.setMotorPower(Names.backLeft, backLeftPower);
        robotHardware.setMotorPower(Names.backRight, backRightPower);
        robotHardware.setMotorPower(Names.slurp,gamepad1.right_trigger - gamepad1.left_trigger);
    }
}
