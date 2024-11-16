package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Names;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

@TeleOp
public class BasicTele extends OpMode {
    private RobotHardware robotHardware;
    private double deltaTime = 0;
    private PIDController intakePID, outtakePID;
    private ElapsedTime clawTime;
    private boolean grab=false, manualIntake=false;
    private double ip=0.014, ii=0.15, id=0.00081, op=0.04, oi=0.015, od=0.0005, of=0.05;
    private double iTarget=0, oTarget=0;

    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.setDaemon(true);
        robotHardware.start();
        clawTime = new ElapsedTime();
        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);
    }

    @Override
    public void start() {
        robotHardware.servoInit();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robotHardware.setMotorPower(Names.intakeExtendo, -1);
            manualIntake = true;
        }
        else if (gamepad1.y) {
            robotHardware.setMotorPower(Names.intakeExtendo, 1);
            manualIntake = true;
        }
        if (gamepad1.x) {
            robotHardware.setServoPos(Names.intakeArm, 0.7);
            robotHardware.setServoPos(Names.intakePivot, 0.3);
            robotHardware.setMotorPower(Names.slurp, 1);
        }
        if (gamepad1.b){
            robotHardware.setMotorPower(Names.slurp, 0);
            iTarget = 0;
            robotHardware.setServoPos(Names.intakeArm, 0);
            robotHardware.setServoPos(Names.intakePivot, 0.02);
            oTarget = 75;
        }
        if (gamepad1.left_bumper) robotHardware.setMotorPower(Names.slurp, -1);

        if(gamepad1.right_bumper) robotHardware.setServoPos(Names.door, 0.4);
        else robotHardware.setServoPos(Names.door, 0);

        if(gamepad2.right_bumper && clawTime.seconds() > 0.25) {
            clawTime.reset();
            grab = !grab;
            if (grab) robotHardware.setServoPos(Names.claw ,0.2);
            else robotHardware.setServoPos(Names.claw, 0);
        }
        if(gamepad2.right_stick_button) {
            robotHardware.setServoPos(Names.outtakeArm, 0.06);
            robotHardware.setServoPos(Names.outtakePivot, 0.08);
            oTarget = 0;
        }
        if(gamepad2.left_stick_button) {
            robotHardware.setServoPos(Names.outtakeArm, 0.4);
            robotHardware.setServoPos(Names.outtakePivot, 0.45);
        }

        if (gamepad2.y) oTarget = 720;
        if (gamepad2.a) oTarget = 0;
        if (gamepad2.dpad_left) robotHardware.setServoPos(Names.clawPivot, 0.5);
        if (gamepad2.dpad_right) robotHardware.setServoPos(Names.clawPivot, 1);

        double x = gamepad1.right_trigger - gamepad1.left_trigger;
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

        iTarget = Math.max(Math.min(iTarget, 400), 0);
        oTarget = Math.max(Math.min(oTarget, 725), 0);

        int iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        int oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2;

        if (manualIntake) {
            iTarget = iArmPos;
            manualIntake = false;
        }
        else robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));
        double oPower = outtakePID.calculate(oArmPos, (int) oTarget);
        robotHardware.setMotorPower(Names.leftOuttake, oPower);
        robotHardware.setMotorPower(Names.rightOuttake, oPower);

        deltaTime = getRuntime();
        resetRuntime();

        telemetry.addData("intake target", iTarget);
        telemetry.addData("delta time", deltaTime);
        telemetry.update();
    }
}
