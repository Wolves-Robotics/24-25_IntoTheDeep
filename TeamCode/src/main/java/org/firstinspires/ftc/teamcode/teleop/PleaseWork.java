package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

public class PleaseWork extends OpMode {
    private RobotHardware robotHardware;


    private PIDController intakePID, outtakePID;
    private int iTarget, oTarget;
    private boolean manualIntake;

    private ElapsedTime clawTime;
    private boolean grab;

    private double ip=0.014, ii=0.15, id=0.00081, op=0.04, oi=0, od=0.001, of=0.16;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);

        iTarget = 0;
        oTarget = 0;
        manualIntake = false;
    }

    @Override
    public void loop() {

        double x = gamepad1.left_stick_x + gamepad2.left_stick_x;
        double y = -gamepad1.left_stick_y - gamepad2.left_stick_y;
        double rot = gamepad1.right_stick_x + gamepad2.right_stick_x;

        double botHeading = robotHardware.getImuAngles().getYaw(AngleUnit.RADIANS);
        double rotX = (x * Math.cos(-botHeading) - y * Math.sin(-botHeading)) *1.1;
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rot), 1);
        double frontLeftPower = (rotY + rotX + rot) / denominator;
        double backLeftPower = (rotY - rotX + rot) / denominator;
        double frontRightPower = (rotY - rotX - rot) / denominator;
        double backRightPower = (rotY + rotX - rot) / denominator;

        RobotHardware.getInstance().setMotorPower(Names.frontLeft, frontLeftPower);
        RobotHardware.getInstance().setMotorPower(Names.frontRight, frontRightPower);
        RobotHardware.getInstance().setMotorPower(Names.backLeft, backLeftPower);
        RobotHardware.getInstance().setMotorPower(Names.backRight, backRightPower);
        //PID STUFF
        iTarget = Math.max(Math.min(iTarget, 400), 0);

        int iArmPos = robotHardware.getMotorPos(Names.intakeExtendo);
        int oArmPos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2 -3;
        if (manualIntake) {
            iTarget = iArmPos;
            manualIntake = false;
        }
        else robotHardware.setMotorPower(Names.intakeExtendo, intakePID.calculate(iArmPos, iTarget));
        double oPow = outtakePID.calculate(oArmPos, oTarget);
        double power = oPow + of;
        if (oTarget == 0 && oArmPos < 50 && oArmPos > 5) power -= 0.1;
        if (oTarget == 0 && oArmPos <= 5) power = 0;
        power = Math.max(-0.5, power);
        telemetry.addData("outtake power", power);
        robotHardware.setMotorPower(Names.leftOuttake, power);
        robotHardware.setMotorPower(Names.rightOuttake, power);
    }
}
