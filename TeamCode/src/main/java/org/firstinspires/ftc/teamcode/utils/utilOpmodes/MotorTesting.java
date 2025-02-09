package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@Config
@TeleOp(group = "Utils")
public class MotorTesting extends OpMode {
    private RobotHardware robotHardware;
    public static boolean
            frontLeftReverse = false,
            frontRightReverse = false,
            backLeftReverse = false,
            backRightReverse = false,
            leftOuttakeReverse = false,
            rightOuttakeReverse = false,
            intakeExtendoReverse = false,
            slurpReverse = false;
    public static double
            frontLeftPower = 0,
            frontRightPower = 0,
            backLeftPower = 0,
            backRightPower = 0,
            leftOuttakePower = 0,
            rightOuttakePower = 0,
            intakeExtendoPower = 0,
            slurpPower = 0;
    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        robotHardware = RobotHardware.getInstance();
        robotHardware.start();
    }

    @Override
    public void loop() {
        robotHardware.setMotorDirection(Names.frontLeft, frontLeftReverse);
        robotHardware.setMotorDirection(Names.frontRight, frontRightReverse);
        robotHardware.setMotorDirection(Names.backLeft, backLeftReverse);
        robotHardware.setMotorDirection(Names.backRight, backRightReverse);
        robotHardware.setMotorDirection(Names.leftOuttake, leftOuttakeReverse);
        robotHardware.setMotorDirection(Names.rightOuttake, rightOuttakeReverse);
        robotHardware.setMotorDirection(Names.intakeExtendo, intakeExtendoReverse);
        robotHardware.setMotorDirection(Names.slurp, slurpReverse);

        robotHardware.setMotorPower(Names.frontLeft, frontLeftPower);
        robotHardware.setMotorPower(Names.frontRight, frontRightPower);
        robotHardware.setMotorPower(Names.backLeft, backLeftPower);
        robotHardware.setMotorPower(Names.backRight, backRightPower);
        robotHardware.setMotorPower(Names.leftOuttake, leftOuttakePower);
        robotHardware.setMotorPower(Names.rightOuttake, rightOuttakePower);
        robotHardware.setMotorPower(Names.intakeExtendo, intakeExtendoPower);
        robotHardware.setMotorPower(Names.slurp, slurpPower);
    }
}
