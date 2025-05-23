package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@Config
@TeleOp(group = "Utils")
public class ServoTesting extends OpMode {
    private RobotHardware robotHardware;
    public static boolean
            doorReverse = false,
            intakePivotReverse = false,
            intakeArmReverse = false,
            outtakePivotReverse = false,
            outtakeArmReverse = false,
            clawReverse = false,
            leftHangReverse = false,
            rightHangReverse = true;

    public static double
            doorPos = 0.6,
            intakePivotPos = 0.3,
            intakeArmPos = 0.3,
            outtakePivotPos = 0.5,
            outtakeArmPos = 0.5,
            clawPos = 0.2,
            leftHangPos = 0.5,
            rightHangPos = 0.5;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        robotHardware = RobotHardware.getInstance();
        robotHardware.start();
    }

    @Override
    public void loop() {
        robotHardware.setServoDirection(Names.door, doorReverse);
        robotHardware.setServoDirection(Names.intakePivot, intakePivotReverse);
        robotHardware.setServoDirection(Names.intakeArm, intakeArmReverse);
        robotHardware.setServoDirection(Names.outtakePivot, outtakePivotReverse);
        robotHardware.setServoDirection(Names.outtakeArm, outtakeArmReverse);
        robotHardware.setServoDirection(Names.claw, clawReverse);

        robotHardware.setServoPos(Names.door, doorPos);
        robotHardware.setServoPos(Names.intakePivot, intakePivotPos);
        robotHardware.setServoPos(Names.intakeArm, intakeArmPos);
        robotHardware.setServoPos(Names.outtakePivot, outtakePivotPos);
        robotHardware.setServoPos(Names.outtakeArm, outtakeArmPos);
        robotHardware.setServoPos(Names.claw, clawPos);
    }
}
