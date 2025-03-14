package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Names;

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
            leftHangPos = 0,
            rightHangPos = 0;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        robotHardware = RobotHardware.getInstance();
        robotHardware.start();
    }

    @Override
    public void loop() {
        robotHardware.setServoReverse(Names.door, doorReverse);
        robotHardware.setServoReverse(Names.intakePivot, intakePivotReverse);
        robotHardware.setServoReverse(Names.intakeArm, intakeArmReverse);
        robotHardware.setServoReverse(Names.outtakePivot, outtakePivotReverse);
        robotHardware.setServoReverse(Names.outtakeArm, outtakeArmReverse);
        robotHardware.setServoReverse(Names.claw, clawReverse);

        robotHardware.setServoPos(Names.door, doorPos);
        robotHardware.setServoPos(Names.intakePivot, intakePivotPos);
        robotHardware.setServoPos(Names.intakeArm, intakeArmPos);
        robotHardware.setServoPos(Names.outtakePivot, outtakePivotPos);
        robotHardware.setServoPos(Names.outtakeArm, outtakeArmPos);
        robotHardware.setServoPos(Names.claw, clawPos);
    }
}