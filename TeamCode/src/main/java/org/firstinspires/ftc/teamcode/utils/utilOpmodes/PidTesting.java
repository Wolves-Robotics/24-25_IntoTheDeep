package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.utilClasses.PIDF;

@Config
@TeleOp(group = "Utils")
public class PidTesting extends OpMode {
    private RobotHardware robotHardware;
    private PIDController intake, outtake;
    private PIDF intakePid, outtakePidf;
    public static double ip=0, ii=0, id=0, op=0, oi=0, od=0, of=0;
    public static int iTarget=0, oTarget=0;
    public static double iArm=0, iPiv=0;
    private MultipleTelemetry multipleTelemetry;


    @Override
    public void init() {
        robotHardware = new RobotHardware(hardwareMap);
        robotHardware.start();
        robotHardware.setServoPos(Names.intakeArm, 0);
        robotHardware.setServoPos(Names.intakePivot, 0);
        intake = new PIDController(ip, ii, id);
        intakePid = new PIDF(0, 0, 0,
                () -> robotHardware.getMotorPos(Names.intakeExtendo),
                x -> robotHardware.setMotorPower(Names.intakeExtendo, x)
        );
        intakePid.setDaemon(true);
//        intakePid.start();
        outtake = new PIDController(op, oi, od);
        outtakePidf = new PIDF(0, 0, 0, 0,
                () -> (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake))/2,
                x -> {robotHardware.setMotorPower(Names.leftOuttake, x); robotHardware.setMotorPower(Names.rightOuttake, x);}
        );
        outtakePidf.setDaemon(true);
//        outtakePidf.start();
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
//        intakePid.setCoefficients(ip, ii, id);
//        intakePid.setTarget(iTarget);
//        intakePid.updateTelemetry(multipleTelemetry, "Intake");
//        outtakePidf.setCoefficients(op, oi, od, of);
//        outtakePidf.setTarget(oTarget);
//        outtakePidf.updateTelemetry(multipleTelemetry, "Outtake");
        robotHardware.setServoPos(Names.intakeArm, iArm);
        robotHardware.setServoPos(Names.intakePivot, iPiv);

        intake.setPID(ip, ii, id);
        robotHardware.setMotorPower(Names.intakeExtendo, intake.calculate(robotHardware.getMotorPos(Names.intakeExtendo), iTarget));

        outtake.setPID(op, oi, od);
        double power = outtake.calculate((robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake))/2., oTarget) + of;
        robotHardware.setMotorPower(Names.leftOuttake, power);
        robotHardware.setMotorPower(Names.rightOuttake, power);
        multipleTelemetry.addData("Intake pos", robotHardware.getMotorPos(Names.intakeExtendo));
        multipleTelemetry.addData("Intake target", iTarget);
        multipleTelemetry.addData("Outtake pos", (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake))/2.);
        multipleTelemetry.addData("Outtake target", oTarget);
        multipleTelemetry.update();

    }
}
