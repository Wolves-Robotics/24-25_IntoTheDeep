package org.firstinspires.ftc.teamcode.utils.utilOpmodes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.collections.Names;
import org.firstinspires.ftc.teamcode.utils.utilClasses.PDFL;

@TeleOp
@Config
public class PDFLTesting extends OpMode {
    private MultipleTelemetry multipleTelemetry;

    public static double ip = 0, id = 0, If = 0, il = 0,
                         op = 0, od = 0, of = 0, ol = 0;
    public static int ideadZone = 0, itarget = 0,
                      odeadzone = 0, otarget = 0;

    private PDFL pdfl, pdfl2;

    @Override
    public void init() {
        RobotHardware.reset(hardwareMap);
        RobotHardware.getInstance().autoServoInit();

        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        pdfl = new PDFL(
                () -> RobotHardware.getInstance().getMotorPos(Names.intakeExtendo),
                (p) -> RobotHardware.getInstance().setMotorPower(Names.intakeExtendo, p));

        pdfl2 = new PDFL(
                () -> (RobotHardware.getInstance().getMotorPos(Names.leftOuttake) + RobotHardware.getInstance().getMotorPos(Names.rightOuttake)) / 2,
                (p) -> {RobotHardware.getInstance().setMotorPower(Names.leftOuttake, p); RobotHardware.getInstance().setMotorPower(Names.rightOuttake, p);});
    }

    @Override
    public void loop() {
        pdfl.setCoefficients(ip, id, If, il, ideadZone);
        pdfl.setTarget(itarget);
        pdfl.update();

        pdfl2.setCoefficients(op, od, of, ol, odeadzone);
        pdfl2.setTarget(otarget);
        pdfl2.update();

        multipleTelemetry.addData("Intake pos", pdfl.getPos());
        multipleTelemetry.addData("Intake target", itarget);
        multipleTelemetry.addData("Intake power", pdfl.getPower());
        multipleTelemetry.addData("Outtake pos", pdfl2.getPos());
        multipleTelemetry.addData("Outtake target", otarget);
        multipleTelemetry.addData("Outtake power", pdfl2.getPower());
        multipleTelemetry.update();
    }
}
