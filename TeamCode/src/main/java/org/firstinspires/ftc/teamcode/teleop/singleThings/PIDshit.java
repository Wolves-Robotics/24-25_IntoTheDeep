package org.firstinspires.ftc.teamcode.teleop.singleThings;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class PIDshit extends OpMode {
    DcMotor leftOuttake, rightOuttake, intake;
    public static double p = 0.015, i = 0, d = 0.00022, f =0.05;
    private final double ticksPerDeg = 760/180.;
    public static int target = 0;
    PIDController controller;
    MultipleTelemetry multipleTelemetry;


    @Override
    public void init() {
        multipleTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        controller = new PIDController(p, i, d);
        leftOuttake = hardwareMap.get(DcMotor.class, "leftOuttake");
        rightOuttake = hardwareMap.get(DcMotor.class, "rightOuttake");
        intake = hardwareMap.get(DcMotor.class, "intakeExtendo");
        leftOuttake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        controller.setPID(p, i, d);
        int armPos = (leftOuttake.getCurrentPosition() + rightOuttake.getCurrentPosition())/2 -3;
//        int armPos = intake.getCurrentPosition();
        double pow = controller.calculate(armPos, target);
        double power = pow + f;

        leftOuttake.setPower(power);
        rightOuttake.setPower(power);
//        intake.setPower(power);
        multipleTelemetry.addData("Outtake pos", armPos);
        multipleTelemetry.addData("Outtake target", target);
        multipleTelemetry.addData("Outtake ff", f);
        multipleTelemetry.addData("Outtake power", pow);
        multipleTelemetry.update();
    }
}
