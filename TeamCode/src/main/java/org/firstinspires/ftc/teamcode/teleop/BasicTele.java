package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.commands.complex.sample.GrabSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetract;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

@TeleOp
public class BasicTele extends OpMode {
    private RobotHardware robotHardware;
    private double deltaTime = 0;
    private PIDController intakePID, outtakePID;
    private ElapsedTime clawTime;
    private boolean grab=false, manualIntake=false;
    private final double ticksPerDeg = 760/180.;
    private double ip=0.014, ii=0.15, id=0.00081, op=0.015, oi=0, od=0.0002, of=0.05;
    private double iTarget=0, oTarget=0;
    double ff = Math.cos(Math.toRadians(oTarget / ticksPerDeg)) * of;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        RobotHardware.reset(hardwareMap);
        robotHardware = RobotHardware.getInstance();
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
        CommandScheduler.getInstance().run();
        //COMMANDS
        if (gamepad1.a) {
            robotHardware.setMotorPower(Names.intakeExtendo, -1);
            manualIntake = true;
        }
        else if (gamepad1.y) {
            robotHardware.setMotorPower(Names.intakeExtendo, 1);
            manualIntake = true;
        }
        if (gamepad1.x) {
            robotHardware.setServoPos(Names.intakePivot, 0.48);
            robotHardware.setServoPos(Names.intakeArm, 0.73);
            robotHardware.setMotorPower(Names.slurp, 1);
            robotHardware.setServoPos(Names.door, 0.73);

        }
        if (gamepad1.b){
            CommandScheduler.getInstance().schedule(new IntakeRetract());
            iTarget = 0;
        }
        if(gamepad1.dpad_right){
            oTarget = 0;
            robotHardware.setServoPos(Names.outtakeArm, 0.69);
            robotHardware.setServoPos(Names.outtakePivot, 0.3);
            robotHardware.setServoPos(Names.intakePivot, 0.22);
            robotHardware.setServoPos(Names.intakeArm, 0.3);
        }

        if(gamepad1.dpad_up){
            oTarget = 1300;
            robotHardware.setServoPos(Names.outtakeArm, 0.76);
            robotHardware.setServoPos(Names.outtakePivot, 0.35);
        }
        if(gamepad1.dpad_down){
            oTarget = 0;
        }
        if (gamepad1.left_bumper) robotHardware.setMotorPower(Names.slurp, -1);

        if((gamepad1.right_bumper || gamepad2.right_bumper) && clawTime.seconds() > 0.2) {
            clawTime.reset();
            grab = !grab;
            if (grab) robotHardware.setServoPos(Names.claw ,0);
            else robotHardware.setServoPos(Names.claw, 0.35);
        }
        if(gamepad2.right_stick_button) {
            robotHardware.setServoPos(Names.outtakeArm, 0.2);
            robotHardware.setServoPos(Names.outtakePivot, 0.1);
            robotHardware.setServoPos(Names.intakeArm, 0.3);
            robotHardware.setServoPos(Names.intakePivot, 0.22);

            oTarget = 0;
        }
        if(gamepad2.left_stick_button) {
            CommandScheduler.getInstance().schedule(new GrabSample());
            grab = true;
        }

        if (gamepad2.y){
            oTarget = 1940;
            robotHardware.setServoPos(Names.outtakeArm, 0.45);
            robotHardware.setServoPos(Names.outtakePivot, 0.4);
            robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }
        if (gamepad2.a) oTarget = 0;

        if (gamepad2.dpad_down) {
            robotHardware.setServoPos(Names.intakePivot, 0.45);
            robotHardware.setServoPos(Names.intakeArm, 0.6);
            iTarget = 0;
        }
        //DRIVING
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
        double power = oPow + ff;
        robotHardware.setMotorPower(Names.leftOuttake, power);
        robotHardware.setMotorPower(Names.rightOuttake, power);

        if (robotHardware.isRed(Names.intakeColor)) robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.RED);
        else if (robotHardware.isBlue(Names.intakeColor)) robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else if (robotHardware.isYellow(Names.intakeColor)) robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        else robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);

        deltaTime = getRuntime();
        resetRuntime();

        telemetry.addData("delta time", deltaTime);
        telemetry.update();
    }
}