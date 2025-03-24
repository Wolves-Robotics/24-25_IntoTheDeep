package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.commands.complex.sample.GrabSample;
import org.firstinspires.ftc.teamcode.commands.complex.sample.IntakeRetract;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

@TeleOp
public class SoloKota extends OpMode {

    private RobotHardware robotHardware;
    private double deltaTime = 0;
    private PIDController intakePID, outtakePID;
    private ElapsedTime clawTime;
    private boolean grab=false, manualIntake=false;
    private double ip=0.014, ii=0.15, id=0.00081, op=0.04, oi=0, od=0.001, of=0.16;
    private double iTarget=0, oTarget=0;

    private CRServo lefthang;
    private CRServo rightHang;

    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        RobotHardware.reset(hardwareMap);
        robotHardware = RobotHardware.getInstance();
        clawTime = new ElapsedTime();
        intakePID = new PIDController(ip, ii, id);
        outtakePID = new PIDController(op, oi, od);

        lefthang = hardwareMap.get(CRServo.class, "idkhang2");
        rightHang = hardwareMap.get(CRServo.class, "idkhang1");

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
            robotHardware.setServoPos(Names.intakeArm, 0.68);
            robotHardware.setServoPos(Names.intakePivot, 0.48);
            robotHardware.setMotorPower(Names.slurp, 1);
            robotHardware.setServoPos(Names.door, 0.73);

        }
        if (gamepad1.b){
            CommandScheduler.getInstance().schedule(new IntakeRetract());
            iTarget = 0;
        }

        if (gamepad1.dpad_left) robotHardware.setMotorPower(Names.slurp, -1);

        if((gamepad1.right_bumper || gamepad2.right_bumper) && clawTime.seconds() > 0.2) {
            clawTime.reset();
            grab = !grab;
            if (grab) robotHardware.setServoPos(Names.claw ,0);
            else robotHardware.setServoPos(Names.claw, 0.35);
        }
        if(gamepad1.right_stick_button) {
            robotHardware.setServoPos(Names.outtakeArm, 0.2);
            robotHardware.setServoPos(Names.outtakePivot, 0.1);
            robotHardware.setServoPos(Names.intakeArm, 0.3);
            robotHardware.setServoPos(Names.intakePivot, 0.22);
            robotHardware.setServoPos(Names.claw, 0.35);

            oTarget = 0;
        }
        if(gamepad1.left_stick_button) {
            CommandScheduler.getInstance().schedule(new GrabSample());
            grab = true;
        }

        if (gamepad1.left_bumper){
            oTarget = 740;
            robotHardware.setServoPos(Names.outtakeArm, 0.45);
            robotHardware.setServoPos(Names.outtakePivot, 0.4);
            robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }

        if (gamepad1.left_trigger > 0.75) {
            oTarget = 300;
            robotHardware.setServoPos(Names.outtakeArm, 0.45);
            robotHardware.setServoPos(Names.outtakePivot, 0.4);
            robotHardware.setLightColor(RevBlinkinLedDriver.BlinkinPattern.AQUA);
        }

        if (gamepad1.dpad_down) {
            RobotHardware.getInstance().setServoPos(Names.intakeArm, 0.55);
            RobotHardware.getInstance().setServoPos(Names.intakePivot, 0.45);
            iTarget = 0;
        }

        if (gamepad1.options) {
            robotHardware.resetImuYaw();
        }

        if(gamepad2.dpad_right){
            oTarget = 20;
            RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.7);
            RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.3);
            robotHardware.setServoPos(Names.intakePivot, 0.22);
            robotHardware.setServoPos(Names.intakeArm, 0.3);
        }

        if(gamepad2.dpad_up){
            oTarget = 450;
            RobotHardware.getInstance().setServoPos(Names.outtakeArm, 0.6);
            RobotHardware.getInstance().setServoPos(Names.outtakePivot, 0.55);
        }
        if(gamepad2.dpad_down){
            oTarget = 0;
        }

        if (gamepad1.dpad_up) {
            lefthang.setPower(1);
            rightHang.setPower(-1);
        } else if (gamepad1.dpad_right) {
            lefthang.setPower(-1);
            rightHang.setPower(1);
        } else {
            lefthang.setPower(0);
            rightHang.setPower(0);
        }

        // sudo rm /* -rf


        //DRIVING
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
