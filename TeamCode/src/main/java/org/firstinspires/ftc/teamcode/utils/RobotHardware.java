package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;

import java.util.HashMap;
import java.util.List;

@Config
public class RobotHardware extends Thread {
    private void setHardwareMaps() {
        motorClassMap = new HashMap<>();
        motorClassMap.put(Names.frontLeft,     new MotorClass(Names.frontLeft,     true));
        motorClassMap.put(Names.frontRight,    new MotorClass(Names.frontRight,    false));
        motorClassMap.put(Names.backLeft,      new MotorClass(Names.backLeft,      true));
        motorClassMap.put(Names.backRight,     new MotorClass(Names.backRight,     false));
        motorClassMap.put(Names.leftOuttake,   new MotorClass(Names.leftOuttake,   true));
        motorClassMap.put(Names.rightOuttake,  new MotorClass(Names.rightOuttake,  false));
        motorClassMap.put(Names.intakeExtendo, new MotorClass(Names.intakeExtendo, false));
        motorClassMap.put(Names.slurp,         new MotorClass(Names.slurp,         false));

        servoClassMap = new HashMap<>();
        servoClassMap.put(Names.door,          new ServoClass(Names.door,          false));
        servoClassMap.put(Names.intakePivot,   new ServoClass(Names.intakePivot,   false));
        servoClassMap.put(Names.intakeArm,     new ServoClass(Names.intakeArm,     false));
        servoClassMap.put(Names.outtakeArm,    new ServoClass(Names.outtakeArm,    false));
        servoClassMap.put(Names.outtakePivot,  new ServoClass(Names.outtakePivot,  false));
        servoClassMap.put(Names.claw,          new ServoClass(Names.claw,          false));
//        servoClassMap.put(Names.leftHang,      new ServoClass(Names.leftHang,      false));
//        servoClassMap.put(Names.rightHang,     new ServoClass(Names.rightHang,     true));

        colorSensorMap = new HashMap<>();
        colorSensorMap.put(Names.intakeColor,    new ColorSensorClass(Names.intakeColor));
        colorSensorMap.put(Names.transferColor,  new ColorSensorClass(Names.transferColor));
    }

    private static RobotHardware instance = null;

    private static HardwareMap hardwareMap;
    private List<LynxModule> allHubs;

    private HashMap<Names, MotorClass> motorClassMap;
    private HashMap<Names, ServoClass> servoClassMap;
    private HashMap<Names, ColorSensorClass> colorSensorMap;

    private IMU imu;
    private GoBildaPinpointDriver pinpoint;

    private RevBlinkinLedDriver lights;
    private RevBlinkinLedDriver.BlinkinPattern prevPatter;

    private static class MotorClass {
        DcMotor motor;

        public MotorClass(Names name, boolean isReverse) {
            motor = hardwareMap.dcMotor.get(Constants.getStringName(name));
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (isReverse) motor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private static class ServoClass {
        Servo servo;

        public ServoClass(Names name, boolean isReverse) {
            servo = hardwareMap.get(ServoImplEx.class, Constants.getStringName(name));
            if (isReverse) servo.setDirection(Servo.Direction.REVERSE);
        }
    }

    private static class ColorSensorClass {
        RevColorSensorV3 colorSensor;

        public ColorSensorClass(Names name) {
            colorSensor = hardwareMap.get(RevColorSensorV3.class, Constants.getStringName(name));
        }
    }

    public static void reset(HardwareMap _hardwareMap) {
        instance = new RobotHardware(_hardwareMap);
    }

    public static RobotHardware getInstance() {
        return instance;
    }

    private RobotHardware(HardwareMap _hardwareMap) {
        hardwareMap = _hardwareMap;

        lynxModuleInit();

        setHardwareMaps();

        setImu();

        setLights();

        resetSubsystems();

        start();
    }

    public void teleOpServoInit() {
        setServoPos(Names.intakeArm, 0.1);
        setServoPos(Names.intakePivot, 0.19);
        setServoPos(Names.claw, 0.5);
        setServoPos(Names.outtakeArm, 0.2);
        setServoPos(Names.outtakePivot, 0.1);
        setServoPos(Names.door, 0.7);
    }

    private void lynxModuleInit() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void setImu() {
        imu = hardwareMap.get(IMU.class, Constants.getStringName(Names.imu));
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setOffsets(2.875, -6.875);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
    }

    private void setLights() {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, Constants.getStringName(Names.lights));
    }

    public void servoInit() {
        setServoPos(Names.intakeArm, 0.02);
        setServoPos(Names.intakePivot, 0.19);
        setServoPos(Names.claw, 0.5);
        setServoPos(Names.outtakeArm, 0.23);
        setServoPos(Names.outtakePivot, 0.4);
        setServoPos(Names.door, 0.7);
    }

    private void resetSubsystems() {
        IntakeSubsystem.reset();
        OuttakeSubsystem.reset();
        DriveSubsystem.reset();
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            lynxModuleUpdate();
        }
        IntakeSubsystem.getInstance().interrupt();
        OuttakeSubsystem.getInstance().interrupt();
        DriveSubsystem.getInstance().interrupt();
    }

    public HardwareMap getHardwareMap() {
        return hardwareMap;
    }

    public int getMotorPos(Names name) {
        return motorClassMap.get(name).motor.getCurrentPosition();
    }
    public double getServoPos(Names name) {
        return servoClassMap.get(name).servo.getPosition();
    }

    public double getYaw() {
        return pinpoint.getHeading();
    }
    public YawPitchRollAngles getImuAngles() {
        return imu.getRobotYawPitchRollAngles();
    }
    public void resetImuYaw() {
        imu.resetYaw();
//        pinpoint.recalibrateIMU();
    }

    public void setMotorPower(Names name, double power) {
        motorClassMap.get(name).motor.setPower(power);
    }
    public void setServoPos(Names name, double pos) {
        servoClassMap.get(name).servo.setPosition(pos);
    }

    public void setMotorDirection(Names name, boolean reverse) {
        motorClassMap.get(name).motor.setDirection(reverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }

    public void setServoDirection(Names name, boolean reverse) {
        servoClassMap.get(name).servo.setDirection(reverse ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public boolean isRed(Names name) {
        return  colorSensorMap.get(name).colorSensor.red() >= 110 &&
                colorSensorMap.get(name).colorSensor.green() <= 110 &&
                colorSensorMap.get(name).colorSensor.blue() <= 50;
    }

    public int getRed(Names name) {
        return colorSensorMap.get(name).colorSensor.red();
    }

    public boolean isBlue(Names name) {
        return  colorSensorMap.get(name).colorSensor.red() <= 80 &&
                colorSensorMap.get(name).colorSensor.green() <= 94 &&
                colorSensorMap.get(name).colorSensor.blue() >= 100;
    }

    public int getBlue(Names name) {
        return colorSensorMap.get(name).colorSensor.blue();
    }

    public boolean isYellow(Names name) {
        return  colorSensorMap.get(name).colorSensor.red() >= 200 &&
                colorSensorMap.get(name).colorSensor.green() >= 210 &&
                colorSensorMap.get(name).colorSensor.blue() <= 90;
    }

    public int getGreen(Names name) {
        return colorSensorMap.get(name).colorSensor.green();
    }

    public double getDistance(Names name) {
        return colorSensorMap.get(name).colorSensor.getDistance(DistanceUnit.CM);
    }

    public void startPids() {
        IntakeSubsystem.getInstance().startPid();
        OuttakeSubsystem.getInstance().startPid();
    }

    public void setLightColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (pattern != prevPatter) {
            prevPatter = pattern;
            lights.setPattern(pattern);
        }
    }

//    public void killServos() {
//        servoClassMap.get(Names.claw).servo.setPwmDisable();
//        servoClassMap.get(Names.outtakeArm).servo.setPwmDisable();
//        servoClassMap.get(Names.outtakePivot).servo.setPwmDisable();
//        servoClassMap.get(Names.intakeArm).servo.setPwmDisable();
//        servoClassMap.get(Names.intakePivot).servo.setPwmDisable();
//        servoClassMap.get(Names.door).servo.setPwmDisable();
//    }

    public void updateLeds(Names names) {
        if (isRed(names)) setLightColor(RevBlinkinLedDriver.BlinkinPattern.RED);
        else if (isBlue(names)) setLightColor(RevBlinkinLedDriver.BlinkinPattern.BLUE);
        else if (isYellow(names)) setLightColor(RevBlinkinLedDriver.BlinkinPattern.GOLD);
        else setLightColor(RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE);
    }

    private void lynxModuleUpdate() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}