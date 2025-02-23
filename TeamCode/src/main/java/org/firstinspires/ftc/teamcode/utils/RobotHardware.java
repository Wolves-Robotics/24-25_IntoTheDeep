package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.collections.Names;

import java.util.HashMap;
import java.util.List;

@Config
public class RobotHardware extends Thread {
    private void setHardwareMaps() {
        motorClassMap = new HashMap<>();
        motorClassMap.put(Names.frontLeft,     new MotorClass(Names.frontLeft));
        motorClassMap.put(Names.frontRight,    new MotorClass(Names.frontRight));
        motorClassMap.put(Names.backLeft,      new MotorClass(Names.backLeft));
        motorClassMap.put(Names.backRight,     new MotorClass(Names.backRight));
        motorClassMap.put(Names.leftOuttake,   new MotorClass(Names.leftOuttake));
        motorClassMap.put(Names.rightOuttake,  new MotorClass(Names.rightOuttake));
        motorClassMap.put(Names.intakeExtendo, new MotorClass(Names.intakeExtendo));
        motorClassMap.put(Names.slurp,         new MotorClass(Names.slurp));

        servoClassMap = new HashMap<>();
        servoClassMap.put(Names.door,          new ServoClass(Names.door));
        servoClassMap.put(Names.intakePivot,   new ServoClass(Names.intakePivot));
        servoClassMap.put(Names.intakeArm,     new ServoClass(Names.intakeArm));
        servoClassMap.put(Names.outtakeArm,    new ServoClass(Names.outtakeArm));
        servoClassMap.put(Names.outtakePivot,  new ServoClass(Names.outtakePivot));
        servoClassMap.put(Names.claw,          new ServoClass(Names.claw));

        colorSensorMap = new HashMap<>();
        colorSensorMap.put(Names.intakeColor,    new ColorSensorClass(Names.intakeColor));
        colorSensorMap.put(Names.transferDistance,  new ColorSensorClass(Names.transferDistance));
    }

    private static RobotHardware instance = null;

    private static HardwareMap hardwareMap;
    private List<LynxModule> allHubs;

    private HashMap<Names, MotorClass> motorClassMap;
    private HashMap<Names, ServoClass> servoClassMap;
    private HashMap<Names, ColorSensorClass> colorSensorMap;

    private GoBildaPinpointDriver pinpoint;

    private RevBlinkinLedDriver lights;
    private RevBlinkinLedDriver.BlinkinPattern prevPatter;

    private static class MotorClass {
        DcMotor motor;

        public MotorClass(Names name) {
            motor = hardwareMap.dcMotor.get(Constants.getStringName(name));
            if (Constants.getHardwareReversed(name)) motor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private static class ServoClass {
        ServoImplEx servo;

        public ServoClass(Names name) {
            servo = hardwareMap.get(ServoImplEx.class, Constants.getStringName(name));
            if (Constants.getHardwareReversed(name)) servo.setDirection(Servo.Direction.REVERSE);
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

//        IntakeSubsystem.reset();
//        OuttakeSubsystem.reset();
//        DriveSubsystem.reset();

        lynxModuleInit();

        setHardwareMaps();

        setImu();

        setLights();

        start();
    }

    private void lynxModuleInit() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void setImu() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, Constants.getStringName(Names.pinpoint));
        pinpoint.setOffsets(73.025, -174.498);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    }

    private void setLights() {
        lights = hardwareMap.get(RevBlinkinLedDriver.class, Constants.getStringName(Names.lights));
    }

    public void autoServoInit() {
        setServoPos(Names.intakeArm, 0.1);
        setServoPos(Names.intakePivot, 0.19);
        setServoPos(Names.claw, 0.3);
        setServoPos(Names.outtakeArm, 0.23);
        setServoPos(Names.outtakePivot, 0.4);
        setServoPos(Names.door, 0.7);
    }

    public void teleOpServoInit() {
        setServoPos(Names.intakeArm, 0.1);
        setServoPos(Names.intakePivot, 0.19);
        setServoPos(Names.claw, 0.3);
        setServoPos(Names.outtakeArm, 0.23);
        setServoPos(Names.outtakePivot, 0.4);
        setServoPos(Names.door, 0.7);
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            lynxModuleUpdate();
            pinpoint.update();
        }
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

    public double getHeading() {
        return pinpoint.getHeading();
    }

    public void resetImuYaw() {
        pinpoint.resetPosAndIMU();
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

    public void setServoReverse(Names name, boolean reverse) {
        servoClassMap.get(name).servo.setDirection(reverse ? Servo.Direction.REVERSE : Servo.Direction.FORWARD);
    }

    public boolean isRed(Names name) {
        return  colorSensorMap.get(name).colorSensor.red() >= 150 &&
                colorSensorMap.get(name).colorSensor.green() <= 285 &&
                colorSensorMap.get(name).colorSensor.blue() <= 120;
    }

    public int getRed(Names name) {
        return colorSensorMap.get(name).colorSensor.red();
    }

    public boolean isBlue(Names name) {
        return  colorSensorMap.get(name).colorSensor.red() <= 110 &&
                colorSensorMap.get(name).colorSensor.green() <= 220 &&
                colorSensorMap.get(name).colorSensor.blue() >= 130;
    }

    public int getBlue(Names name) {
        return colorSensorMap.get(name).colorSensor.blue();
    }

    public boolean isYellow(Names name) {
        return  colorSensorMap.get(name).colorSensor.red() >= 240 &&
                colorSensorMap.get(name).colorSensor.green() >= 300 &&
                colorSensorMap.get(name).colorSensor.blue() <= 300;

    }

    public int getGreen(Names name) {
        return colorSensorMap.get(name).colorSensor.green();
    }

    public double getDistance(Names name) {
        return colorSensorMap.get(name).colorSensor.getDistance(DistanceUnit.CM);
    }

    public void startPids() {

    }

    public void setLightColor(RevBlinkinLedDriver.BlinkinPattern pattern) {
        if (pattern != prevPatter) {
            prevPatter = pattern;
            lights.setPattern(pattern);
        }
    }

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