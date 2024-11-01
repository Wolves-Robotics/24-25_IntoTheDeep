package org.firstinspires.ftc.teamcode.hardware;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.autonomous.collections.Color;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.HashMap;
import java.util.List;

@Config
public class RobotHardware extends Thread {
    private static HardwareMap hardwareMap;
    private List<LynxModule> allHubs;

    private static HashMap<Names, String> nameHashMap;
    private HashMap<String, MotorClass> motorClassMap;
    private HashMap<String, ServoClass> servoClassMap;

    private IMU imu;
    private YawPitchRollAngles imuAngles;

    private ElapsedTime deltaTime;
    private double lastLoopTime;

    private static class MotorClass {
        DcMotor motor;
        int pos = 0;

        public MotorClass(Names name, boolean isReverse) {
            motor = hardwareMap.dcMotor.get(nameHashMap.get(name));
            if (isReverse) motor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private static class ServoClass {
        ServoImplEx servo;
        double pos = 0d;

        public ServoClass(Names name, boolean isReverse) {
            servo = hardwareMap.get(ServoImplEx.class, nameHashMap.get(name));
            if (isReverse) servo.setDirection(Servo.Direction.REVERSE);
        }
    }

    private void standardInit(HardwareMap _hardwareMap) {
        hardwareMap = _hardwareMap;
        deltaTime = new ElapsedTime();

        lynxModuleInit();

        setNameHashMap();
        setHardwareMap();

        setImu();

//        servoInit();
    }

    public RobotHardware(HardwareMap _hardwareMap) {
        standardInit(_hardwareMap);
    }

    private void lynxModuleInit() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void setNameHashMap() {
        nameHashMap = new HashMap<>();

        nameHashMap.put(Names.frontLeft, "frontLeft");
        nameHashMap.put(Names.frontRight, "frontRight");
        nameHashMap.put(Names.backLeft, "backLeft");
        nameHashMap.put(Names.backRight, "backRight");
        nameHashMap.put(Names.slideLeft, "slideLeft");
        nameHashMap.put(Names.slideRight, "slideRight");
        nameHashMap.put(Names.intakeExtendo, "intakeExtendo");
        nameHashMap.put(Names.slurp, "slurp");

        nameHashMap.put(Names.colorServo, "colorServo");
        nameHashMap.put(Names.intakePitch, "intakePitch");
        nameHashMap.put(Names.armPitch, "armPitch");
        nameHashMap.put(Names.outtake1, "");
        nameHashMap.put(Names.outtake2, "");
        nameHashMap.put(Names.outtakeRotate, "");
        nameHashMap.put(Names.outtakeGrab, "");

        nameHashMap.put(Names.colorLeft, "colorLeft");
        nameHashMap.put(Names.colorMid, "colorMid");
        nameHashMap.put(Names.colorRight, "colorRight");
    }

    private void setHardwareMap() {
        motorClassMap = new HashMap<>();
        motorClassMap.put("frontLeft", new MotorClass(Names.frontLeft, true));
        motorClassMap.put("frontRight", new MotorClass(Names.frontRight, false));
        motorClassMap.put("backLeft", new MotorClass(Names.backLeft, true));
        motorClassMap.put("backRight", new MotorClass(Names.backRight, false));
//        motorClassMap.put("slideLeft", new MotorClass(Names.slideLeft, false));
//        motorClassMap.put("slideRight", new MotorClass(Names.slideRight, false));
//        motorClassMap.put("intakeExtendo", new MotorClass(Names.intakeExtendo, false));
//        motorClassMap.put("slurp", new MotorClass(Names.slurp, false));

        servoClassMap = new HashMap<>();
//        servoClassMap.put("colorServo", new ServoClass(Names.colorServo, false));
//        servoClassMap.put("intakePitch", new ServoClass(Names.intakePitch, false));
//        servoClassMap.put("armPitch", new ServoClass(Names.armPitch, false));
//        servoClassMap.put("", new ServoClass(Names.outtake1, false));
//        servoClassMap.put("", new ServoClass(Names.outtake2, false));
//        servoClassMap.put("", new ServoClass(Names.outtakeRotate, false));
//        servoClassMap.put("", new ServoClass(Names.outtakeGrab, false));
    }

    private void setImu() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    private void servoInit() {
        setServoPos(Names.colorServo, 0);
        setServoPos(Names.intakePitch, 0);
        setServoPos(Names.armPitch, 0);
    }

    @Override
    public void run() {
        while (true) {
            deltaTime.reset();

            lynxModuleUpdate();

            updateMotorPosition();
            updateServoPosition();

            imuAngles = imu.getRobotYawPitchRollAngles();

            lastLoopTime = deltaTime.milliseconds();
        }
    }

    private void updateMotorPosition() {
        for (MotorClass motorClass: motorClassMap.values()) {
            motorClass.pos = motorClass.motor.getCurrentPosition();
        }
    }
    private void updateServoPosition() {
        for (ServoClass servoClass: servoClassMap.values()) {
            servoClass.pos = servoClass.servo.getPosition();
        }
    }

    public int getMotorPos(Names name) {
        return motorClassMap.get(nameHashMap.get(name)).pos;
    }
    public double getServoPos(Names name) {
        return servoClassMap.get(nameHashMap.get(name)).pos;
    }

    public YawPitchRollAngles getImuAngles() {
        return imuAngles;
    }
    public void resetImuYaw() {imu.resetYaw();}

    public void setMotorPower(Names name, double power) {
        motorClassMap.get(nameHashMap.get(name)).motor.setPower(power);
    }
    public void setServoPos(Names name, double pos) {
        servoClassMap.get(nameHashMap.get(name)).servo.setPosition(pos);
    }

    private void lynxModuleUpdate() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}