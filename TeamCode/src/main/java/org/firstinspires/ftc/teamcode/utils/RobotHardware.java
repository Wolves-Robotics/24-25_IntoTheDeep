package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utils.Constants.Names;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.HashMap;
import java.util.List;

@Config
public class RobotHardware extends Thread {
    private void setHardwareMap() {
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
        servoClassMap.put(Names.clawPivot,     new ServoClass(Names.clawPivot,     false));
        servoClassMap.put(Names.claw,          new ServoClass(Names.claw,          false));
    }

    private static HardwareMap hardwareMap;
    private List<LynxModule> allHubs;

    private HashMap<Names, MotorClass> motorClassMap;
    private HashMap<Names, ServoClass> servoClassMap;

    private IMU imu;
    private YawPitchRollAngles imuAngles;

    private ElapsedTime deltaTime;

    private static class MotorClass {
        DcMotor motor;

        public MotorClass(Names name, boolean isReverse) {
            motor = hardwareMap.dcMotor.get(Constants.getStringName(name));
            if (isReverse) motor.setDirection(DcMotor.Direction.REVERSE);
        }
    }

    private static class ServoClass {
        ServoImplEx servo;

        public ServoClass(Names name, boolean isReverse) {
            servo = hardwareMap.get(ServoImplEx.class, Constants.getStringName(name));
            if (isReverse) servo.setDirection(Servo.Direction.REVERSE);
        }
    }

    public RobotHardware(HardwareMap _hardwareMap) {
        hardwareMap = _hardwareMap;
        deltaTime = new ElapsedTime();

        lynxModuleInit();

        setHardwareMap();

        setImu();
    }

    private void lynxModuleInit() {
        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private void setImu() {
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void servoInit() {
        setServoPos(Names.door, 0.7);
        setServoPos(Names.intakeArm, 0.135);
        setServoPos(Names.intakePivot, 0.19);
        setServoPos(Names.outtakeArm, 0.1);
        setServoPos(Names.outtakePivot, 0.15);
        setServoPos(Names.clawPivot, 0.2);
        setServoPos(Names.claw, 0.3);
    }

    @Override
    public void run() {
        while (true) {
            deltaTime.reset();

            lynxModuleUpdate();
        }
    }

    public int getMotorPos(Names name) {
        return motorClassMap.get(name).motor.getCurrentPosition();
    }
    public double getServoPos(Names name) {
        return servoClassMap.get(name).servo.getPosition();
    }

    public YawPitchRollAngles getImuAngles() {
        return imu.getRobotYawPitchRollAngles();
    }
    public void resetImuYaw() {imu.resetYaw();}

    public void setMotorPower(Names name, double power) {
        motorClassMap.get(name).motor.setPower(power);
    }
    public void setServoPos(Names name, double pos) {
        servoClassMap.get(name).servo.setPosition(pos);
    }

    private void lynxModuleUpdate() {
        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }
    }
}