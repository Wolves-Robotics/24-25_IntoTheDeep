package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.Names;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;
import org.firstinspires.ftc.teamcode.utils.utilClasses.PIDF;

public class OuttakeSubsystem extends Thread{
    private static OuttakeSubsystem instance = null;
    private RobotHardware robotHardware;
    private PIDController pidController;
    private ElapsedTime elapsedTime;
    private PIDF pidf;
    private int target;
    private boolean pidOn;
    private double power, pos;

    public static void reset() {
        instance = new OuttakeSubsystem();
    }

    public static OuttakeSubsystem getInstance() {
        return instance;
    }

    private OuttakeSubsystem() {
        robotHardware = RobotHardware.getInstance();
        pidController = new PIDController(Constants.op, Constants.oi, Constants.od);

        pidf = new PIDF(
                Constants.op,
                Constants.oi,
                Constants.od,
                Constants.of,
                () -> (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake))/2,
                x -> {robotHardware.setMotorPower(Names.leftOuttake, x); robotHardware.setMotorPower(Names.rightOuttake, x);}
        );

        target = 0;
        pidOn = false;
        power = 0;

        elapsedTime.reset();

        start();
//        pidf.start();
    }

    public void setTarget(int _target) {target = Math.max(Math.min(_target, Constants.outtakeMaxTarget), Constants.outtakeMinTarget);}
    public int getTarget() {return target;}
    public double getPos() {return pos;}

    public void clawOpen() {robotHardware.setServoPos(Names.claw, 0.35);}
    public void clawClose() {robotHardware.setServoPos(Names.claw, 0);}

    public void clawDown() {
        robotHardware.setServoPos(Names.outtakeArm, 0.05);
        robotHardware.setServoPos(Names.outtakePivot, 0.15);
    }
    public void clawNeutral() {
        robotHardware.setServoPos(Names.outtakeArm, 0.2);
        robotHardware.setServoPos(Names.outtakePivot, 0.1);
    }
    public void clawSample() {
        robotHardware.setServoPos(Names.outtakeArm, 0.45);
        robotHardware.setServoPos(Names.outtakePivot, 0.4);
    }
    public void clawSpecimenGrab() {
        robotHardware.setServoPos(Names.outtakeArm, 0.74);
        robotHardware.setServoPos(Names.outtakePivot, 0.29);
    }
    public void clawSpecimenPlace() {
        robotHardware.setServoPos(Names.outtakeArm, 0.76);
        robotHardware.setServoPos(Names.outtakePivot, 0.35);
    }

    public void startPid() {
        target = 0;
        pidOn = true;
    }
    public void stopPid() {
        pidOn = false;
    }

    public void resetTimer() {
        elapsedTime.reset();
    }

    public double getSeconds() {
        return elapsedTime.seconds();
    }

    @Override
    public void run() {
        while (!currentThread().isInterrupted()) {
            if (pidOn) {
                pos = (robotHardware.getMotorPos(Names.leftOuttake) + robotHardware.getMotorPos(Names.rightOuttake)) / 2.;
                power = pidController.calculate(pos, target) + Constants.of;
                robotHardware.setMotorPower(Names.leftOuttake, power);
                robotHardware.setMotorPower(Names.rightOuttake, power);
            }
        }
    }

    public void updateTelemetry(Telemetry telemetry) {
        telemetry.addData("Outtake pos", pos);
        telemetry.addData("Outtake power", power);
    }
}
