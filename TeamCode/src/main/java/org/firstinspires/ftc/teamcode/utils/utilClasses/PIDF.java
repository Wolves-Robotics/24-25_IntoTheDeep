package org.firstinspires.ftc.teamcode.utils.utilClasses;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.IntSupplier;

public class PIDF extends Thread {
    private double p, i, d, f;
    private IntSupplier input;
    private Motors motors;
    private double deltaTime;
    private int target = 0;
    private int error=0, prevError=0;
    private double totalError=0d, errorSlope=0d;
    private double power;
    private boolean running = true;
    private double currentTime, lastTime=0;

    public PIDF(double _p, double _i, double _d, double _f, IntSupplier _input, Motors _motors) {
        p = _p;
        i = _i;
        d = _d;
        f = _f;
        input = _input;
        motors = _motors;
    }

    public PIDF(double _p, double _i, double _d, IntSupplier _input, Motors _motors) {
        this(_p, _i, _d, 0, _input, _motors);
    }

    public void setCoefficients(double _p, double _i, double _d, double _f) {
        p = _p;
        i = _i;
        d = _d;
        f = _f;
    }

    public void setCoefficients(double _p, double _i, double _d) {
        setCoefficients(_p, _i, _d, 0);
    }

    public void setTarget(int _target) {
        target = _target;
        error = 0;
        prevError = 0;
        totalError = 0;
    }

    public void enable() {
        target = input.getAsInt();
        running = true;
    }

    public void disable() {
        running = false;
    }

    @Override
    public void run() {
        while (!currentThread().isInterrupted()) {
            currentTime = (double) System.nanoTime() / 1E9;
            if (lastTime == 0) lastTime = currentTime;
            deltaTime = currentTime - lastTime;
            lastTime = currentTime;
            prevError = error;
            error = target - input.getAsInt();

            totalError += 0.5 * (error + prevError) * deltaTime;

            errorSlope = deltaTime > 1E-6 ? (error - prevError) / deltaTime : 0;
            power = p * error + i * totalError + d * errorSlope + f;
            if (running) motors.setPowers(power);
        }
    }

    public void updateTelemetry(MultipleTelemetry telemetry, String pid) {
        telemetry.addData(pid + " position", input.getAsInt());
        telemetry.addData(pid + " target", target);
        telemetry.addData(pid + " power", power);
        telemetry.addData(pid + " error", error);
        telemetry.addData(pid + " previous error", prevError);
        telemetry.addData(pid + " delta time", deltaTime);
        telemetry.addData(pid + " total error", totalError);
        telemetry.addData(pid + " error slope", errorSlope);
    }
}
