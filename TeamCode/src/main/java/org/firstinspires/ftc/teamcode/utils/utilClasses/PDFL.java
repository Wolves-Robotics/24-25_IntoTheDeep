package org.firstinspires.ftc.teamcode.utils.utilClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;
import java.util.function.IntSupplier;

public class PDFL {
    private IntSupplier input;
    private SetMotorPower output;
    private double kP, kD, kF, kL;
    private int deadZone;
    private int target, pos;
    private double power;

    private ElapsedTime elapsedTime = new ElapsedTime();

    private RingBuffer<Double> timeBuffer = new RingBuffer<>(3, 0.0);
    private RingBuffer<Integer> errorBuffer = new RingBuffer<>(3, 0);

    public PDFL(IntSupplier _input, SetMotorPower _output) {
        input = _input;
        output = _output;

        setCoefficients(0, 0, 0, 0, 0);
        target = 0;
        pos = 0;
    }

    public void setCoefficients(double _kP, double _kD, double _kF, double _kL, int _deadZone) {
        kP = _kP;
        kD = _kD;
        kF = _kF;
        kL = _kL;

        deadZone = _deadZone;
    }

    public void setTarget(int _target) {
        target = _target;
    }

    public void update() {
        pos = input.getAsInt();
        int error = target - pos;
        int prevError  = errorBuffer.getValue(error);

        double time = elapsedTime.time(TimeUnit.MILLISECONDS);
        double prevTime = timeBuffer.getValue(time);

        double deltaTime = time - prevTime;
        int deltaError = error - prevError;

        if (deltaTime > 200) {
            reset();
            update();
        }

        double p = pComponent(error);
        double d = dComponent(deltaError, deltaTime);
        double f = fComponent();
        double l = lComponent(error);

        power = p + d + f + l;

        if (Math.abs(error) < deadZone) {
            power -= l;
        }

        if (target == 0 && 2 < pos && pos < 15) {
            power -= 0.1;
        }

        power = Math.max(-0.7, power);

        output.setMotorPower(power);
    }

    private void reset() {
        timeBuffer.fill(0.0);
        errorBuffer.fill(0);
        elapsedTime.reset();
    }

    private double pComponent(int error) {
        return kP * error;
    }

    private double dComponent(int deltaError, double deltaTime) {
        double derivative = deltaError / deltaTime;

        return derivative * kD;
    }

    private double fComponent() {
        return kF;
    }

    private double lComponent(double error) {
        return Math.signum(error) * kL;
    }

    public int getPos() {
        return input.getAsInt();
    }

    public double getPower() {
        return power;
    }


}