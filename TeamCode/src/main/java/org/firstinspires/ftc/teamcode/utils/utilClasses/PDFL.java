package org.firstinspires.ftc.teamcode.utils.utilClasses;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class PDFL {
    private double kP, kD, kF, kL;
    private int target;

    private ElapsedTime elapsedTime = new ElapsedTime();

    private RingBuffer<Double> timeBuffer = new RingBuffer<>(3, 0.0);
    private RingBuffer<Integer> errorBuffer = new RingBuffer<>(3, 0);

    public PDFL(double _kP, double _kD, double _kF, double _kL) {
        kP = _kP;
        kD = _kD;
        kF = _kF;
        kL = _kL;
    }

    public double run(int position) {
        int error = target - position;
        int prevError  = errorBuffer.getValue(error);

        double time = elapsedTime.time(TimeUnit.MILLISECONDS);
        double prevTime = timeBuffer.getValue(time);

        double deltaTime = time - prevTime;
        int deltaError = error - prevError;

        if (deltaTime > 200) {
            reset();
            return run(position);
        }

        double p = pComponent(error);
        double d = dComponent(deltaError, deltaTime);
        double f = fComponent();
        double l = lComponent(error);

        double response = p + d + f + l;

        return response;
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


}
