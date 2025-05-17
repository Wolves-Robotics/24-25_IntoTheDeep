package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.RobotHardware;

import java.util.function.BooleanSupplier;

public class ColorSensor2Thing extends CommandBase {
    private double delay;
    private ElapsedTime elapsedTime;
    private ColorSensor colorSensor;
    private boolean done;

    public ColorSensor2Thing(double _delay, Telemetry telemetry) {
        delay = _delay;

        elapsedTime = new ElapsedTime();

        colorSensor = RobotHardware.getInstance().getHardwareMap().colorSensor.get("color2");

        done = false;
    }

    @Override
    public void execute() {
        done = elapsedTime.milliseconds() > delay || (
                colorSensor.red() >= 3000 &&
                colorSensor.green() >= 3500 &&
                colorSensor.blue() >= 900);

    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
