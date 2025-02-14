package org.firstinspires.ftc.teamcode.auto.classes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.collections.Color;

abstract public class BaseAuto {
    protected final Color color;

    protected BaseAuto(Color _color) {
        color = _color;
    }

    abstract public void update();
    abstract public void updateTelemetry(Telemetry telemetry);
}
