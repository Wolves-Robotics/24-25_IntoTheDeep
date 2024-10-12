package org.firstinspires.ftc.teamcode.autonomous.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.enums.Color;
import org.firstinspires.ftc.teamcode.autonomous.enums.StartPos;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

abstract public class BaseAuto extends OpMode {
    private RobotHardware robotHardware;
    private Follower follower;
    protected Color color;
    protected StartPos startPos;

    abstract void setVars();

    @Override
    public void init() {
        setVars();
        robotHardware = new RobotHardware(hardwareMap, color);
    }

    @Override
    public void loop() {
    }
}
